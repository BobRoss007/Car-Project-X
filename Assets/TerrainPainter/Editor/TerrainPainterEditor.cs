using UnityEditor;
using UnityEngine;
using Editor = UnityEditor.Editor;
using TP = TerrainPainter.TerrainPainter;

namespace TerrainPainter
{
    [CustomEditor(typeof(TP))]
    public class TerrainPainterEditor : Editor
    {
        #region Editor Things
        private GUIContent[] m_menuToolbarItems;
        private GUIContent m_undoIcon;
        private GUIContent m_redoIcon;

        private GUIContent[] MenuToolbarItems
        {
            get
            {
                if (m_menuToolbarItems == null || m_menuToolbarItems.Length < 1)
                {
                    m_menuToolbarItems = new GUIContent[] {
                        //tp_Editor.IconContent("PaintHeight", "PaintHeight"), // This is for later
                        tp_Editor.IconContent("PaintTexture", "Texture"),
                        tp_Editor.IconContent("PaintDetails", "PaintDetails"),
                        //tp_Editor.IconContent("PaintObjects", "Ramp"), // This is for later
                        tp_Editor.IconContent("Settings", "Settings")
                    };
                }
                return m_menuToolbarItems;
            }
        }
        private GUIContent UndoIcon { get { return m_undoIcon ?? (m_undoIcon = new GUIContent(tp_Editor.IconContent("Undo", "Undo"))); } }
        private GUIContent RedoIcon { get { return m_redoIcon ?? (m_redoIcon = new GUIContent(tp_Editor.IconContent("Redo", "Redo"))); } }

        #endregion

        #region Components
        private tp_Projector m_projector;
        private tp_HeightmapEditor m_heightmapEditor;
        private tp_SplatmapEditor m_splatmapEditor;
        private tp_FoliageEditor m_foliageEditor;
        private tp_SettingsEditor m_settings;


        private TP component { get { return (TP)target as TP; } }
        #endregion

        #region Base Methods
        private void Initialize()
        {
            if (m_projector == null) { m_projector = new tp_Projector(); }
            if (m_heightmapEditor == null) { m_heightmapEditor = new tp_HeightmapEditor(); }
            if (m_splatmapEditor == null) { m_splatmapEditor = new tp_SplatmapEditor(); }
            if (m_foliageEditor == null) { m_foliageEditor = new tp_FoliageEditor(); }
            if (m_settings == null) { m_settings = new tp_SettingsEditor(); }
            component.Initialize();
        }
        private void OnEnable()
        {
            Initialize();
            component.Alpha.Update();
        }
        private void OnDisable()
        {
            if (m_projector != null && m_projector.Projector != null) { m_projector.DisposeProjector(); }
        }
        /// <summary>
        /// Lets the Editor handle an event in the scene view
        /// http://docs.unity3d.com/ScriptReference/Editor.OnSceneGUI.html
        /// </summary>
        public void OnSceneGUI()
        {
            Initialize();

            // 
            tp_ComponentBaseEditor target = null;
            string undoMsg = string.Empty;

            switch (component.EditingTool)
            { // Update the current menu
                case EditingTool.Height:
                    m_heightmapEditor.OnSceneGUI(component);
                    undoMsg = TP.UndoMessages.HEIGHTMAP;
                    target = m_heightmapEditor;
                    break;
                case EditingTool.Texture:
                    m_splatmapEditor.OnSceneGUI(component);
                    undoMsg = TP.UndoMessages.TEXTURE;
                    target = m_splatmapEditor;
                    break;
                case EditingTool.Foliage:
                    m_foliageEditor.OnSceneGUI(component);
                    undoMsg = TP.UndoMessages.FOLIAGE;
                    target = m_foliageEditor;
                    break;
                case EditingTool.Object: break;
                case EditingTool.Settings: m_settings.OnSceneGUI(component); break;
            }

            // Update projector
            if (Event.current.type == EventType.Repaint) { m_projector.Update(component.IsPainting); }
            if (target == null || !component.Alpha.Enabled) { return; }
            component.brushPosition = Vector3.zero;
            bool wasPainting = component.IsPainting;
            tp_Input.Update(component, target, ref component.brushPosition, ref wasPainting, undoMsg);
            component.IsPainting = wasPainting;
        }
        public override void OnInspectorGUI()
        {
            Initialize();
            if (component.TerrainData == null) { tp_Editor.DrawHelpBox("Error:", "Please make sure you added the script to a terrain and \"TerrainData\" is attached to the \"Terrain\" component."); return; }
            DrawMenu();
            switch (component.EditingTool)
            {
                case EditingTool.Height: m_heightmapEditor.OnInspectorGUI(component); break;
                case EditingTool.Texture: m_splatmapEditor.OnInspectorGUI(component); break;
                case EditingTool.Foliage: m_foliageEditor.OnInspectorGUI(component); break;
                case EditingTool.Object: break;
                case EditingTool.Settings: m_settings.OnInspectorGUI(component); break;
            }
            DrawLogo();
            if (GUI.changed) { EditorUtility.SetDirty(this); }
        }
        #endregion

        #region Main Toolbar
        private void DrawMenu()
        {
            GUILayoutOption[] options = new GUILayoutOption[]
            {
                GUILayout.Height(tp_Editor.height),
                GUILayout.MaxHeight(tp_Editor.height),
                GUILayout.Width(tp_Editor.width)
            };

            GUILayout.BeginHorizontal();
            DrawUndo(options);
            tp_Editor.FlexibleSpace(5);

            // Simple version if everything is working
            //component.EditingTool = (EditingTool)GUILayout.Toolbar((int)component.EditingTool, MenuToolbarItems, tp_Editor.ToolbarOptions(MenuToolbarItems.Length));

            // Currest version
            int currentSelected;
            switch (component.EditingTool)
            {
                //case EditingTool.Height: currentSelected = 0; break;
                case EditingTool.Texture: currentSelected = 0; break;
                case EditingTool.Foliage: currentSelected = 1; break;
                case EditingTool.Settings: currentSelected = 2; break;
                default: currentSelected = -1; break;
            }

            currentSelected = GUILayout.Toolbar(currentSelected, MenuToolbarItems, tp_Editor.ToolbarOptions(MenuToolbarItems.Length));

            switch (currentSelected)
            {
                //case 0 : component.EditingTool = EditingTool.Height   ; break;
                case 0 : component.EditingTool = EditingTool.Texture  ; break;
                case 1 : component.EditingTool = EditingTool.Foliage  ; break;
                case 2 : component.EditingTool = EditingTool.Settings ; break;
                default: component.EditingTool = EditingTool.None     ; break;
            }

            tp_Editor.FlexibleSpace(5);
            DrawRedo(options);
            GUILayout.EndHorizontal();
        }
        private void DrawUndo(GUILayoutOption[] options)
        {
            GUI.enabled = component.History.CanUndo();
            if (GUILayout.Button(UndoIcon, options)) { component.History.Undo(); }
            GUI.enabled = true;
        }
        /// <summary>
        /// Redo button
        /// </summary>
        private void DrawRedo(GUILayoutOption[] options)
        {
            GUI.enabled = component.History.CurrentIndex < component.History.CurrentCapacity;
            if (GUILayout.Button(RedoIcon, options)) { component.History.Redo(); }
            GUI.enabled = true;
        }
        #endregion

        private void DrawLogo()
        {
            Rect r = tp_Editor.CurrentRect(30);
            GUIStyle style = new GUIStyle()
            {
                fontStyle = FontStyle.Bold
            };

            tp_Editor.Seperator(3);
            EditorGUILayout.BeginHorizontal();
            tp_Editor.FlexibleSpace(5);
            GUILayout.Label(new GUIContent("                   ", "Thank you for buying Terrain Painter!"), style);
            EditorGUI.DropShadowLabel(r, "HOEKKII");
            tp_Editor.FlexibleSpace(5);
            EditorGUILayout.EndHorizontal();
            tp_Editor.Seperator(1);
        }
    }
}
