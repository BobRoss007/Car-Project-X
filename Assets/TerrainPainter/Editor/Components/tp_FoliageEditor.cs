////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_FoliageEditor.cs
//      Author:             HOEKKII
//      
//      Description:        Editor for Foliage/Detail editing
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;
using UnityEditor;
using TP = TerrainPainter.TerrainPainter;
using System;
using TerrainPainter.Enums;

namespace TerrainPainter
{
    public class tp_FoliageEditor : tp_ComponentBaseEditor
    {
        private EditWindow m_detailAddMenu = EditWindow.None;
        private DetailPrototype m_detailContainer;
        
        internal override void OnSceneGUI(TP c)
        {
            base.OnSceneGUI(c);
        }
        internal override void OnInspectorGUI(TP c)
        {
            base.OnInspectorGUI(c);

            component.Foliages.Erase = DrawSwitch(component.Foliages.Erase , "paint", "erase");
            DrawFoliageSelectorEditor();
            DrawAlphaSelector(component, TP.Colors.SelectedPaintTextureColor);
            DrawSlopeEditor(component, null);
            DrawNoiseEditor(component, component.TerrainData.detailResolution);
            
            if (!component.HidePaintAll) { DrawPaintAllButton(); }
            
        }
        private void DrawPaintAllButton()
        {
            GUIStyle style = new GUIStyle(GUI.skin.button);
            style.fontStyle = FontStyle.Bold;
            style.fontSize = 14;

            Seperator(2);
            // Button, paint!
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            if (GUILayout.Button("Paint", style))
            {
                if (component.Noise.GenerateOnClick) { component.Noise.Generate(new Point(component.TerrainData.alphamapWidth, component.TerrainData.alphamapHeight)); }
                component.History.CreateRestorePoint(UndoType.DetailMap, TP.UndoMessages.FOLIAGE);
                Paint(true);
            }
            //GUI.enabled = true;
            FlexibleSpace(5);
            EditorGUILayout.EndHorizontal();
        }
        
        /// <summary>
        /// Paint method
        /// </summary>
        /// <param name="paintAll"></param>
        internal override void Paint(bool paintAll)
        {
            if (component.Terrain == null) { return; }
            if (component.TerrainData.detailPrototypes.Length < 1) { return; }

	        Point brushSize;
	        Point position, startPosition;
            Texture2D alphaTexture = null;
            
            if (paintAll)
            { // Select whole terrain
                startPosition.x = startPosition.y = 0;
                position.x = position.y = 0;
				int size = component.TerrainData.detailResolution;
				brushSize = new Point(size, size);
			}
            else
            { // Select selected part of terrain
                if (component.Alpha == null || component.Alpha.alphaBrushes == null || component.Alpha.alphaBrushes.Count < 1) { return; }
                alphaTexture = (Texture2D)component.Alpha;
                try { alphaTexture.GetPixel(0, 0); }
                catch { throw new AccessViolationException(TP.ErrorMessages.TEXTURE_NOT_ACCESSABLE); }
				
				brushSize = new Point(
					tp_Utilities.WorldToDetail(component.BrushSize * component.BrushSizeMultiplier, component.TerrainData.size.x, component.TerrainData),
					tp_Utilities.WorldToDetail(component.BrushSize * component.BrushSizeMultiplier, component.TerrainData.size.z, component.TerrainData));
				Point halfBrushSize = brushSize / 2;

                Point center = new Point(
                    tp_Utilities.WorldToDetail(component.brushPosition.x - component.Transform.position.x, component.TerrainData),
                    tp_Utilities.WorldToDetail(component.brushPosition.z - component.Transform.position.z, component.TerrainData.size.z, component.TerrainData));
                
                position = center - halfBrushSize;
                startPosition = Point.Max(position, Point.zero);
            }

            // Set human readable format to an easier modifiable format
            float opacity = component.Opacity / 100.0f;

            // get offset middle brush position to upper left brush position
            Point offset = startPosition - position;
            
            // Some memory stuff, see tp_SplatmapEditor.cs for reason
            Point current;
            float height, angle, alpha;
            float detailmapResolution = component.TerrainData.detailResolution;
            int x, y;
            
            for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++)
            { // For all details

                if (!component.Foliages.IsSelected(i)) { continue; }
                int[,] data = component.TerrainData.GetDetailLayer(
                    startPosition.x, startPosition.y,
                    Mathf.Max(0, Mathf.Min(position.x + brushSize.x, component.TerrainData.detailResolution) - startPosition.x),
                    Mathf.Max(0, Mathf.Min(position.y + brushSize.y, component.TerrainData.detailResolution) - startPosition.y), i);
                for (y = 0; y < data.GetLength(1); y++)
                {
                    for (x = 0; x < data.GetLength(0); x++)
                    {
                        current = new Point(y, x);
                        alpha = paintAll ? 1.0f : opacity * GetAlpha(current + offset, brushSize, alphaTexture);
                        if (component.Foliages.UseRandom) { alpha *= UnityEngine.Random.value; }

                        if (component.Foliages.Erase)
                        { // Erase, no need for calculating all the other stuff
                            data[x, y] = Mathf.RoundToInt(Mathf.Lerp(data[x, y], -component.CurrentFoliageStrength, alpha));
                            continue;
                        }

                        // Get height and angle
                        GetBaseData(current + startPosition, detailmapResolution, out height, out angle);

                        // Mutliply angle value
                        alpha *= 1.0f - angle;
                        // Mulktiply noise value
                        alpha *= GetNoiseValue(current + startPosition, detailmapResolution);
                        // Aplly "pixel"
                        data[x, y] = Mathf.RoundToInt(Mathf.Lerp(data[x, y], component.CurrentFoliageStrength, alpha));//(int)(Mathf.Clamp((paintAll ? 0 : data[x, y]) + alpha * 5.0f, 0.0f, 3.1f));
                    }
                }
                // Apply
                component.TerrainData.SetDetailLayer(startPosition.x, startPosition.y, i, data);
            }
        }

        private void DrawFoliageSelectorEditor()
        {
            // Draw box
            Rect position = EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);

            // Draw label
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(10);
            GUILayout.Label("Foliage", EditorStyles.boldLabel);
            FlexibleSpace(10);
            EditorGUILayout.EndHorizontal();

            // Settings
            DrawBrushSettings();

            // Strength
            component.CurrentFoliageStrength = EditorGUILayout.IntSlider(new GUIContent("Target Strength", "Target detail density"), component.CurrentFoliageStrength, 0, component.MaxFoliageStrength, new GUILayoutOption[0]);

            // MultiSelectEnebled
            if (m_detailAddMenu == EditWindow.Edit) { GUI.enabled = false; }
            component.Foliages.IsMultiSelect = EditorGUILayout.Toggle(new GUIContent("Multi Selecting Enabled", "Enable painting multiple details at the same time"), component.Foliages.IsMultiSelect, new GUILayoutOption[0]);
            GUI.enabled = true;
            component.Foliages.UseRandom = EditorGUILayout.Toggle(new GUIContent("Use Random", "Use random opacity"), component.Foliages.UseRandom, new GUILayoutOption[0]);
            position.y += 100;

            // Settings
            Color tempBackColor = GUI.backgroundColor;
            const int imgSize = 50;
            const int spacing = 4;
            float totalSpacing = spacing;
            position.width = Screen.width - (20.0f + spacing * 2.0f);

            // Set position
            Rect current = position;
            current.width = current.height = imgSize;
            current.x += spacing;
            current.y += spacing;

            // Get amount of columns
            if (m_detailAddMenu == EditWindow.Edit) { GUI.enabled = false; }
            int columnCount = (int)((position.width - (2 * spacing)) / (spacing + imgSize));

            for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++)
            {
                GUI.backgroundColor = TP.Colors.UnSelectedColor;
                if (i > 0 && i % columnCount == 0)
                {
                    // Do new line
                    totalSpacing += imgSize + spacing;
                    current.x = position.x + spacing;
                    current.y = position.y + totalSpacing;
                }
                else if (i != 0) { current.x += imgSize + spacing; }
                if (component.Foliages.IsSelected(i)) { GUI.backgroundColor = TP.Colors.SelectedFoliageColor;  }
                if (GUI.Button(current, component.TerrainData.detailPrototypes[i].prototypeTexture))
                {
                    // Set selected texture
                    if (component.Foliages.IsMultiSelect)
                    {
                        if (component.Foliages.IsSelected(i)) { component.Foliages.SelectedFoliages &= ~(1 << i); }
                        else { component.Foliages.SelectedFoliages |= 1 << i; }
                    }
                    else { component.Foliages.SelectedFoliage = i; }
                }
            }

            // Reset settings
            GUI.enabled = true;
            GUI.backgroundColor = tempBackColor;
            totalSpacing += imgSize + spacing;
            current.x = position.x + spacing;
            current.y = position.y + totalSpacing;
            current.width = position.width;
            GUILayout.Space(totalSpacing);

            switch (m_detailAddMenu)
            { // Some edit stuff, same as tp_splatmapeditor.cs
                case EditWindow.Add: AddPrototypeEditor(); break;
                case EditWindow.Edit: EditPrototypeEditor(); break;
                case EditWindow.Remove: RemovePrototypeEditor(); break;
                default: CostomizeButtonsEditor(); break;
            }

            EditorGUILayout.EndVertical();
        }
        private void CostomizeButtonsEditor()
        {
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            if (GUILayout.Button(new GUIContent("add", "Add new detail")))
            {
                m_detailContainer = DefaultPrototype;
                m_detailAddMenu = EditWindow.Add;
            }
            int canEdit = -1;
            for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++) { if (component.Foliages.IsSelected(i)) { canEdit++; } }
            GUI.enabled = canEdit == 0;
            if (GUILayout.Button(new GUIContent("edit", "Edit current selected detail")))
            {
                for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++)
                {
                    if (component.Foliages.IsSelected(i))
                    {
                        m_detailContainer = component.TerrainData.detailPrototypes[i];
                        break;
                    }
                }
                m_detailAddMenu = EditWindow.Edit;
            }
            GUI.enabled = canEdit >= 0;
            if (GUILayout.Button(new GUIContent("remove", "remove current selected " + (canEdit > 0 ? "textures" : "texture"))))
            {
                m_detailAddMenu = EditWindow.Remove;
            }
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();
        }
        private void AddPrototypeEditor()
        {
            DefaultEditProtoType();
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            GUI.enabled = m_detailContainer.prototypeTexture != null || m_detailContainer.prototype != null; // thing is not null
            if (GUILayout.Button(new GUIContent("add", "Apply and add the current configured detail")))
            {
                AddSplatMap(m_detailContainer);
                CancelPrototypeEditor();
            }
            GUI.enabled = true;
            if (GUILayout.Button(new GUIContent("cancel", "Cancel current configured detail"))) { CancelPrototypeEditor(); }
            EditorGUILayout.EndHorizontal();
        }
        private void EditPrototypeEditor()
        {
            DefaultEditProtoType();
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            GUI.enabled = m_detailContainer.prototypeTexture != null || m_detailContainer.prototype != null;
            if (GUILayout.Button(new GUIContent("edit", "Apply changes to the current selected detail")))
            {
                for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++)
                {
                    if (component.Foliages.IsSelected(i))
                    {
                        ReplaceSplatMap(m_detailContainer, i);
                        break;
                    }
                }
                CancelPrototypeEditor();
            }
            GUI.enabled = true;
            if (GUILayout.Button(new GUIContent("cancel", "Cancel changes of the current selected detail"))) { CancelPrototypeEditor(); }
            EditorGUILayout.EndHorizontal();
        }
        private void RemovePrototypeEditor()
        {
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label("Are you sure?", EditorStyles.boldLabel);
            FlexibleSpace(5);
            if (GUILayout.Button(new GUIContent("yes"))) { for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++) { if (component.Foliages.IsSelected(i)) { RemoveSplatMap(i); } } CancelPrototypeEditor(); }
            if (GUILayout.Button(new GUIContent("no"))) { CancelPrototypeEditor(); }
            EditorGUILayout.EndHorizontal();
        }
        private void CancelPrototypeEditor()
        {
            m_detailContainer = null;
            m_detailAddMenu = EditWindow.None;
        }
        private void DefaultEditProtoType()
        {
            bool prevProtoTypeType = m_detailContainer.usePrototypeMesh;
            m_detailContainer.usePrototypeMesh = DrawSwitch(prevProtoTypeType, "texture", "mesh");

            if (m_detailContainer.usePrototypeMesh)
            {
                if (!prevProtoTypeType)
                { // Reset Mesh to null
                    m_detailContainer.prototype = null;
                    m_detailContainer.renderMode = DetailRenderMode.VertexLit;
                }
                m_detailContainer.prototype = EditorGUILayout.ObjectField("GameObject", m_detailContainer.prototype, typeof(GameObject), false) as GameObject;
            }
            else
            {
                if (prevProtoTypeType)
                { // Reset Texture to null
                    m_detailContainer.prototypeTexture = null;
                    m_detailContainer.renderMode = DetailRenderMode.Grass;
                }
                m_detailContainer.prototypeTexture = EditorGUILayout.ObjectField("Texture", m_detailContainer.prototypeTexture, typeof(Texture2D), false) as Texture2D;
                m_detailContainer.renderMode = EditorGUILayout.Toggle(new GUIContent("Billboard", "Draw texture as billboard"), m_detailContainer.renderMode == DetailRenderMode.GrassBillboard) ? DetailRenderMode.GrassBillboard : DetailRenderMode.Grass;
            }
            m_detailContainer.bendFactor = EditorGUILayout.FloatField(new GUIContent("Bend Factor"), m_detailContainer.bendFactor);
            m_detailContainer.noiseSpread = EditorGUILayout.FloatField(new GUIContent("Noise Spread", "Variation of dry and healty details"), m_detailContainer.noiseSpread);
            m_detailContainer.minWidth = EditorGUILayout.FloatField(new GUIContent("Min Width", "Min detail width"), m_detailContainer.minWidth);
            m_detailContainer.maxWidth = EditorGUILayout.FloatField(new GUIContent("Max Width", "Max detail width"), m_detailContainer.maxWidth);
            m_detailContainer.minHeight = EditorGUILayout.FloatField(new GUIContent("Min Heigth", "Min detail height"), m_detailContainer.minHeight);
            m_detailContainer.maxHeight = EditorGUILayout.FloatField(new GUIContent("Max Height", "Max detail height"), m_detailContainer.maxHeight);
            m_detailContainer.healthyColor = EditorGUILayout.ColorField(new GUIContent("Healthy Color"), m_detailContainer.healthyColor);
            m_detailContainer.dryColor = EditorGUILayout.ColorField(new GUIContent("Dry Color"), m_detailContainer.dryColor);
        }
        private static DetailPrototype DefaultPrototype
        {
            get
            {
                DetailPrototype p = new DetailPrototype();
                p.usePrototypeMesh = false;
                p.renderMode = DetailRenderMode.Grass;
                p.bendFactor = 0.1f;
                p.noiseSpread = 0.0f;
                p.minWidth = 1.0f;
                p.maxWidth = 2.0f;
                p.minHeight = 1.0f;
                p.maxHeight = 2.0f;
                p.healthyColor = new Color(1.0f, 1.0f, 1.0f, 1.0f);
                p.dryColor = new Color(0.7f, 0.6f, 0.5f, 1.0f);
                return p;
            }
        }

        internal override UndoType UndoType
        {
            get
            {
                return UndoType.DetailMap;
            }
        }

        private void RemoveSplatMap(int index)
        {
            DetailPrototype[] newPototypes = new DetailPrototype[component.TerrainData.detailPrototypes.Length - 1];
            for (int i = 0, j = 0; i < component.TerrainData.detailPrototypes.Length; i++)
            {
                if (i == index) { continue; }
                newPototypes[j] = component.TerrainData.detailPrototypes[i];
                j++;
            }
            component.TerrainData.detailPrototypes = newPototypes;
        }
        private void AddSplatMap(DetailPrototype p)
        {
            DetailPrototype[] newPototypes = new DetailPrototype[component.TerrainData.detailPrototypes.Length + 1];
            for (int i = 0; i < component.TerrainData.detailPrototypes.Length; i++) { newPototypes[i] = component.TerrainData.detailPrototypes[i]; }
            newPototypes[newPototypes.Length - 1] = p;
            component.TerrainData.detailPrototypes = newPototypes;
        }
        private void ReplaceSplatMap(DetailPrototype p, int index)
        {
            DetailPrototype[] newPototypes = component.TerrainData.detailPrototypes;
            newPototypes[index] = p;
            component.TerrainData.detailPrototypes = newPototypes;
        }
    }
}
