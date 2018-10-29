////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Editor.cs
//      Author:             HOEKKII
//      
//      Description:        Some frequently used functions
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;
using UnityEditor;
using TP = TerrainPainter.TerrainPainter;
using System;
using System.IO;

namespace TerrainPainter
{
    public class tp_Editor
    {

        public const float minSize = 4.0f; // when 32 = 12
        public const float height = minSize + 24.0f;
        public const float width = minSize + 24.0f;
        
        private TP m_component = null;
        protected TP component { get { return m_component; } }
        internal virtual void OnSceneGUI(TP c)
        {
            m_component = c;
        }
        internal virtual void OnInspectorGUI(TP c)
        {
            m_component = c;
        }

        public static GUILayoutOption[] ToolbarOptions(int items)
        {
            return new GUILayoutOption[]
            {
                GUILayout.Height(height),
                GUILayout.MaxHeight(height),
                GUILayout.Width(width * items),
                GUILayout.MaxWidth(width * items)
            };
        }

        /// <summary>
        /// Adds EditorGUILayout seperators
        /// </summary>
        /// <param name="amount">Amount of seperators</param>
        internal static void Seperator(int amount = 1)
        {
            if (amount > 0)
            {
                EditorGUILayout.Separator();
                Seperator(amount - 1);
            }
        }
        /// <summary>
        /// Add EditorGUILayout FlexibleSpace
        /// </summary>
        /// <param name="amount"></param>
        internal static void FlexibleSpace(int amount = 1)
        {
            if (amount > 1)
            {
                GUILayout.FlexibleSpace();
                FlexibleSpace(amount - 1);
            }
        }

        /// <summary>
        /// See EditorGUIUtilitiy.IconContent
        /// But this is for this DLL
        /// </summary>
        /// <returns></returns>
        internal static GUIContent IconContent(string file, string tooltip = default(string))
        {
            return new GUIContent(LoadIcon(file), tooltip);
        }

        /// <summary>
        /// Get current rect
        /// </summary>
        /// <returns>editor rect</returns>
        internal static Rect CurrentRect()
        {
            Rect rect = EditorGUILayout.BeginVertical();
            EditorGUILayout.EndVertical();
            return rect;
        }
        /// <summary>
        /// Get current rect
        /// </summary>
        /// <param name="height">rect height</param>
        /// <returns>editor rect</returns>
        internal static Rect CurrentRect(float height)
        {
            Rect rect = CurrentRect();
            rect.height = height;
            return rect;
        }
        /// <summary>
        /// Get current rect
        /// </summary>
        /// <param name="width">rect width</param>
        /// <param name="height">rect height</param>
        /// <returns>editor rect</returns>
        internal static Rect CurrentRect(float width, float height)
        {
            Rect rect = CurrentRect();
            rect.width = width;
            rect.height = height;
            return rect;
        }


        /// <summary>
        /// Draw help box
        /// </summary>
        /// <param name="title"></param>
        /// <param name="msg"></param>
        internal static void DrawHelpBox(string title, string msg)
        {
            GUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);
            if (!string.IsNullOrEmpty(title)) { GUILayout.Label(title, new GUILayoutOption[0]); }

            string[] sMsg = msg.Split('\n');
            foreach (string nMsg in sMsg)
            {
                GUILayout.Label(nMsg, EditorStyles.wordWrappedMiniLabel, new GUILayoutOption[0]);
            }
            GUILayout.EndVertical();
        }

        /// <summary>
        /// No tool selected
        /// </summary>
        public static void DrawNone()
        {
            DrawHelpBox("No tool selected", "Please select a tool");
        }

        public static void DrawTitleLabel(string msg)
        {
            GUILayout.Label(new GUIContent(msg), EditorStyles.boldLabel, new GUILayoutOption[0]);
        }
        /// <summary>
        /// Load icon
        /// </summary>
        /// <param name="file"></param>
        /// <returns></returns>
        internal static Texture2D LoadIcon(string file)
        {
            switch (file)
            {
                // Main Toolbar
                case "PaintHeight": return Resources.Load<Texture2D>("TerrainPainter_PaintHeight_Icon");
                case "PaintTexture": return Resources.Load<Texture2D>("TerrainPainter_PaintTexture_Icon");
                case "PaintDetails": return Resources.Load<Texture2D>("TerrainPainter_PaintDetails_Icon");
                case "PaintObjects": return Resources.Load<Texture2D>("TerrainPainter_PaintObjects_Icon");
                case "Settings": return Resources.Load<Texture2D>("TerrainPainterSettingsIcon");

                // Undo/Redo
                case "Undo": return Resources.Load<Texture2D>("TerrainPainterUndoIcon");
                case "Redo": return Resources.Load<Texture2D>("TerrainPainterRedoIcon");

                // Paint Heigt

                // Paint Texture
                case "Paint": return Resources.Load<Texture2D>("TerrainPainterPaintIcon");
                case "Ramp": return Resources.Load<Texture2D>("TerrainPainterRampIcon");
                case "Height": return Resources.Load<Texture2D>("TerrainPainterHeightIcon");
                case "Texture": return Resources.Load<Texture2D>("TerrainPainterPaintIcon");
                case "Sharpen": return Resources.Load<Texture2D>("TerrainPainterSharpIcon");
                case "Soften": return Resources.Load<Texture2D>("TerrainPainterSmoothIcon");

                // Paint Details


                // Paint Objects

                default: return Resources.Load<Texture2D>("TerrainPainterLogIcono");
            }
        }

        public static GUIStyle ButtonDoAllStyle()
        {
            GUIStyle style = new GUIStyle(GUI.skin.button);
            style.fontStyle = FontStyle.Bold;
            style.fontSize = 14;

            return style;
        }
        protected void DrawBrushSettings(bool drawStrength = false)
        {
            // Draw opacity slider
            component.Opacity = EditorGUILayout.Slider(new GUIContent("Opacity"), component.Opacity, 0.0f, 100.0f, new GUILayoutOption[0]);

            // Draw strength slider
            if (drawStrength) { component.Strength = EditorGUILayout.Slider(new GUIContent("Strength"), component.Strength, 0.0f, 100.0f, new GUILayoutOption[0]); }
            else { component.Strength = 100.0f; }
        }
        public void DrawAlphaSelector(TP tp, Color color)
        {
            // Draw label
            Rect position = EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            tp.Alpha.Enabled = GUILayout.Toggle(tp.Alpha.Enabled, new GUIContent());
            DrawTitleLabel("Alpha");
            EditorGUILayout.EndHorizontal();
            Seperator(1);

            // Settings
            Color tempColor = GUI.backgroundColor;

            if (tp.Alpha.Enabled)
            {
                // Size slider
                tp.BrushSize = EditorGUILayout.FloatField(new GUIContent("Size"), tp.BrushSize, new GUILayoutOption[0]);
                tp.BrushRotation = EditorGUILayout.FloatField(new GUIContent("Angle"), tp.BrushRotation);

                DrawTooltip(1, "", new string[] {
                    "Press ctrl and drag mouse to rotate the brush",
                    "Press ctrl and drag mouse to rotate the brush"
                });

                position = EditorGUILayout.BeginVertical(); EditorGUILayout.EndVertical();
                
                const int imgSize = 50;
                const int spacing = 4;
                float totalSpacing = (float)spacing;
                position.width = Screen.width - (20.0f + spacing * 2.0f);

                // Set potition
                Rect current = position;
                current.width = current.height = imgSize;
                current.x += spacing;
                current.y += spacing;

                // Get amount of columns
                int columnCount = (int)((position.width - (2 * spacing)) / (spacing + imgSize));

                for (int i = 0; i < tp.Alpha.alphaBrushes.Count; i++)
                {
                    if (i > 0 && i % columnCount == 0)
                    {
                        // New line
                        totalSpacing += imgSize + spacing;
                        current.x = position.x + spacing;
                        current.y = position.y + totalSpacing;
                    }
                    else if (i != 0) { current.x += imgSize + spacing; }

                    // Set colour, set selected alpha texture
                    if (i == (int)tp.Alpha) { GUI.backgroundColor = color; }
                    else { GUI.backgroundColor = TP.Colors.UnSelectedColor; }
                    if (GUI.Button(current, tp.Alpha.alphaBrushes[i])) { tp.Alpha.selectedAlphaBrush = i; }
                }
                GUI.backgroundColor = tempColor;

                totalSpacing += imgSize + spacing;
                GUILayout.Space(totalSpacing);

                // Update button
                EditorGUILayout.BeginHorizontal();
                FlexibleSpace(5);
                if (GUILayout.Button(new GUIContent("update.."))) { tp.Alpha.Update(); }
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();

            // Reset color
            GUI.color = tempColor;
        }
        public static void DrawNoiseEditor(TP tp, int resolution)
        {
            // Draw label
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            tp.Noise.Enabled = GUILayout.Toggle(tp.Noise.Enabled, new GUIContent());
            DrawTitleLabel("Noise");
            EditorGUILayout.EndHorizontal();
            Seperator(1);

            //tp.Noise.Enabled = EditorGUILayout.ToggleLeft(new GUIContent("Noise"), tp.Noise.Enabled, EditorStyles.boldLabel, new GUILayoutOption[0]);

            // If noise is enabled
            if (tp.Noise.Enabled)
            {
                // Texture preview Settings
                const int previewSize = 100;

                EditorGUILayout.BeginHorizontal();
                Rect current = EditorGUILayout.BeginVertical();

                // Get texture
                Texture2D tex = new Texture2D(previewSize, previewSize);
                tex.hideFlags = HideFlags.HideAndDontSave;

                // Draw preview texture
                if (tp.Noise.Texture != null) { tex = tp.Noise.Texture; }
                current.height = current.width = previewSize;
                GUI.Label(current, new GUIContent(tex, tex.name));

                // Offset
                current.y += previewSize + 2.0f;
                current.height = 16.0f;

                // Draw generate button
                if (GUI.Button(current, new GUIContent("Generate")))
                {
                    //tp.Noise.Generate(new Point(tp.TerrainData.alphamapWidth, tp.TerrainData.alphamapHeight));
                    tp.Noise.Generate(new Point(resolution, resolution));
                }
                EditorGUILayout.EndVertical();
                GUILayout.Space(previewSize + 2.0f);

                // Settings
                EditorGUILayout.BeginVertical();
                // Draw generate on click checkbox
                tp.Noise.GenerateOnClick = EditorGUILayout.Toggle(new GUIContent("Generate on click", "Generate new splatmap on mousebutton down"), tp.Noise.GenerateOnClick, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Scale", "Noise scale"), new GUILayoutOption[0]);
                tp.Noise.Scale = EditorGUILayout.FloatField(tp.Noise.Scale, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Octaves", "Amount of iterations applied"), new GUILayoutOption[0]);
                tp.Noise.Octaves = EditorGUILayout.IntSlider(tp.Noise.Octaves, 1, 10, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Gain", "Gain of each iteration"), new GUILayoutOption[0]);
                tp.Noise.Gain = EditorGUILayout.Slider(tp.Noise.Gain, 0.1f, 0.9f, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Lacunarity", ""), new GUILayoutOption[0]);
                tp.Noise.Lacunarity = EditorGUILayout.Slider(tp.Noise.Lacunarity, 1.5f, 2.5f, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Amplitude", ""), new GUILayoutOption[0]);
                tp.Noise.Amplitude = EditorGUILayout.Slider(tp.Noise.Amplitude, -10, 10.0f, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent("Offset", ""), new GUILayoutOption[0]);
                tp.Noise.Offset = EditorGUILayout.Slider(tp.Noise.Offset, -10f, 10.0f, new GUILayoutOption[0]);

                EditorGUILayout.EndVertical();
                EditorGUILayout.EndHorizontal();


            }
            EditorGUILayout.EndVertical();
        }
        protected void DrawSlopeEditor(TP tp, Action additional)
        {
            // Draw label
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.BeginHorizontal();
            tp.Ramp.Active = GUILayout.Toggle(tp.Ramp.Active, new GUIContent());
            DrawTitleLabel("Ramp");
            EditorGUILayout.EndHorizontal();
            Seperator(1);


            if (tp.Ramp.Active || component.SplatTool == SplatTool.AngleBased)
            {
                // Get current slope values
                float min = component.Ramp.Min;
                float max = component.Ramp.Max;

                GUILayout.BeginHorizontal();
                // Draw label
                GUILayout.Label("Ramp \t");

                Seperator(3);
                min = EditorGUILayout.FloatField(min, new GUILayoutOption[0]);
                GUILayout.Label(new GUIContent(" / "), new GUILayoutOption[0]);
                max = EditorGUILayout.FloatField(max, new GUILayoutOption[0]);
                Seperator(8);
                GUILayout.EndHorizontal();
                
                // Draw start- end-height slider
                EditorGUILayout.MinMaxSlider(ref min, ref max, 0.0f, 90.0f);

                // Fix values if needed
                if (min <= 0.01f) { if (min + 0.01f > max) { max = min + 0.01f; } }
                else if (min + 0.01f > max) { min = max - 0.01f; }
                component.Ramp.Set(Mathf.Clamp(min, 0.01f, 90.0f), Mathf.Clamp(max, 0.00f, 89.99f));
                //DrawSlopeCurve();

                if (additional != null) { additional(); }
            }
            EditorGUILayout.EndVertical();
        }
        
        protected void GetBaseData(Point current, float otherResolution, out float height, out float angle)
        {
            Vector2 normal = current; 
            //normal += Vector2.one; // Add Vector2.one because the resolution is the length (max index + 1) and index starts at 0
            normal /= otherResolution;

            // Height
            height = component.TerrainData.GetHeight(
                Mathf.RoundToInt(normal.x * (float)component.TerrainData.heightmapResolution),
                Mathf.RoundToInt(normal.y * (float)component.TerrainData.heightmapResolution));

            // Angle
            if (component.Ramp.Active)
            {
                angle = component.TerrainData.GetSteepness(normal.x, normal.y);
                angle = component.Ramp.LerpValue(angle);
            }
            else
            {
                angle = 0.0f;
            }
        }
        protected float GetNoiseValue(Point current, float otherResolution)
        {
            if (component.Noise.Enabled == false) { return 1.0f; }
            if (component.Noise.Texture == null) { return 1.0f; }
            if (otherResolution <= 0.0f) { return 1.0f; }

            Vector2 texPos = current;
            texPos /= otherResolution;

            return component.Noise.Texture.GetPixel((int)(texPos.x * component.Noise.Texture.width), (int)(texPos.y * component.Noise.Texture.height)).grayscale;
        }
        protected float GetAlpha(Point pos, Point target, Texture2D tex)
        {
            if (tex == null) { return 1.0f; }
            pos += Point.one;
            if (component.BrushRotation == 0.0f) { return GetAlphaRaw(pos, target, tex); }
            Vector2 halfTarget = (Vector2)target / 2.0f;
            Vector2 origin = pos - halfTarget;
            origin *= component.BrushSizeMultiplier;
            origin = new Vector2(
                origin.x * component.BrushCosAngle - origin.y * component.BrushSinAngle + halfTarget.x,
                origin.x * component.BrushSinAngle + origin.y * component.BrushCosAngle + halfTarget.y);

            if (origin.x < 0.0f || origin.x > target.x || origin.y < 0.0f || origin.y > target.y) { return 0.0f; }

            return GetAlphaRaw(origin, target, tex);
        }
        protected float GetAlphaRaw(Vector2 pos, Vector2 target, Texture2D tex)
        {
            if (tex == null) { return 1.0f; }
	        float x = (pos.x - 1) / target.x;
	        float y = (pos.y - 1) / target.y;

	        //Vector3 size = TP.Instance.TerrainData.size;
			//if (size.z != size.x)
			//{
			//	if (size.x < size.z)
			//		x = (x - 0.5f) * (size.z / size.x) + 0.5f;
			//	else
			//		y = (x - 0.5f) * (size.x / size.z) + 0.5f;
			//}

			return tex.GetPixelBilinear(x, y).a;

            //pos += Vector2.one;
            //return tex.GetPixel(
            //        Mathf.RoundToInt((pos.x / target.x) * tex.width),
            //        Mathf.RoundToInt((pos.y / target.y) * tex.height)).grayscale;
        }
        /// <summary>
        /// Draw brush settings
        /// </summary>
        /// <param name="drawStrength">strengthe enabled</param>
        public static bool DrawSwitch(bool value, string left_false, string right_true)
        {
            int c = value ? 1 : 0;
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(3);
            if (GUILayout.Button(new GUIContent(left_false), EditorStyles.label)) { c = 0; }
            FlexibleSpace(1);
            c = Mathf.RoundToInt(GUI.HorizontalSlider(GUILayoutUtility.GetRect(20, 20), c, 0.49f, 0.51f));
            FlexibleSpace(1);
            if (GUILayout.Button(new GUIContent(right_true), EditorStyles.label)) { c = 1; }
            FlexibleSpace(3);
            EditorGUILayout.EndHorizontal();
            
            return c == 1;
        }

        public void DrawTooltip(string title, params string[] msg)
        {
            DrawTooltip(1, title, msg);
        }
        public void DrawTooltip(int startIndex, string title, params string[] msg)
        {
            if (component.TooltipLevel < startIndex) { return; }
            startIndex = Mathf.Clamp(component.TooltipLevel - startIndex, 0, msg.Length - 1);
            DrawHelpBox(title, msg[startIndex]);
        }

    }
}