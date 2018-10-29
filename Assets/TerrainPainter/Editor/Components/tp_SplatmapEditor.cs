////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_SplatmapEditor.cs
//      Author:             HOEKKII
//      
//      Description:        Splat map editor
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;
using UnityEditor;
using TerrainPainter.Enums;
using TP = TerrainPainter.TerrainPainter;
using System;

namespace TerrainPainter
{
    public class tp_SplatmapEditor : tp_ComponentBaseEditor
    {
        private GUIContent[] m_toolbarItems;

        private GUIContent[] ToolbarItems
        {
            get
            {
                if (m_toolbarItems == null || m_toolbarItems.Length < 1)
                {
                    m_toolbarItems = new GUIContent[] {
                        IconContent("Texture", "Texture"),
                        IconContent("Height", "Height-Based"),
                        IconContent("Ramp", "Ramp"),
                        IconContent("Sharpen", "Sharpen"),
                        IconContent("Soften", "Soften")
                    };
                }
                return m_toolbarItems;
            }
        }

        // Variables used to store temporary data
        private TextureEditContainer m_cliffContainer = new TextureEditContainer();     // Store add/edit cliff texture data
        private TextureEditContainer m_textureContainer = new TextureEditContainer();   // Store add/edit texture data
        private bool m_isAddingHeight = false;                                          // Checks if the add height textures is being added
        private int m_tempHeightIndex = 0;                                              // Used for which textures

        internal override UndoType UndoType
        {
            get
            {
                return UndoType.Splatmap;
            }
        }

        private class TextureEditContainer
        {
            internal EditWindow m_textureWindow = EditWindow.None;
            internal Texture2D m_mainTextureSelector = null;
            internal Texture2D m_normTextureSelector = null;
            internal Vector2 m_textureSize = Vector2.one;
            internal Vector2 m_textureOffset = Vector2.zero;
            internal Vector2 m_textureSpecialSettings = Vector2.zero; // Metalic, Smoothness
            internal Color m_textureSpecularity = Color.black;
        }
        private enum TextureSelectType
        {
            Default = 0,
            Cliff,
            Multi
        }
        
        /// <summary>
        /// Mail paint method
        /// </summary>
        /// <param name="paintAll"></param>
        internal override void Paint(bool paintAll)
        {
            #region Init
            // Safety
            if (component.Terrain == null) { return; }
            if (component.TerrainData.splatPrototypes.Length < 2) { return; }
            if (component.SplatmapLock) { return; }
            if (component.SplatTool == SplatTool.HeightBased && component.Heights.Count < 1) { return; }

            // Data
	        Point brushSize;
	        Point position, startPosition;
            Texture2D alphaTexture = null;
            int splatPrototypesCount = component.TerrainData.splatPrototypes.Length;
            int[] texIndexes = component.Heights.GetAvailablePaintTextures();
            
            if (paintAll)
            { // Get whole terrain
                startPosition.x = startPosition.y = 0;
                position.x = position.y = 0;
	            int size = component.TerrainData.alphamapResolution;
				brushSize = new Point(size, size);
			}
            else
            { // Get selected part of terrain
                if (component.Alpha == null || component.Alpha.alphaBrushes == null || component.Alpha.alphaBrushes.Count < 1) { return; }
                
                // Get alpha texture, and test if it can be used
                alphaTexture = (Texture2D)component.Alpha;
                try { alphaTexture.GetPixel(0, 0); }
                catch { throw new AccessViolationException(TP.ErrorMessages.TEXTURE_NOT_ACCESSABLE); }

				// Brush size
				brushSize = new Point(
					tp_Utilities.WorldToSplat(component.BrushSize * component.BrushSizeMultiplier, component.TerrainData.size.x, component.TerrainData),
					tp_Utilities.WorldToSplat(component.BrushSize * component.BrushSizeMultiplier, component.TerrainData.size.z, component.TerrainData));
				Point halfBrushSize = brushSize / 2;

                Point center = new Point(
                    tp_Utilities.WorldToSplat(component.brushPosition.x - component.Transform.position.x, component.TerrainData.size.x, component.TerrainData),
                    tp_Utilities.WorldToSplat(component.brushPosition.z - component.Transform.position.z, component.TerrainData.size.z, component.TerrainData));

                position = center - halfBrushSize;
                startPosition = Point.Max(position, Point.zero);
            }

            // Get current data
            Point offset = startPosition - position;
            float[,,] alphaMaps = component.TerrainData.GetAlphamaps(
                startPosition.x, startPosition.y,
                Mathf.Max(0, Mathf.Min(position.x + brushSize.x, component.TerrainData.alphamapResolution) - startPosition.x),
                Mathf.Max(0, Mathf.Min(position.y + brushSize.y, component.TerrainData.alphamapResolution) - startPosition.y));

            // Safety
            if (component.Noise.Texture != null) { if (component.Noise.Texture.width != component.TerrainData.alphamapWidth || component.Noise.Texture.height != component.TerrainData.alphamapHeight || !component.Noise.Enabled) { component.Noise.Texture = null; } }
            
            /* 
             * I am not completely sure if this makes any difference
             * Or if the compiler does this automaticly..
             *  
             * But this is for optimasation;
             *      we don't want to create a float 1000000 times, so thats why we do it here
             * 
             */
            float[] prevAlpha = new float[splatPrototypesCount];
            float opacity = component.Opacity / 100.0f;

            Point current;
            float height, angle, alpha, strength;
            float alphamapResolution = (float)component.TerrainData.alphamapResolution;

            int y, x;
            #endregion
            for (y = 0; y < alphaMaps.GetLength(0); y++) for (x = 0; x < alphaMaps.GetLength(1); x++)
            {
                current = new Point(x, y);
                GetBaseData(current + startPosition, alphamapResolution, out height, out angle);

                // Alpha
                alpha = paintAll ? 1.0f : opacity * GetAlpha(current + offset, brushSize, alphaTexture);
                strength = paintAll ? 1.0f : 1.0f;

                if (alpha <= 0.0f) { continue; }
                strength *= GetNoiseValue(current + startPosition, alphamapResolution);
                if (component.SplatTool == SplatTool.AngleBased && (alphaMaps[y, x, component.Textures.SelectedCliffBrush] > angle * strength || strength * angle <= 0.0f)) { continue; }
                if (component.SplatTool == SplatTool.OneTexture)
                {
                    if (alphaMaps[y, x, component.Textures.SelectedTexture] >= (1.0f - angle) * strength &&
                        alphaMaps[y, x, component.Textures.SelectedCliffBrush] >= angle * strength)
                        continue;
                }
                // 
                for (int i = 0; i < splatPrototypesCount; i++)
                {
                    prevAlpha[i] = alphaMaps[y, x, i];
                    alphaMaps[y, x, i] = 0.0f;
                }

                // 
                switch (component.SplatTool)
                {
                    case SplatTool.OneTexture:
                        alphaMaps[y, x, component.Textures.SelectedTexture] = 1.0f - (alphaMaps[y, x, component.Textures.SelectedCliffBrush] = angle);
                        alpha *= strength;
                        break;

                    case SplatTool.HeightBased:
                        alphaMaps[y, x, component.Textures.SelectedCliffBrush] = angle;
                        angle = 1.0f - angle;

                        for (int a = 0; a < texIndexes.Length; a++)
                        {
                            int i = texIndexes[a];

                            if (height < component.Heights[0].y || a == texIndexes.Length - 1)
                            {
                                alphaMaps[y, x, i] += angle;
                                break;
                            }
                            else
                            {
                                int b = a + 1;
                                int j = texIndexes[b];

                                if (height >= component.Heights[a].y && height < component.Heights[b].x)
                                {
                                    float damp = Mathf.InverseLerp(component.Heights[a].y, component.Heights[b].x, height);
                                    damp *= Mathf.Lerp(strength, 1.0f, damp);
                                    //damp += strength * 2.0f * (Mathf.Abs(damp - 0.5f));

                                    alphaMaps[y, x, i] += angle * (1.0f - damp);
                                    alphaMaps[y, x, j] += angle * damp;
                                    break;
                                }
                                else if (height >= component.Heights[a].x && height < component.Heights[a].y)
                                {
                                    alphaMaps[y, x, i] += angle;
                                    break;
                                }
                            }
                        }
                        break;

                    case SplatTool.AngleBased:
                        alpha = Mathf.Clamp(alpha * angle, 0.0f, strength);
                        alphaMaps[y, x, component.Textures.SelectedCliffBrush] = 1.0f;
                        break;
                    case SplatTool.Sharpen:
                        float total = 0.0f;
                        for (int i = 0; i < splatPrototypesCount; i++)
                        {
                            alphaMaps[y, x, i] = prevAlpha[i];
                            if (component.Textures.IsMultiSelect && !component.Textures.IsSelected(i))
                            {
                                total += alphaMaps[y, x, i];
                                continue;
                            }
                            prevAlpha[i] = Mathf.Pow(prevAlpha[i], component.SharpenStrength);
                            alphaMaps[y, x, i] = Mathf.Lerp(alphaMaps[y, x, i], prevAlpha[i], alpha);
                            total += alphaMaps[y, x, i];
                        }
                        for (int i = 0; i < splatPrototypesCount; i++)
                        {
                            alphaMaps[y, x, i] /= total;
                        }
                        continue;
                    case SplatTool.Blur:
                        int halfRadius = component.SmoothStrength;
                        int radius = halfRadius + halfRadius;
                        float totalb = 0.0f;
                        for (int i = 0; i < splatPrototypesCount; i++)
                        {
                            // if not selected i then continue
                            alphaMaps[y, x, i] = prevAlpha[i];
                            if (component.Textures.IsMultiSelect && !component.Textures.IsSelected(i))
                            {
                                totalb += alphaMaps[y, x, i];
                                continue;
                            }

                            float val = 0.0f;
                            float sum = 0.0f;
                            for (int j = -halfRadius; j <= halfRadius; j++) // y
                            {
                                int yj = y + j;
                                if (yj < 0 || yj >= alphaMaps.GetLength(0)) { continue; }

                                for (int k = -halfRadius; k <= halfRadius; k++) // x
                                {
                                    int xk = x + k;
                                    if (xk < 0 || xk >= alphaMaps.GetLength(1)) { continue; }
                                    float dsq = j * j * k * k;
                                    float whg = Mathf.Exp(-dsq / (2.0f * radius * radius)) / (Mathf.PI * 2.0f * radius * radius);
                                    val += alphaMaps[yj, xk, i] * whg;
                                    sum += whg;
                                }
                            }
                            alphaMaps[y, x, i] = Mathf.Lerp(alphaMaps[y, x, i], val / sum, alpha);
                            totalb += alphaMaps[y, x, i];
                        }
                        for (int i = 0; i < splatPrototypesCount; i++)
                        {
                            alphaMaps[y, x, i] /= totalb;
                        }
                        continue;
                }

                // Set alpha map
                for (int i = 0; i < splatPrototypesCount; i++)
                {
                    alphaMaps[y, x, i] *= alpha;
                    alphaMaps[y, x, i] += prevAlpha[i] * (1.0f - alpha);
                }
            }
            component.TerrainData.SetAlphamaps(startPosition.x, startPosition.y, alphaMaps);
        }
        

        internal override void OnSceneGUI(TP c)
        {
            base.OnSceneGUI(c);
        }

        internal override void OnInspectorGUI(TP c)
        {
            base.OnInspectorGUI(c);

            // Toolbar
            GUILayout.BeginHorizontal();
            FlexibleSpace(5);
            component.SplatTool = (SplatTool)GUILayout.Toolbar((int)component.SplatTool, ToolbarItems, ToolbarOptions(ToolbarItems.Length));
            FlexibleSpace(5);
            GUILayout.EndHorizontal();
            
            // 
            switch (c.SplatTool)
            {
                case SplatTool.OneTexture:
                    DrawTooltip(2, "", new string[] {
                        "Paint with one texture"
                    });

                    EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                    DrawBrushSettings();
                    DrawSelectorEditor(m_textureContainer, TextureSelectType.Default);
                    break;
                case SplatTool.HeightBased:
                    DrawTooltip(2, "", new string[] {
                        "Paint textures based on height"
                    });

                    EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                    DrawBrushSettings();
                    DrawHeightEditor();
                    break;
                case SplatTool.AngleBased:
                    DrawTooltip(2, "", new string[] {
                        "Paint only the terrain slopes"
                    });

                    EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                    DrawBrushSettings();
                    break;
                case SplatTool.Sharpen:
                    DrawTooltip(2, "", new string[] {
                        "Sharpen texture transitions"
                    });

                    EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                    DrawBrushSettings();
                    component.SharpenStrength = EditorGUILayout.Slider(new GUIContent("Sharpen Strength", "Strength of sharpening"), component.SharpenStrength, 1.0f, 10.0f);
                    component.Textures.IsMultiSelect = EditorGUILayout.Toggle(new GUIContent("Use selected textures", "Aplly sharpen to selected textures (disabling this will apply to all textures)"), component.Textures.IsMultiSelect);
                    if (component.Textures.IsMultiSelect) { DrawSelectorEditor(m_cliffContainer, TextureSelectType.Multi); }
                    EditorGUILayout.EndVertical();
                    DrawAlphaSelector(component, TP.Colors.SelectedPaintTextureColor);
                    PaintAllEditor();
                    return;
                case SplatTool.Blur:
                    DrawTooltip(2, "", new string[] {
                        "Smooth texture transitions"
                    });

                    EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                    DrawBrushSettings();
                    component.SmoothStrength = EditorGUILayout.IntSlider(new GUIContent("Blur Strength", "Blurring Radius/Strength"), component.SmoothStrength, 1, 8);
                    component.Textures.IsMultiSelect = EditorGUILayout.Toggle(new GUIContent("Use selected textures", "Aplly smoothing to selected textures (disabling this will apply to all textures)"), component.Textures.IsMultiSelect);
                    if (component.Textures.IsMultiSelect) { DrawSelectorEditor(m_cliffContainer, TextureSelectType.Multi); }
                    EditorGUILayout.EndVertical();
                    DrawAlphaSelector(component, TP.Colors.SelectedPaintTextureColor);
                    PaintAllEditor();
                    return;
                default:
                    DrawNone();
                    return;
            }
            EditorGUILayout.EndVertical();

            DrawAlphaSelector(component, TP.Colors.SelectedPaintTextureColor);
            DrawNoiseEditor(component, component.TerrainData.alphamapResolution);
            DrawSlopeEditor(component, DrawCliffSelectorEditor);
            PaintAllEditor();
        }
        #region Heights
        /// <summary>
        /// Draw height option
        /// </summary>
        private void DrawHeightEditor()
        {
            if (component.TerrainData.splatPrototypes.Length < 1) { return; }

            // Draw label
            GUILayout.Label(new GUIContent("Heights"), EditorStyles.boldLabel);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);
            if (component.Heights.Count < 1)
            {
                GUILayout.Label("No Heights", new GUILayoutOption[0]);
                GUILayout.Label("Please add a height texture", EditorStyles.wordWrappedMiniLabel, new GUILayoutOption[0]);
            }
            else
            {
                // Draw all heights
                int i = -1;
                while (++i < component.Heights.Count)
                {
                    EditorGUI.BeginChangeCheck();
                    DrawHeightRow(ref i);
                    if (component.Heights.Count < 1) { break; }
                }
            }

            // Draw add button
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            if (GUILayout.Button(new GUIContent("add"), new GUILayoutOption[0])) { m_isAddingHeight = true; }
            EditorGUILayout.EndHorizontal();

            // Adding a texture
            if (m_isAddingHeight)
            {
                // Draw preview texture
                Rect current = EditorGUILayout.BeginHorizontal();
                current.height = current.width = 35;
                GUI.Label(current, new GUIContent(component.TerrainData.splatPrototypes[m_tempHeightIndex].texture, component.TerrainData.splatPrototypes[m_tempHeightIndex].texture.name));
                GUILayout.Space(current.height + 2.0f);

                // Save index of selected texture
                m_tempHeightIndex = EditorGUILayout.IntSlider(m_tempHeightIndex, 0, component.TerrainData.splatPrototypes.Length - 1);
                EditorGUILayout.EndHorizontal();

                EditorGUILayout.BeginHorizontal();
                FlexibleSpace(10);

                // Add button
                if (GUILayout.Button(new GUIContent("add"), new GUILayoutOption[0]))
                {
                    component.Heights.Add(new tp_Height(m_tempHeightIndex, component.Heights));
                    m_tempHeightIndex = 0;
                    m_isAddingHeight = false;
                }

                // Cancel button
                if (GUILayout.Button(new GUIContent("cancel"), new GUILayoutOption[0]))
                {
                    m_tempHeightIndex = 0;
                    m_isAddingHeight = false;
                }
                EditorGUILayout.EndHorizontal();
            }
            EditorGUILayout.EndVertical();
        }
        /// <summary>
        /// Draw Height item
        /// </summary>
        /// <param name="i">Index</param>
        private void DrawHeightRow(ref int i)
        {
            // Check if texture is valid, in case of a deleted texture is still in the height option
            if (component.Heights[i].Index < 0 || component.Heights[i].Index >= component.TerrainData.splatPrototypes.Length)
            {
                component.Heights.RemoveAt(i--);
                return;
            }

            // Save and set the backgroundcolor
            Color backColor = GUI.backgroundColor;
            GUI.backgroundColor = Color.Lerp(GUI.backgroundColor, ((i % 2) == 0) ? Color.white : Color.black, 0.2f);

            EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);
            GUI.backgroundColor = backColor;

            EditorGUILayout.BeginHorizontal();
            // Go up button
            if (i == 0) { GUI.enabled = false; }
            if (GUILayout.Button(new GUIContent("/\\", "Up"), new GUILayoutOption[0])) { component.Heights.Swap(i, i - 1); }

            // Go down button
            GUI.enabled = true;
            if (i == component.Heights.Count - 1) { GUI.enabled = false; }
            if (GUILayout.Button(new GUIContent("\\/", "Down"), new GUILayoutOption[0])) { component.Heights.Swap(i, i + 1); }
            GUI.enabled = true;

            // Draw preview textyre
            Rect current = EditorGUILayout.BeginHorizontal();
            current.height = current.width = 35;
            GUI.Label(current, new GUIContent(
                component.TerrainData.splatPrototypes[component.Heights[i].Index].texture, 
                component.TerrainData.splatPrototypes[component.Heights[i].Index].texture.name));
            GUILayout.Space(current.height + 2.0f);
            EditorGUILayout.EndHorizontal();

            // Draw height start- and end-height values
            FlexibleSpace(3);
            Vector2 currentRange = component.Heights[i].Range;

            GUILayout.BeginHorizontal();
            Seperator(3);
            currentRange.x = EditorGUILayout.FloatField(currentRange.x, new GUILayoutOption[0]);
            GUILayout.Label(new GUIContent(" / "), new GUILayoutOption[0]);
            currentRange.y = EditorGUILayout.FloatField(currentRange.y, new GUILayoutOption[0]);
            GUILayout.EndHorizontal();
            Seperator(3);
            FlexibleSpace(10);

            // Remove button
            if (GUILayout.Button(new GUIContent("X", "Remove"), new GUILayoutOption[0]))
            {
                component.Heights.RemoveAt(i--);
                EditorGUILayout.EndHorizontal();
                EditorGUILayout.EndVertical();
                return;
            }
            EditorGUILayout.EndHorizontal();

            GUILayout.Space(10.0f);

            EditorGUILayout.BeginHorizontal();
            Vector2 range = Vector2.zero;
            if (i < 1)
            {
                // Upper height slider min max values
                range.x = 0.0f;

                if (i == component.Heights.Count - 1) { range.y = component.TerrainData.heightmapScale.y; }
                else { range.y = component.Heights[i + 1].x; }
            }
            else if (i < component.Heights.Count - 1)
            {
                // Middle height slider min max values
                range.x = component.Heights[i - 1].y;
                range.y = component.Heights[i + 1].x;
            }
            else
            {
                // Lower height slider min max values
                range.x = component.Heights[i - 1].y;
                range.y = component.TerrainData.heightmapScale.y;
            }

            // Draw min max slider
            EditorGUILayout.MinMaxSlider(ref currentRange.x, ref currentRange.y, range.x, range.y, new GUILayoutOption[0]);
            if (i < 1) { currentRange.x = 0.0f; }
            if (i == component.Heights.Count - 1) { currentRange.y = component.TerrainData.heightmapScale.y; }
            component.Heights[i].Range = currentRange;

            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }
        #endregion
        #region Cliff
        /// <summary>
        /// Draw cliff editor
        /// </summary>
        
        
        #endregion
        #region Selector
        /// <summary>
        /// Draw texture selector
        /// </summary>
        private void DrawCliffSelectorEditor()
        {
            DrawSelectorEditor(m_cliffContainer, TextureSelectType.Cliff);
        }
        /// <summary>
        /// Draw texture selector
        /// </summary>
        private void DrawSelectorEditor(TextureEditContainer tec, TextureSelectType type, bool drawEditor = true)
        {
            // Draw label
            string labelMsg;
            switch (type)
            {
                case TextureSelectType.Default: labelMsg = "Texture Selector"; break;
                case TextureSelectType.Cliff: labelMsg = "Cliff Selector"; break;
                case TextureSelectType.Multi: labelMsg = ""; break;
                default: labelMsg = ""; break;
            }
            GUILayout.Label(labelMsg, EditorStyles.boldLabel);

            Rect position = EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);

            // Settings
            Color tempBackColor = GUI.backgroundColor;
            const int imgSize = 50;
            const int spacing = 4;
            float totalSpacing = (float)spacing;
            position.width = Screen.width - (20.0f + spacing * 2.0f);

            // Set position
            Rect current = position;
            current.width = current.height = imgSize;
            current.x += spacing;
            current.y += spacing;

            // Get amount of columns
            int columnCount = (int)((position.width - (2 * spacing)) / (spacing + imgSize));

            for (int i = 0; i < component.TerrainData.splatPrototypes.Length; i++)
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

                switch (type)
                {
                    case TextureSelectType.Default:
                        // Set colour
                        if (i == component.Textures.SelectedTexture) { GUI.backgroundColor = TP.Colors.SelectedPaintTextureColor; }
                        else if (i == component.Textures.SelectedCliffBrush && component.Ramp.Active) { GUI.backgroundColor = Color.Lerp(TP.Colors.UnSelectedColor, TP.Colors.SelectedCliffTextureColor, 0.3f); }
                        if (GUI.Button(current, component.TerrainData.splatPrototypes[i].texture))
                        {
                            // Set selected texture
                            if (component.Textures.SelectedCliffBrush == i) { component.Textures.SelectedCliffBrush = component.Textures.SelectedTexture; }
                            component.Textures.SelectedTexture = i;
                        }
                        break;

                    case TextureSelectType.Cliff:
                        // Set colour
                        if (i == component.Textures.SelectedCliffBrush) { GUI.backgroundColor = TP.Colors.SelectedCliffTextureColor; }
                        else if (i == component.Textures.SelectedTexture) { GUI.backgroundColor = Color.Lerp(TP.Colors.UnSelectedColor, TP.Colors.SelectedPaintTextureColor, 0.3f); }
                        if (GUI.Button(current, component.TerrainData.splatPrototypes[i].texture))
                        {
                            // Set selected texture
                            if (component.Textures.SelectedTexture == i) { component.Textures.SelectedTexture = component.Textures.SelectedCliffBrush; }
                            component.Textures.SelectedCliffBrush = i;
                        }
                        break;

                    case TextureSelectType.Multi:
                        if (component.Textures.IsSelected(i)) { GUI.backgroundColor = TP.Colors.SelectedCliffTextureColor; }
                        if (GUI.Button(current, component.TerrainData.splatPrototypes[i].texture))
                        {
                            if (component.Textures.IsSelected(i)) { component.Textures.SelectedTextures &= ~(1 << i); }
                            else { component.Textures.SelectedTextures |= 1 << i; }
                        }
                        break;
                }
            }

            // Reset settings
            GUI.backgroundColor = tempBackColor;
            totalSpacing += imgSize + spacing;
            current.x = position.x + spacing;
            current.y = position.y + totalSpacing;
            current.width = position.width;
            GUILayout.Space(totalSpacing);

            if (drawEditor)
            {
                switch (type)
                {
                    case TextureSelectType.Default:
                        DrawTextureEditor(tec, component.Textures.SelectedTexture);
                        break;
                    case TextureSelectType.Cliff:
                        DrawTextureEditor(tec, component.Textures.SelectedCliffBrush);
                        break;
                    case TextureSelectType.Multi:
                        //DrawTextureEditor(tec, component.Textures.SelectedTexture);
                        break;
                }
            }
            EditorGUILayout.EndVertical();
        }
        /// <summary>
        /// Draw texture edit
        /// Add, Edit, Remove buttons
        /// </summary>
        /// <param name="tec">Member to save the current values</param>
        /// <param name="selected"></param>
        private void DrawTextureEditor(TextureEditContainer tec, int selected)
        {
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);

            // Draw add button
            if (GUILayout.Button(new GUIContent("add")))
            {
                ResetTextureEditValues(tec);
                tec.m_textureWindow = EditWindow.Add;
            }

            // Check if texture is selected
            if (component.TerrainData.splatPrototypes.Length < 1 || selected < 0 || selected >= component.TerrainData.splatPrototypes.Length) { GUI.enabled = false; }

            // Draw edit button
            if (GUILayout.Button(new GUIContent("edit")))
            {
                // Get splatprototype data
                SplatPrototype sp = component.TerrainData.splatPrototypes[selected];
                tec.m_mainTextureSelector = sp.texture;
                tec.m_normTextureSelector = sp.normalMap;
                tec.m_textureSize = sp.tileSize;
                tec.m_textureOffset = sp.tileOffset;
                tec.m_textureSpecialSettings.x = sp.metallic;
                tec.m_textureSpecialSettings.y = sp.smoothness;
                tec.m_textureSpecularity = sp.specular;
                tec.m_textureWindow = EditWindow.Edit;
            }

            // Draw remove button
            if (GUILayout.Button(new GUIContent("remove")))
            {
                tec.m_textureWindow = EditWindow.Remove;
            }
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();

            // Check which menu needs to be drawn
            switch (tec.m_textureWindow)
            {
                case EditWindow.Add:
                    DrawTextureAdd(tec);
                    break;
                case EditWindow.Edit:
                    DrawTextureEdit(tec, selected);
                    break;
                case EditWindow.Remove:
                    DrawRemoveEditor(tec, selected);
                    break;
            }
        }
        /// <summary>
        /// Add texture
        /// </summary>
        /// <param name="tec">Member to save the current values</param>
        private void DrawTextureAdd(TextureEditContainer tec)
        {
            GUILayout.Label(new GUIContent("Add Texture"), EditorStyles.boldLabel);
            DrawTextureEditFields(tec);

            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);

            // Draw cancel button
            if (GUILayout.Button("cancel")) { ResetTextureEditValues(tec); }

            // Add splatprototype
            if (GUILayout.Button("add"))
            {
                if (tec.m_mainTextureSelector && tec.m_textureSize.x > 0.0 && tec.m_textureSize.y > 0.0f)
                {
                    tp_Utilities.AddSplatTexture(
                        component.TerrainData,
                        tec.m_mainTextureSelector,
                        tec.m_normTextureSelector,
                        tec.m_textureSize,
                        tec.m_textureOffset,
                        tec.m_textureSpecialSettings.x,
                        tec.m_textureSpecialSettings.y,
                        tec.m_textureSpecularity);

                    ResetTextureEditValues(tec);
                }
            }
            EditorGUILayout.EndHorizontal();
        }
        /// <summary>
        /// Edit texture
        /// </summary>
        /// <param name="tec">Member to save the current values</param>
        /// <param name="selected">Selected texture</param>
        private void DrawTextureEdit(TextureEditContainer tec, int selected)
        {
            // Draw label
            GUILayout.Label(new GUIContent("Edit Texture"), EditorStyles.boldLabel);

            DrawTextureEditFields(tec);

            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);

            // Draw calcel button
            if (GUILayout.Button("cancel")) { ResetTextureEditValues(tec); }

            // Set new values to splatprototype
            if (GUILayout.Button("edit"))
            {
                if (tec.m_mainTextureSelector && tec.m_textureSize.x > 0.0 && tec.m_textureSize.y > 0.0f)
                {
                    tp_Utilities.EditSplatTexture(
                        component.TerrainData,
                        selected,
                        tec.m_mainTextureSelector,
                        tec.m_normTextureSelector,
                        tec.m_textureSize,
                        tec.m_textureOffset,
                        tec.m_textureSpecialSettings.x,
                        tec.m_textureSpecialSettings.y,
                        tec.m_textureSpecularity);

                    ResetTextureEditValues(tec);
                }
            }
            EditorGUILayout.EndHorizontal();
        }
        /// <summary>
        /// Edit texture
        /// </summary>
        /// <param name="tec">Member to save the current values</param>
        /// <param name="selected">Selected texture</param>
        private void DrawRemoveEditor(TextureEditContainer tec, int selected)
        {
            // Draw label
            GUILayout.Label(new GUIContent("Remove Texture"), EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();

            // Draw label
            GUILayout.Label("Are you sure?", EditorStyles.boldLabel);

            FlexibleSpace(5);

            // Remvoe splatprototype
            if (GUILayout.Button(new GUIContent("yes")))
            {
                tp_Utilities.RemoveSplatTexture(component.TerrainData, selected);
                tec.m_textureWindow = EditWindow.None;
            }

            // Draw cancel button
            if (GUILayout.Button(new GUIContent("no")))
            {
                tec.m_textureWindow = EditWindow.None;
            }
            EditorGUILayout.EndHorizontal();
        }
        /// <summary>
        /// Draw splatprototype editor
        /// </summary>
        /// <param name="tec">Member to save values</param>
        private void DrawTextureEditFields(TextureEditContainer tec)
        {
            // Create texture fields
            EditorGUILayout.BeginVertical();
            tec.m_mainTextureSelector = EditorGUILayout.ObjectField("Texture", tec.m_mainTextureSelector, typeof(Texture2D), false) as Texture2D;
            tec.m_normTextureSelector = EditorGUILayout.ObjectField("Normal map", tec.m_normTextureSelector, typeof(Texture2D), false) as Texture2D;
            tec.m_textureSize = EditorGUILayout.Vector2Field("Size", tec.m_textureSize);
            tec.m_textureOffset = EditorGUILayout.Vector2Field("Offset", tec.m_textureOffset);
            tec.m_textureSpecialSettings.x = EditorGUILayout.Slider("Metalic", tec.m_textureSpecialSettings.x, 0.0f, 1.0f);
            tec.m_textureSpecialSettings.y = EditorGUILayout.Slider("Smoothness", tec.m_textureSpecialSettings.y, 0.0f, 1.0f);
            tec.m_textureSpecularity = EditorGUILayout.ColorField("Specular", tec.m_textureSpecularity);
            Seperator(2);
            EditorGUILayout.EndVertical();
        }
        /// <summary>
        /// Reset container
        /// </summary>
        /// <param name="tec">Member where splatprototypes values have been stored</param>
        private void ResetTextureEditValues(TextureEditContainer tec)
        {
            tec.m_mainTextureSelector = null;
            tec.m_normTextureSelector = null;
            tec.m_textureSize = new Vector2(15.0f, 15.0f);
            tec.m_textureOffset = Vector2.zero;
            tec.m_textureSpecialSettings.x = 0.0f;
            tec.m_textureSpecialSettings.y = 0.0f;
            tec.m_textureSpecularity = Color.black;
            tec.m_textureWindow = EditWindow.None;
        }
        #endregion
        #region Toher
        private void ProjectorEnabled()
        {

        }
        private void PaintAllEditor()
        {
            if (component.HidePaintAll) { return; }
            Seperator(2);

            GUIStyle style = new GUIStyle(GUI.skin.button);
            style.fontStyle = FontStyle.Bold;
            style.fontSize = 14;

            // Button, paint!
            EditorGUILayout.BeginHorizontal();
            FlexibleSpace(5);
            //if (component.Heights.Count < 1 || TP.TerrainData.splatPrototypes.Length < 2) { GUI.enabled = false; }
            if (GUILayout.Button(new GUIContent("Paint All", "Paints the whole terrain"), style, new GUILayoutOption[0]))
            {
                // Paint
                if (component.Noise.GenerateOnClick) { component.Noise.Generate(new Point(component.TerrainData.alphamapWidth, component.TerrainData.alphamapHeight)); }
                component.History.CreateRestorePoint(UndoType.Splatmap, TP.UndoMessages.TEXTURE);
                Paint(true);
            }
            GUI.enabled = true;
            FlexibleSpace(5);
            EditorGUILayout.EndHorizontal();
        }
        #endregion
    }
}
