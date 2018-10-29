////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_HeightmapEditor.cs
//      Author:             HOEKKII
//      
//      Description:        NOT AVAILABLE YET
//      
////////////////////////////////////////////////////////////////////////////


using UnityEngine;
using UnityEditor;
using TerrainPainter.Enums;
using TP = TerrainPainter.TerrainPainter;
using System;
using System.IO;

namespace TerrainPainter
{
    public class tp_HeightmapEditor : tp_ComponentBaseEditor
    {
        private float m_noiseResolutionMultiplier = 2.0f;
        private GUIContent[] m_toolbarItems;

        private GUIContent[] ToolbarItems
        {
            get
            {
                if (m_toolbarItems == null || m_toolbarItems.Length < 1)
                {
                    m_toolbarItems = new GUIContent[] {
                        IconContent("Sculpt", "Sculpt"),
                        IconContent("Flatten", "Flatten"),
                        IconContent("Pull", "Pull"),
                        IconContent("Erosion", "Erosion"),
                        IconContent("Smooth", "Smooth"),
                        IconContent("Clone", "Clone"),
                        IconContent("Path", "Path")
                    };
                }
                return m_toolbarItems;
            }
        }
        internal override UndoType UndoType
        {
            get
            {
                return UndoType.HeightMap;
            }
        }

        internal override void Paint(bool paintAll)
        {
            // 
            if (component.Terrain == null) { return; }

            // 
            //int brushSize, halfBrushSize;
            //Point  startPosition; //position,

            // testing
            //brushSize = component.TerrainData.heightmapResolution;
            //halfBrushSize = brushSize / 2;
            //startPosition = Point.zero;



            // 
            int resolution = component.TerrainData.heightmapResolution;
            float[,] heightMaps = component.TerrainData.GetHeights(0, 0, resolution, resolution);

            float alpha; //height, angle, 

            int x, y;
            for (y = 0; y < heightMaps.GetLength(0); y++)
            {
                for (x = 0; x < heightMaps.GetLength(1); x++)
                {
                    Point current = new Point(x, y);
                    alpha = GetNoiseValue(current, resolution);
                    heightMaps[y, x] = alpha;
                }
            }

            component.TerrainData.SetHeights(0, 0, heightMaps);
        }

        internal override void OnSceneGUI(TerrainPainter c)
        {
            base.OnSceneGUI(c);

        }
        internal override void OnInspectorGUI(TerrainPainter c)
        {
            base.OnInspectorGUI(c);
            GUILayout.BeginHorizontal();
            FlexibleSpace(5);
            component.HeightTool = (HeightTool)GUILayout.Toolbar((int)component.HeightTool, ToolbarItems, ToolbarOptions(ToolbarItems.Length));
            FlexibleSpace(5);
            GUILayout.EndHorizontal();

            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            switch (c.HeightTool)
            {
                case HeightTool.Sculpt:



                    break;
                case HeightTool.Flatten:
                    break;
                case HeightTool.Pull:
                    break;
                case HeightTool.Erosion:
                    break;
                case HeightTool.Smooth:
                    break;
                case HeightTool.Clone:
                    break;
                case HeightTool.Path:
                    break;
                default:
                    DrawNone();
                    EditorGUILayout.EndVertical();
                    return;
            }

            m_noiseResolutionMultiplier = EditorGUILayout.FloatField("noise resolution", m_noiseResolutionMultiplier);


            EditorGUILayout.EndVertical();
            DrawNoiseEditor(component, Mathf.RoundToInt(component.TerrainData.heightmapResolution * m_noiseResolutionMultiplier));
            PaintAllEditor();
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
                int resolution = Mathf.RoundToInt(component.TerrainData.heightmapResolution * m_noiseResolutionMultiplier);
                if (component.Noise.GenerateOnClick) { component.Noise.Generate(new Point(resolution, resolution)); }
                component.History.CreateRestorePoint(UndoType.HeightMap, TP.UndoMessages.HEIGHTMAP);
                Paint(true);
            }
            GUI.enabled = true;
            FlexibleSpace(5);
            EditorGUILayout.EndHorizontal();
        }
    }
}