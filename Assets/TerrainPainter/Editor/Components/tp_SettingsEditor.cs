////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_SettingsEditor.cs
//      Author:             HOEKKII
//      
//      Description:        All settings
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;
using UnityEditor;
using TP = TerrainPainter.TerrainPainter;
using System;
using System.IO;

namespace TerrainPainter
{
    // Unfinished, this is for future updates
    // The settings are for now in "tp_SplatmapEditor"
    public class tp_SettingsEditor : tp_Editor
    {
        internal override void OnSceneGUI(TP component)
        {
            base.OnSceneGUI(component);
        }

        internal override void OnInspectorGUI(TP component)
        {
            base.OnInspectorGUI(component);
            // Set brush projector color
            GUILayout.Label(new GUIContent("Brush"), EditorStyles.boldLabel);
            tp_Projector.BrushInactiveColor = EditorGUILayout.ColorField(new GUIContent("Inactive Color"), tp_Projector.BrushInactiveColor);
            tp_Projector.BrushActiveColor = EditorGUILayout.ColorField(new GUIContent("Active Color"), tp_Projector.BrushActiveColor);
            component.BrushRotationSensitivity = EditorGUILayout.FloatField(new GUIContent("Rotating sensitivity"), component.BrushRotationSensitivity);
            EditorGUILayout.BeginHorizontal();
            Seperator(1);
            if (GUILayout.Button("Load Deafault")) { LoadDefault(); }
            EditorGUILayout.EndHorizontal();
            Seperator(3);

            // Undo
            GUILayout.Label(new GUIContent("History"), EditorStyles.boldLabel);
            component.History.Capacity = EditorGUILayout.IntField(new GUIContent("Undo Capacity", "How may time undo can be performed"), component.History.Capacity);
            Seperator(3);

            // Save load Settings
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Settings"), EditorStyles.boldLabel);
            FlexibleSpace(5);
            if (GUILayout.Button("Save..")) { SaveSettings(); }
            if (GUILayout.Button("Load..")) { LoadSettings(); }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);
            GUILayout.Label(new GUIContent("You can save and load your settings \ni.e. Height settings, projector colors"), EditorStyles.label);
            EditorGUILayout.EndVertical();

            Seperator(3);

            // Save load terrain splat
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Splatmaps"), EditorStyles.boldLabel);
            FlexibleSpace(5);
            if (GUILayout.Button("Save..")) { SaveSplatMap(); }
            if (GUILayout.Button("Load..")) { LoadSplatMap(); }
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox, new GUILayoutOption[0]);
            GUILayout.Label(new GUIContent("NOTE: The process could take \"a few\" seconds; \ndepending on your system, \nsplatmap size and amount of splatmaps.\n\nNOTE: Make sure the loaded splatmap-resolution \nmatches the terrain splatmap-resolution"), EditorStyles.miniLabel);
            EditorGUILayout.EndVertical();


            Seperator(3);

            GUILayout.Label(new GUIContent("Foliage"), EditorStyles.boldLabel);
            component.MaxFoliageStrength = EditorGUILayout.IntField(new GUIContent("Max Foliage Strength"), component.MaxFoliageStrength, new GUILayoutOption[0]);
            component.MaxFoliageStrength = Mathf.Max(1, component.MaxFoliageStrength);

            Seperator(3);

            // Hide/Show Paint all
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Paint all Button: "), EditorStyles.boldLabel);
            component.HidePaintAll = DrawSwitch(component.HidePaintAll, "Show", "Hide");
            EditorGUILayout.EndHorizontal();

            Seperator(3);

            // Lock/Unlock splatmap
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Lock Splatmap: "), EditorStyles.boldLabel);
            component.SplatmapLock = DrawSwitch(component.SplatmapLock, "Unlock", "Lock");
            EditorGUILayout.EndHorizontal();


            Seperator(3);

            // Tooltip level
            EditorGUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Tooltip Level: "), EditorStyles.boldLabel);
            component.TooltipLevel = EditorGUILayout.IntSlider(component.TooltipLevel, 0, TP.MAX_TOOLTIP_LEVEL);
            EditorGUILayout.EndHorizontal();
            DrawTooltip(0, "Tooltip level", new string[] {
                "No tooltips will be shown at all",
                "Some tooltips will be shown",
                "All tooltips will be shown"
            });
        }
        private void SaveSplatMap()
        {
            // Get file path
            string path = EditorUtility.SaveFilePanel("Save Terrain Splatmaps", "", "splatmap" + ".xml", "xml");
            if (path.Length > 0)
            {
                // Get current time for determining how long it takes to save the splatmaps
                DateTime dt = DateTime.Now;

                // Get current splatmap
                tp_Containers.SplatmapContainer tc = new tp_Containers.SplatmapContainer();
                tc.SetSplatmap(component.TerrainData.GetAlphamaps(0, 0, component.TerrainData.alphamapWidth, component.TerrainData.alphamapHeight));

                // Serialize and save splatmap
                string file = LoadSave.Serialize(tc);
                StreamWriter writer = new StreamWriter(path);
                writer.Write(file);
                writer.Close();

                // Print how long it took to save the splatmaps
                Debug.Log("Saving splatmap finished! time: [" + (DateTime.Now - dt).ToString() + "]");
            }
        }
        private void LoadSplatMap()
        {
            // Get file path
            string path = EditorUtility.OpenFilePanel("Load Terrain Splatmaps", "", "xml");
            if (path.Length > 0)
            {
                // Get current time for determining how long it takes to load the splatmaps
                DateTime dt = DateTime.Now;

                // Desirialize data
                StreamReader reader = new StreamReader(path);
                string file = reader.ReadToEnd();
                reader.Close();

                // Convert deserialized data to splatmaps
                tp_Containers.SplatmapContainer tc = LoadSave.Deserialize<tp_Containers.SplatmapContainer>(file);
                float[,,] splat = tc.GetSplatmap();

                // Set splatmap
                component.TerrainData.SetAlphamaps(0, 0, splat);

                // Print how long it took to load the splatmaps
                Debug.Log("Loading splatmap finished! time: [" + (DateTime.Now - dt).ToString() + "]");
            }
        }
        private void LoadDefault()
        {
            tp_Projector.BrushActiveColor = new Color(1.0f, 0.34f, 0.0f, 0.7f);
            tp_Projector.BrushInactiveColor = new Color(0.0f, 0.34f, 1.0f, 0.7f);
            component.BrushRotationSensitivity = -6.0f;
        }
        private void SaveSettings()
        {
            // Get file path
            string path = EditorUtility.SaveFilePanel("Save Terrain Settings", "", "settings" + ".xml", "xml");
            if (path.Length > 0)
            {
                // Get current splatmap
                tp_Containers.SettingsContainer c = new tp_Containers.SettingsContainer();
                c.opacity = component.Opacity;
                c.strength = component.Strength;
                c.size = component.BrushSize;

                c.heights = new tp_Height[component.Heights.Count];
                for (int i = 0; i < c.heights.Length; i++) { c.heights[i] = component.Heights[i]; }
                c.ramp = component.Ramp.Value;

                // Serialize and save splatmap
                string file = LoadSave.Serialize(c);
                StreamWriter writer = new StreamWriter(path);
                writer.Write(file);
                writer.Close();

                // Print
                Debug.Log("Saving settings finished!");
            }
        }
        private void LoadSettings()
        {
            // Get file path
            string path = EditorUtility.OpenFilePanel("Load Terrain Settings", "", "xml");
            if (path.Length > 0)
            {
                // Desirialize data
                StreamReader reader = new StreamReader(path);
                string file = reader.ReadToEnd();
                reader.Close();

                // Convert deserialized data to splatmaps
                tp_Containers.SettingsContainer c = LoadSave.Deserialize<tp_Containers.SettingsContainer>(file);
                component.Opacity = c.opacity;
                component.Strength = c.strength;
                component.BrushSize = c.size;

                component.Heights.Clear();
                for (int i = 0; i < c.heights.Length; i++) { component.Heights.Add(c.heights[i]); }
                component.Ramp.Set(c.ramp);


                // Print how long it took to load the splatmaps
                Debug.Log("Loading settings finished!");
            }
        }
    }
}
