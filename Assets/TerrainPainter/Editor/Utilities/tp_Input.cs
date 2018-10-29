////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Input.cs
//      Author:             HOEKKII
//      
//      Description:        Editor input
//      
////////////////////////////////////////////////////////////////////////////

using UnityEditor;
using UnityEngine;
using TP = TerrainPainter.TerrainPainter;

namespace TerrainPainter
{
    class tp_Input
    {
        public static bool RotateButtonPressed { get { return Event.current.control; } }
        public static bool ScaleButtonPressed { get { return Event.current.shift; } }

        /// <summary>
        /// Get input
        /// </summary>
        /// <param name="method">Delegate method</param>
        /// <param name="position">Mouse world position</param>
        /// <param name="active">Is painting</param>
        /// <param name="saveMsg">Undo message</param>
        internal static void Update(TP component, tp_ComponentBaseEditor target, ref Vector3 position, ref bool active, string saveMsg = default(string))
        {
            // Get current event
            Event current = Event.current;

            // checkkeys
            int controlID = GUIUtility.GetControlID(TP.HashCode, FocusType.Passive);
            int hotControl = GUIUtility.hotControl;
            EventType filteredControlID = current.GetTypeForControl(controlID);

            switch (filteredControlID)
            {
                // (If mouse down or mouse drag)
                case EventType.MouseDown:
                case EventType.MouseDrag:
                    // Check if current control is Terrain Painter control
                    if (hotControl != controlID && (hotControl != 0) || TP.Instance.SplatmapLock || RotateButtonPressed || ScaleButtonPressed) { return; }
                    if (current.button != 0 || HandleUtility.nearestControl != controlID || (TP.Instance.EditingTool == EditingTool.Texture && TP.Instance.SplatTool == SplatTool.None)) { return; }

                    if (current.type == EventType.MouseDown) { GUIUtility.hotControl = controlID; }
                    if (!GetPosition(out position)) { break; }
                    if (current.type == EventType.MouseDown)
                    {
                        // Save current splatmap
                        TP.Instance.History.CreateRestorePoint(target.UndoType, saveMsg);
                        if (TP.Instance.Noise.GenerateOnClick) { TP.Instance.Noise.Generate(new Point(component.TerrainData.alphamapWidth, component.TerrainData.alphamapHeight)); }
                    }
                    
                    // Run method, and Painting is in use
                    if (target != null) { target.Paint(false); }
                    active = true;
                    current.Use();
                    break;

                // Update projector position
                case EventType.MouseMove:
                    if (GetPosition(out position)) { HandleUtility.Repaint(); }
                    break;

                // Disable painting, and current control is not Terrain Painter any more
                case EventType.MouseUp:
                    active = false;
                    if (hotControl != controlID || TP.Instance.SplatmapLock) { return; }
                    GUIUtility.hotControl = hotControl = 0;
                    if (TP.Instance.EditingTool == EditingTool.Texture && TP.Instance.SplatTool == SplatTool.None) { return; }
                    current.Use();
                    break;

                // Disable the selection tool
                case EventType.Layout:
                    if (RotateButtonPressed || ScaleButtonPressed)
                    {
                        HandleUtility.AddDefaultControl(controlID);
                        HandleUtility.Repaint();
                        break;
                    }

                    if ((TP.Instance.EditingTool == EditingTool.Texture && TP.Instance.SplatTool == SplatTool.None) || TP.Instance.SplatmapLock) { return; }
                    HandleUtility.AddDefaultControl(controlID);
                    break;
            }
        }

        /// <summary>
        /// Raycast mouse
        /// </summary>
        /// <param name="position"></param>
        /// <returns></returns>
        private static bool GetPosition(out Vector3 position)
        {
            // Check if mouse hits the terrain
            RaycastHit hit = default(RaycastHit);
            TP.Instance.Terrain.GetComponent<Collider>().Raycast(
            HandleUtility.GUIPointToWorldRay(Event.current.mousePosition + TP.Settings.BRUSH_OFFSET),
                out hit,
                Mathf.Infinity);

            // Set mouse world position
            position = Vector3.zero;
            if (hit.collider != null) { position = hit.point; }
            return hit.collider != null;
        }
    }
}
