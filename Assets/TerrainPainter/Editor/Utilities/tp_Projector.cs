////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Projector.cs
//      Author:             HOEKKII
//      
//      Description:        Projector
//      
////////////////////////////////////////////////////////////////////////////

using System;
using UnityEditor;
using UnityEngine;
using UnityEditorInternal;
using TP = TerrainPainter.TerrainPainter;

namespace TerrainPainter
{
    [Serializable]
    public class tp_Projector
    {
        private const string PROJECTOR_NAME = "TerrainPainterProjector";
        private const float PROJECTOR_Y_OFFSET = 20.0f;

        // Components
        private GameObject m_brushProjectorObject;                                      // Projector
        private Transform m_brushProjectorTransform;                                    // Projector
        private Projector m_brushProjector;                                             // 
        private Material m_brushProjectorMaterial;                                      // Projector material
        private Vector2? m_previousMousePosition;
        private Vector2 m_startRotationAngle;
        
        private static Color m_brushColor = new Color(0.0f, 0.34f, 1.0f, 0.7f);         // Brush/Projector color
        private static Color m_brushActiveColor = new Color(1.0f, 0.34f, 0.0f, 0.7f);   // Brush/Projector active color

        // 
        private float m_colorLerpTime = 0.0f;                                           // 
        private const float m_colorLerpDuration = 0.1f;                                 // 

        public Projector Projector { get { return m_brushProjector; } }
        private Color BrushColor { get { return Color.Lerp(m_brushColor, m_brushActiveColor, ColorLerpValue); } }
        private float ColorLerpValue { get { return m_colorLerpTime / m_colorLerpDuration; } }
        public static Color BrushInactiveColor
        {
            get { return m_brushColor;              }
            set { m_brushColor = value;             }
        }
        public static Color BrushActiveColor
        {
            get { return m_brushActiveColor;        }
            set { m_brushActiveColor = value;       }
        }

        private LayerMask GetTerrainLayer
        {
            get
            {
                // Default is everything
                LayerMask layerMask = -1;

                // Checks if Terrain layer exsists
                int terLayer = LayerMask.NameToLayer("Terrain");
                if (terLayer != -1)
                {
                    if (TP.Instance.Terrain.gameObject.layer == terLayer)
                    {
                        // Set layermask to Terrain only
                        layerMask = 1 << terLayer;
                    }
                }
                return layerMask;
            }
        }
        private LayerMask GetInvTerrainLayer
        {
            get
            {
                // Default is none
                LayerMask layerMask = 0;

                // Checks if Terrain layer exsists
                int terLayer = LayerMask.NameToLayer("Terrain");
                if (terLayer != -1)
                {
                    if (TP.Instance.Terrain.gameObject.layer == terLayer)
                    {
                        // Set layermask to Terrain only
                        layerMask = 1 >> terLayer;
                    }
                }
                return layerMask;
            }
        }

        /// <summary>
        /// Create Projector
        /// </summary>
        public void Create()
        {
            if (m_brushProjectorObject == null)
            {
                m_brushProjectorObject = GameObject.Find(PROJECTOR_NAME);

                // Create Projector
                if (m_brushProjectorObject == null)
                {
                    Type[] components = new Type[] { typeof(Projector) };
                    m_brushProjectorObject = EditorUtility.CreateGameObjectWithHideFlags(PROJECTOR_NAME, HideFlags.HideAndDontSave, components);
                }

                // Get conponents
                m_brushProjectorTransform = m_brushProjectorObject.transform;
                m_brushProjector = m_brushProjectorObject.GetComponent<Projector>();

                // Set Projector component settings
                m_brushProjectorTransform.eulerAngles = new Vector3(90.0f, 0.0f, 0.0f);
                m_brushProjector.nearClipPlane = -1000.0f;
                m_brushProjector.farClipPlane = 1000.0f;
                m_brushProjector.orthographic = true;
            }
            if (m_brushProjectorMaterial == null)
            {
                m_brushProjectorMaterial = new Material(Shader.Find("Terrain Painter/Projector"));
                m_brushProjectorMaterial.hideFlags = HideFlags.HideAndDontSave;

                m_brushProjector.material = m_brushProjectorMaterial;
                m_brushProjector.ignoreLayers = GetInvTerrainLayer;
            }
        }
        /// <summary>
        /// Disable Projector
        /// </summary>
        public void DisableProjector()
        {
            if (m_brushProjector == null) { return; }
            m_brushProjector.enabled = false;
        }
        /// <summary>
        /// Enable Projector
        /// </summary>
        public void EnableProjector()
        {
            if (m_brushProjector == null) { return; }
            m_brushProjector.enabled = true;
        }
        /// <summary>
        /// Destroy Projector
        /// </summary>
        public void DisposeProjector()
        {
            m_brushProjector.enabled = false;
            m_colorLerpTime = 0.0f;
            m_brushProjectorTransform = null;
            m_brushProjector = null;
            m_brushProjectorMaterial = null;
            if (m_brushProjectorObject != null) { GameObject.DestroyImmediate(m_brushProjectorObject); }
        }

        /// <summary>
        /// Update Projector
        /// </summary>
        /// <param name="isPainting"></param>
        public void Update(bool isPainting)
        {
            TP tpc = TP.Instance;
            if (tpc == null || tpc.SplatmapLock || !IsPaintTab(tpc))
            {
                DisableProjector();
                return;
            }

            /*

            if key -> prev mouse pos - cur mouse pos
            val * sensitivity
            brush rot + val
            
            */
            
            Vector2 mousePosition = Event.current.mousePosition;
            if (tp_Input.RotateButtonPressed || tp_Input.ScaleButtonPressed)
            {
                EditorGUIUtility.SetWantsMouseJumping((int)EventModifiers.Alt);
                if (m_previousMousePosition.HasValue)
                {
                    float change = m_previousMousePosition.Value.x - mousePosition.x;

                    if (tp_Input.ScaleButtonPressed)
                    {
                        tpc.BrushSize -= change * tpc.BrushScaleSensitivity * Mathf.Max(1.0f, tpc.BrushSize);
                    }
                    else // tp_Input.RotateButtonPressed
                    {
                        tpc.BrushRotation += change * tpc.BrushRotationSensitivity;
                    }
                }
                else
                {
                    m_startRotationAngle = mousePosition;
                }

                m_previousMousePosition = mousePosition;
                mousePosition = m_startRotationAngle;
            }
            else
            {
                if (m_previousMousePosition.HasValue) { Event.current.mousePosition = m_startRotationAngle; }
                m_previousMousePosition = null;
            }




            Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition + TP.Settings.BRUSH_OFFSET);
            RaycastHit hit;
            if (tpc.Terrain.GetComponent<Collider>().Raycast(ray, out hit, Mathf.Infinity))
            {
                Create();
                EnableProjector();
                UpdateColorLerp(isPainting);
                UpdateProjector(hit.point, tpc.BrushSize);
                m_brushProjectorTransform.position += Vector3.up * tpc.Transform.position.y;
                m_brushProjectorTransform.eulerAngles = new Vector3(90.0f, tpc.BrushRotation, 0.0f);
                return;
            }
            DisableProjector();
        }

        /// <summary>
        /// Load default colours
        /// </summary>
        public void LoadDefault()
        {
            m_brushColor = new Color(0.0f, 0.34f, 1.0f, 0.7f);
            m_brushActiveColor = new Color(1.0f, 0.34f, 0.0f, 0.7f);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="add">is Painting</param>
        private void UpdateColorLerp(bool add)
        {
            m_colorLerpTime = (add ? m_colorLerpDuration : 0.0f);
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="deltatime">Time between previous and current call</param>
        /// <param name="add">is Painting</param>
        private void UpdateColorLerp(float deltatime, bool add)
        {
            m_colorLerpTime += (add ? deltatime : -deltatime);
            m_colorLerpTime = Mathf.Clamp(m_colorLerpTime, 0.0f, m_colorLerpDuration);
        }

        /// <summary>
        /// Update Projector
        /// </summary>
        /// <param name="pos">Projector position</param>
        /// <param name="brushSize">Projector size</param>
        private void UpdateProjector(Vector3 pos, float brushSize)
        {
            if (TP.Instance.Alpha == null || TP.Instance.Alpha.alphaBrushes == null || TP.Instance.Alpha.alphaBrushes.Count < 1) { return; }
            m_brushProjectorMaterial.SetTexture("_MainTex", (Texture2D)TP.Instance.Alpha);
            m_brushProjectorTransform.position = new Vector3(pos.x, TP.Instance.TerrainData.heightmapScale.y + TP.Instance.Transform.position.y + PROJECTOR_Y_OFFSET, pos.z);
            m_brushProjector.farClipPlane = TP.Instance.TerrainData.heightmapScale.y + TP.Instance.Transform.position.y + PROJECTOR_Y_OFFSET * 2.0f;
            m_brushProjector.material.color = BrushColor;
            m_brushProjector.orthographicSize = brushSize / 2.0f;
        }

        private static bool IsPaintTab(TP tpc)
        {
            if (!tpc.Alpha.Enabled) { return false; }
            switch (tpc.EditingTool)
            {
                case EditingTool.Height:
                    break;
                case EditingTool.Texture:
                    switch (tpc.SplatTool)
                    {
                        case SplatTool.OneTexture:
                        case SplatTool.HeightBased:
                        case SplatTool.AngleBased:
                        case SplatTool.Sharpen:
                        case SplatTool.Blur:
                            return true;
                    }
                    break;
                case EditingTool.Foliage: return true;
                case EditingTool.Object:
                    break;
            }
            return false;
        }
    }
}
