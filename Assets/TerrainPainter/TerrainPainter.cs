using UnityEngine;

namespace TerrainPainter
{
	[AddComponentMenu("Terrain/Terrain Painter")]
    public class TerrainPainter : MonoBehaviour
    {
		#if UNITY_EDITOR
		public sealed class ErrorMessages
        {
            public const string NO_TERRAIN_COMPONENT = "Unity Terrain Component not found. Make sure Terrain Component is attached to the GameObject.";
            public const string TEXTURE_NOT_ACCESSABLE = "Texture is not accessable. Make sure you checked \"Read/Write enabled\" in the import settings.";
            public const string ALPHA_NOT_FOUND = "Could not get any Brushes. Make sure you put brush textures in the \"" + Settings.BRUSHES_FOLDER_NAME + "\" folder";
        }
        public sealed class UndoMessages
        {
            public const string TEXTURE = "";
            public const string FOLIAGE = "";
            public const string HEIGHTMAP = "";
        }
        public sealed class Colors
        {
            public static Color SelectedPaintTextureColor { get { return new Color(0.2f, 0.9f, 0.3f, 1.0f); } }
            public static Color SelectedCliffTextureColor { get { return new Color(0.9f, 0.3f, 0.2f, 1.0f); } }

            public static Color SelectedAlphaColor { get { return new Color(0.2f, 0.9f, 0.3f, 1.0f); } }

            public static Color UnSelectedColor { get { return new Color(1f, 1f, 1f, 1.0f); } }

            public static Color SelectedFoliageColor { get { return SelectedPaintTextureColor; } }
        }
        public sealed class Settings
        {
            public static readonly Vector2 BRUSH_OFFSET = new Vector2(-15.0f, -15.0f);
            public const string BRUSHES_FOLDER_NAME = "TP_Brushes";
        }

        // 
        private static TerrainPainter m_instance;
        public static int HashCode = "Terrain Painter".GetHashCode();
        public const int MAX_TOOLTIP_LEVEL = 2;

        // Components
        private GameObject m_gameObject;
        private Transform m_transform;
        private Terrain m_terrain;
        private TerrainData m_terrainData;

        [SerializeField] private tp_Alpha m_alpha;
        [SerializeField] private tp_Heights m_heights;
        [SerializeField] private tp_History m_history;
        [SerializeField] private tp_Noise m_noise;
        [SerializeField] private tp_MinMax m_ramp;
        [SerializeField] private tp_Textures m_textures;
        [SerializeField] private tp_Foliages m_foliages;

        // 
        [SerializeField] private bool m_splatmapLocked = false;
        [SerializeField] private bool m_hidePaintAll = false;
        [SerializeField] private EditingTool m_selectedEditingTool = EditingTool.None;
        [SerializeField] private SplatTool m_selectedSplatTool = SplatTool.None;
        [SerializeField] private HeightTool m_selectedHeightTool = HeightTool.None;
        [SerializeField] private float m_opacity = 100.0f;
        [SerializeField] private float m_strength = 100.0f;
        [SerializeField] private float m_brushSize = 100.0f;
        [SerializeField] private float m_brushRotation = 0.0f;
        [SerializeField] private float m_brushCosAngle = 0.0f;
        [SerializeField] private float m_brushSinAngle = 0.0f;
        [SerializeField] private float m_brushRotationSizeMultiplier = 1.0f;
        [SerializeField] private float m_sharpenStrength = 3.0f;
        [SerializeField] private int m_smoothStrength = 2;

        // 
        [SerializeField] private int m_maxFoliageStrength = 10;
        [SerializeField] private int m_targetFoliageStrength = 1;
        
        // Input
        public bool MouseDown = false;
        public Vector3 brushPosition = Vector2.zero;
        private bool m_isPainting = false;

        // User Settings
        [SerializeField] private float m_rotationSensitivity = -6.0f;
        const float _scaleSensitivity = 0.01f;
        [SerializeField] private int m_tooltipLevel = 0;

        // Properties
        public bool SplatmapLock
        {
            get { return m_splatmapLocked; }
            set { m_splatmapLocked = value; }
        }
        public bool HidePaintAll
        {
            get { return m_hidePaintAll; }
            set { m_hidePaintAll = value; }
        }
        public float BrushSize
        {
            get { return m_brushSize; }
            set { m_brushSize = Mathf.Max(0.05f, value); }
        }
        public float BrushSizeMultiplier
        {
            get { return m_brushRotationSizeMultiplier; }
        }
        public float BrushSinAngle
        {
            get { return m_brushSinAngle; }
        }
        public float BrushCosAngle
        {
            get { return m_brushCosAngle; }
        }
        public float BrushRotation
        {
            get { return m_brushRotation; }
            set
            {
                m_brushRotation = FixAngle(value);
                m_brushRotationSizeMultiplier = Mathf.Abs(m_brushCosAngle = Mathf.Cos(m_brushRotation * Mathf.Deg2Rad));
                m_brushRotationSizeMultiplier += Mathf.Abs(m_brushSinAngle = Mathf.Sin(m_brushRotation * Mathf.Deg2Rad));
            }
        }
        public bool IsPainting
        {
            get { return m_isPainting; }
            set { m_isPainting = value; }
        }
        public float Opacity
        {
            get { return m_opacity; }
            set { m_opacity = Mathf.Clamp(value, 0.0f, 100.0f); } }
        public float Strength
        {
            get { return m_strength; }
            set { m_strength = Mathf.Clamp(value, 0.0f, 100.0f); }
        }
        public float SharpenStrength
        {
            get { return m_sharpenStrength; }
            set { m_sharpenStrength = Mathf.Max(1.0f, value); }
        }
        public int SmoothStrength
        {
            get { return m_smoothStrength; }
            set { m_smoothStrength = Mathf.Max(1, value); }
        }

        public int MaxFoliageStrength
        {
            get { return m_maxFoliageStrength; }
            set { m_maxFoliageStrength = value; }
        }
        public int CurrentFoliageStrength
        {
            get { return m_targetFoliageStrength; }
            set { m_targetFoliageStrength = Mathf.Max(0, value); }
        }
        public float BrushRotationSensitivity
        {
            get { return m_rotationSensitivity; }
            set { m_rotationSensitivity = value; }
        }

        public float BrushScaleSensitivity
        {
            get { return _scaleSensitivity; }
            //set { _scaleSensitivity = value; }
        }
        public int TooltipLevel
        {
            get { return m_tooltipLevel; }
            set { m_tooltipLevel = Mathf.Clamp(value, 0, MAX_TOOLTIP_LEVEL); }
        }

        public SplatTool SplatTool
        {
            get { return m_selectedSplatTool;  }
            set { m_selectedSplatTool = value; }
        }
        public EditingTool EditingTool
        {
            get { return m_selectedEditingTool;  }
            set { m_selectedEditingTool = value; }
        }
        public HeightTool HeightTool
        {
            get { return m_selectedHeightTool; }
            set { m_selectedHeightTool = value; }
        }

        public tp_Alpha     Alpha       { get { return m_alpha;       } }
        public tp_Heights   Heights     { get { return m_heights;     } }
        public tp_History   History     { get { return m_history;     } }
        public tp_Noise     Noise       { get { return m_noise;       } }
        public tp_MinMax    Ramp        { get { return m_ramp;        } }
        public tp_Textures  Textures    { get { return m_textures;    } }
        public tp_Foliages  Foliages    { get { return m_foliages;    } }

        public Terrain      Terrain     { get { return m_terrain;     } }
        public TerrainData  TerrainData { get { return m_terrainData; } }
        public Transform    Transform   { get { return m_transform;   } }


        // Static Properties
        public static TerrainPainter Instance { get { return m_instance; } }
        
        /// <summary>
        /// Initialize component
        /// </summary>
        public void Initialize()
        {
            m_instance = this;

            if (!m_gameObject) { m_gameObject = GetComponent<GameObject>(); }
            if (!m_transform) { m_transform = GetComponent<Transform>(); }

            m_terrain = GetComponent<Terrain>();
            if (Terrain == null) { return; } // { throw new NullReferenceException(Settings.ErrorMessages.NO_TERRAIN_COMPONENT); }

            m_terrainData = m_terrain.terrainData;
            if (TerrainData == null) { return; } // { throw new NullReferenceException(Settings.ErrorMessages.NO_TERRAIN_DATA_COMPONENT); }

            if (m_alpha == null) { m_alpha = new tp_Alpha(); }
            if (m_heights == null) { m_heights = new tp_Heights(); }
            if (m_history == null) { m_history = new tp_History(); }
            if (m_ramp == null) { m_ramp = new tp_MinMax(45.0f, 65.0f); }
            if (m_textures == null) { m_textures = new tp_Textures(); }
            if (m_foliages == null) { m_foliages = new tp_Foliages(); }

            SafeTextureSelect();
            
            if (m_noise == null)
            {
                m_noise = new tp_Noise();
                m_noise.Generate(new Point(TerrainData.alphamapWidth, TerrainData.alphamapHeight));
            }
        }
        /// <summary>
        /// Check if selected texture and selected cliff texture are not the same
        /// </summary>
        public void SafeTextureSelect()
        {
            if (Textures.SelectedCliffBrush != Textures.SelectedTexture) { return; }
            if (Textures.SelectedCliffBrush < 1) { Textures.SelectedCliffBrush += 1; }
            else { Textures.SelectedCliffBrush -= 1; }
        }

        private static float FixAngle(float angle)
        {
            while (angle < 0.0f) { angle += 360.0f; }
            return angle % 360.0f;
        }
		#endif
	}
}
