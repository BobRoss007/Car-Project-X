////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Noise.cs
//      Author:             HOEKKII
//      
//      Description:        Used to create noise texture
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using UnityEngine;
using Random = UnityEngine.Random;

namespace TerrainPainter
{
    [Serializable] public class tp_Noise
    {
        private Texture2D m_texture = null;     // Texture
        private bool m_generateOnClick = false; // Generate on paint
        private bool m_enabled = false;         // Is active

        // Settings
        private float m_scale = 200.0f;
        private int m_octaves = 5;              // Range   1 - 10
        private float m_amplitude = 1.0f;       // Range 0.1 - 2.0
        private float m_gain = 0.6f;            // Range 0.1 - 0.9
        private float m_lacunarity = 2.1042f;   // Range 1.5 - 2.5
        private float m_offset = -0.5f;         // Range-2.5 - 1.0

        /// <summary>
        /// texture
        /// </summary>
        public Texture2D Texture
        {
            get { return m_texture; }
            set { m_texture = value; }
        }
        /// <summary>
        /// Generate on paint
        /// </summary>
        public bool GenerateOnClick
        {
            get { return m_generateOnClick; }
            set { m_generateOnClick = value; }
        }
        /// <summary>
        /// Is active
        /// </summary>
        public bool Enabled
        {
            get { return m_enabled; }
            set { m_enabled = value; }
        }

        /// <summary>
        /// Noise Scale
        /// </summary>
        public float Scale
        {
            get { return m_scale; }
            set { m_scale = value < 0.01f ? 0.01f : value; }
        }
        /// <summary>
        /// Noise octaves
        /// Iterations
        /// </summary>
        public int Octaves
        {
            get { return m_octaves; }
            set { m_octaves = value; }
        }
        /// <summary>
        /// Noise amplitude
        /// </summary>
        public float Amplitude
        {
            get { return m_amplitude; }
            set { m_amplitude = value; }
        }
        /// <summary>
        /// Noise gain (per octave)
        /// </summary>
        public float Gain
        {
            get { return m_gain; }
            set { m_gain = value; }
        }
        /// <summary>
        /// Noise Lacunarity
        /// </summary>
        public float Lacunarity
        {
            get { return m_lacunarity; }
            set { m_lacunarity = value; }
        }
        /// <summary>
        /// Noise Offset
        /// </summary>
        public float Offset
        {
            get { return m_offset; }
            set { m_offset = value; }
        }

        /// <summary>
        /// Generate noise texture
        /// </summary>
        /// <param name="size"></param>
        public void Generate(Point size)
        {
            // Create texture
            Color[] colors = new Color[size.x * size.y];
            Vector2 offset = new Vector2(Random.Range(-10000.0f, 10000.0f), Random.Range(-10000.0f, 10000.0f));

            for (int i = 0; i < size.x * size.y; i++)
            { // for every pixel
                float total = 0.0f;
                float scale = m_scale / ((size.x + size.y) / 2.0f);
                float amplitude = m_amplitude;

                // Calculate point
                Vector2 p = new Vector2(i % size.x, i / size.y);

                for (int j = 0; j < m_octaves; j++)
                { // For all octaves apply perlin noise with settings
                    total += Mathf.PerlinNoise(offset.x + p.x * scale, offset.y + p.y * scale) * amplitude;
                    scale *= m_lacunarity;
                    amplitude *= m_gain;
                }

                // Final adjustments
                float v = Mathf.Clamp01(total + m_offset);

                // Apply pixel
                for (int j = 0; j < 3; j++) { colors[i][j] = v; }
                colors[i].a = 1.0f;
            }

            if (m_texture == null || m_texture.width != size.x || m_texture.height != size.y)
            { // Create new texture if not exsists
                m_texture = new Texture2D(size.x, size.y);
                m_texture.hideFlags = HideFlags.HideAndDontSave;
            }

            // Apply texture
            m_texture.SetPixels(colors);
            m_texture.Apply();
        }
    }
}
#endif
