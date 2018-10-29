////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Heights.cs
//      Author:             HOEKKII
//      
//      Description:        Used to store the heighs 
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;
using TP = TerrainPainter.TerrainPainter;

namespace TerrainPainter
{
    [Serializable] public class tp_Heights
    {
        [SerializeField] private List<tp_Height> m_heights = new List<tp_Height>();  // Height ranges

        /// <summary>
        /// Height
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public tp_Height this[int index]
        {
            get { return m_heights[index]; }
            set { m_heights[index] = value; }
        }

        /// <summary>
        /// Amount of items in list
        /// </summary>
        public int Count { get { return m_heights.Count; } }

        /// <summary>
        /// Add an item to end of the list
        /// </summary>
        /// <param name="h"></param>
        public void Add(tp_Height h)
        {
            m_heights.Add(h);
        }
        
        /// <summary>
        /// Get splatmap indexed
        /// </summary>
        /// <returns></returns>
        public int[] GetAvailablePaintTextures()
        {
            int[] tex = new int[Count];
            for (int i = 0; i < Count; i++) { tex[i] = this[i].Index; }
            return tex;
        }

        /// <summary>
        /// Swap two heights
        /// </summary>
        /// <param name="i"></param>
        /// <param name="p"></param>
        public void Swap(int i, int p)
        {
            int temp = this[i].Index;
            this[i].SetIndex(this[p].Index);
            this[p].SetIndex(temp);
        }
        /// <summary>
        /// List.RemoveAt
        /// </summary>
        /// <param name="v"></param>
        public void RemoveAt(int v)
        {
            m_heights.RemoveAt(v);
        }
        /// <summary>
        /// List.Clear
        /// </summary>
        public void Clear()
        {
            m_heights.Clear();
        }
    }
    [Serializable] public class tp_Height
    {
        [SerializeField] public int m_index = 0;                // Texture index
        [SerializeField] public Vector2 m_range = Vector2.zero; // Min Max where the texture is applied

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="index"></param>
        public tp_Height()
        {
            m_index = 0;
            m_range = Vector2.zero;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="index"></param>
        public tp_Height(int index)
        {
            m_index = index;
            m_range = Vector2.zero;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="index"></param>
        /// <param name="range"></param>
        public tp_Height(int index, Vector2 range)
        {
            m_index = index;
            m_range = range;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="index"></param>
        /// <param name="heights"></param>
        public tp_Height(int index, tp_Heights heights)
        {
            m_index = index;

            if (heights.Count < 1)
            { // First range
                Range = new Vector2(0.0f, TP.Instance.TerrainData.heightmapScale.y);
                return;
            }

            // Create Range
            tp_Height last = heights[heights.Count - 1];
            float max = last.y;
            float v = last.y - last.x;
            v /= 3.0f;
            last.y -= 2.0f * v;

            // Set range
            Range = new Vector2(last.y + v, max);
        }

        /// <summary>
        /// Get texture index
        /// </summary>
        public int Index { get { return m_index; } }
        /// <summary>
        /// Min Max where the texture is applied
        /// </summary>
        public Vector2 Range { get { return m_range; } set { m_range = value; } }
        /// <summary>
        /// Min
        /// </summary>
        public float x { get { return m_range.x; } set { m_range.x = value; } }
        /// <summary>
        /// Max
        /// </summary>
        public float y { get { return m_range.y; } set { m_range.y = value; } }

        /// <summary>
        /// Set Texture index
        /// </summary>
        /// <param name="i"></param>
        public void SetIndex(int i)
        {
            m_index = i;
        }
    }
}
#endif
