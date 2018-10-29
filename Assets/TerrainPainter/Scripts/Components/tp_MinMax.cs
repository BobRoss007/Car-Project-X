////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_MinMax.cs
//      Author:             HOEKKII
//      
//      Description:        Used for the ramp
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using UnityEngine;

namespace TerrainPainter
{
    [Serializable] public class tp_MinMax
    {
        [SerializeField] private Vector2 m_value = new Vector2(45.0f, 65.0f);    // Angle value
        [SerializeField] private bool m_active = true;                           // Is active

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="v"></param>
        public tp_MinMax(Vector2 v)
        {
            m_value = v;
        }
        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="min"></param>
        /// <param name="max"></param>
        public tp_MinMax(float min, float max)
        {
            m_value = Vector2.zero;
            m_value.x = min;
            m_value.y = max;
        }

        /// <summary>
        /// Min
        /// </summary>
        public float Min
        {
            get { return m_value.x; }
            set { m_value.x = Mathf.Min(value, Max); }
        }
        /// <summary>
        /// Max
        /// </summary>
        public float Max
        {
            get { return m_value.y; }
            set { m_value.y = Mathf.Max(value, Min); }
        }
        public Vector2 Value
        {
            get { return m_value; }
        }
        /// <summary>
        /// Is active
        /// </summary>
        public bool Active
        {
            get { return m_active; }
            set { m_active = value; }
        }

        /// <summary>
        /// Set
        /// </summary>
        /// <param name="v"></param>
        public void Set(Vector2 v)
        {
            Min = v.x;
            Max = v.y;
        }
        /// <summary>
        /// Set
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        public void Set(float x, float y)
        {
            Min = x;
            Max = y;
        }
        /// <summary>
        /// Get the value between angles
        /// IverseLerp
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public float LerpValue(float angle)
        {
            return Mathf.InverseLerp(Min, Max, angle);
        }
    }
}
#endif
