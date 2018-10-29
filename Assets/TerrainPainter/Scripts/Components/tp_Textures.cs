////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Textures.cs
//      Author:             HOEKKII
//      
//      Description:        Used for storing selected texture
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using UnityEngine;

namespace TerrainPainter
{
    [Serializable] public class tp_Textures
    {
        [SerializeField] private int m_selectedTexture = 0;         // Selected paint texture
        [SerializeField] private int m_selectedCliffTexture = 1;    // Selected cliff texture
        [SerializeField] private bool m_multiSelectEnabled = true;  // Multi select enabled (only smooth/sharp tool available)
        [SerializeField] private int m_multiSelect = 0;             // Selected textures

        /// <summary>
        /// Selected paint texture
        /// </summary>
        public int SelectedTexture
        {
            get { return m_selectedTexture; }
            set { m_selectedTexture = value; }
        }
        /// <summary>
        /// Setected textures multi-select
        /// </summary>
        public int SelectedTextures
        {
            get { return m_multiSelect; }
            set { m_multiSelect = value; }
        }
        /// <summary>
        /// Selected cliff texture
        /// </summary>
        public int SelectedCliffBrush
        {
            get { return m_selectedCliffTexture; }
            set { m_selectedCliffTexture = value; }
        }
        /// <summary>
        /// Is multi select
        /// </summary>
        public bool IsMultiSelect
        {
            get { return m_multiSelectEnabled; }
            set { m_multiSelectEnabled = value; }
        }
        /// <summary>
        /// Check if selected index is selected
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public bool IsSelected(int index)
        {
            if (!m_multiSelectEnabled) { return m_multiSelect == index; }

            int comparer = 1 << index;
            return (m_multiSelect & comparer) == comparer;
        }

    }
}
#endif
