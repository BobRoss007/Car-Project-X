////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Foliage.cs
//      Author:             HOEKKII
//      
//      Description:        Selecting foliage index(es) and other options
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using UnityEngine;

namespace TerrainPainter
{
    [Serializable] public class tp_Foliages
    {
        [SerializeField] private bool m_erase = false;              // Use erase tool (false -> use paint tool)
        [SerializeField] private bool m_multiSelectEnabled = true;  // Enable selecting multiple details
        [SerializeField] private bool m_useRandom = false;          // Use random opacity every cordinate
        [SerializeField] private int m_selectedFoliage = 0;         // Selected detail by index
        [SerializeField] private int m_multiSelectedFoliage = 0;    // Selected detail by bit index
        
        /// <summary>
        /// Enable the use of selecting multiple details
        /// </summary>
        public bool IsMultiSelect
        {
            get { return m_multiSelectEnabled; }
            set { m_multiSelectEnabled = value; }
        }

        /// <summary>
        /// Selected detail only for single-select
        /// </summary>
        public int SelectedFoliage
        {
            get { return  m_selectedFoliage; }
            set { m_selectedFoliage = value; }
        }
        /// <summary>
        /// Selected details for multiselect
        /// </summary>
        public int SelectedFoliages
        {
            get { return m_multiSelectedFoliage; }
            set { m_multiSelectedFoliage = value; }
        }
        /// <summary>
        /// Check if index is selected
        /// </summary>
        /// <param name="index">detail index</param>
        /// <returns>is selected</returns>
        public bool IsSelected(int index)
        {
            if (!m_multiSelectEnabled) { return m_selectedFoliage == index; }

            int comparer = 1 << index;
            return (m_multiSelectedFoliage & comparer) == comparer;
        }
        /// <summary>
        /// Use random opacity every cordinate
        /// </summary>
        public bool UseRandom
        {
            get { return m_useRandom; }
            set { m_useRandom = value; }
        }
        /// <summary>
        /// Erase details
        /// False -> paint details
        /// </summary>
        public bool Erase
        {
            get { return m_erase; }
            set { m_erase = value; }
        }
    }
}
#endif
