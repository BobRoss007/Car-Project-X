////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_History.cs
//      Author:             HOEKKII
//      
//      Description:        Here are splatmaps stored
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using TP = TerrainPainter.TerrainPainter;

namespace TerrainPainter
{
    public class tp_History
    {
        public class HistoryContainer
        {
            public readonly UndoType undoType;
            public readonly string message;
            public float[,,] splatData = null;
            public int[][,] detailData = null;

            public HistoryContainer(UndoType type, string msg = default(string))
            {
                undoType = type;
                message = msg;
            }
        }

        private bool m_hasCreatedCurrentRestore = false;            // Checks if current is stored
        private int m_capacity = 3;                                 // Max history size
        private int m_currentIndex = 0;                             // Current index in history
        private List<HistoryContainer> history = new List<HistoryContainer>();    // Here is where all the splatmaps are stored

        /// <summary>
        /// Max hirstory size
        /// </summary>
        public int Capacity { get { return m_capacity; } set { m_capacity = value < 0 ? 0 : value; } }
        /// <summary>
        /// Current history index
        /// </summary>
        public int CurrentIndex { get { return m_currentIndex; } }
        /// <summary>
        /// Current history size
        /// </summary>
        public int CurrentCapacity { get { return history.Count; } }

        /// <summary>
        /// Create undo point
        /// </summary>
        /// <param name="saveMsg"></param>
        public void CreateRestorePoint(UndoType type, string saveMsg = default(string))
        {
            // Get
            TerrainData td = TP.Instance.TerrainData;
            if (td.splatPrototypes.Length < 2) { return; }
            while (m_currentIndex != history.Count) { history.RemoveAt(m_currentIndex); }

            // Add and clean
            HistoryContainer c = new HistoryContainer(type, saveMsg);
            switch (type)
            {
                case UndoType.Splatmap:
                    c.splatData = td.GetAlphamaps(0, 0, td.alphamapWidth, td.alphamapHeight);
                    break;
                case UndoType.DetailMap:
                    c.detailData = new int[td.detailPrototypes.Length][,];
                    for (int i = 0; i < td.detailPrototypes.Length; i++) { c.detailData[i] = td.GetDetailLayer(0, 0, td.detailWidth, td.detailHeight, i); }
                    break;
                //default: return;
            }

            history.Add(c);
            while (history.Count > m_capacity + 1) { history.RemoveAt(0); }
            m_currentIndex = history.Count;
            m_hasCreatedCurrentRestore = false;
        }

        /// <summary>
        /// Check if can do an undo
        /// </summary>
        /// <returns></returns>
        public bool CanUndo()
        {
            if (m_hasCreatedCurrentRestore) { if (m_currentIndex <= 1) return false; }
            else if (m_currentIndex < 1) { return false; }
            return true;
        }

        /// <summary>
        /// Perform undo
        /// </summary>
        public void Undo()
        {
            if (!CanUndo()) { return; }
            if (m_currentIndex == history.Count && !m_hasCreatedCurrentRestore) { CreateRestorePoint(UndoType.All); }
            m_hasCreatedCurrentRestore = true;

            m_currentIndex -= 1;
            switch (history[m_currentIndex - 1].undoType)
            {
                case UndoType.Splatmap:
                    TP.Instance.TerrainData.SetAlphamaps(0, 0, history[m_currentIndex - 1].splatData);
                    break;
                case UndoType.DetailMap:
                    for (int i = 0; i < TP.Instance.TerrainData.detailPrototypes.Length; i++)
                    {
                        if (i >= history[m_currentIndex - 1].detailData.Length) { break; }
                        TP.Instance.TerrainData.SetDetailLayer(0, 0, i, history[m_currentIndex - 1].detailData[i]);
                    }
                    break;
                default:
                    TP.Instance.TerrainData.SetAlphamaps(0, 0, history[m_currentIndex - 1].splatData);
                    for (int i = 0; i < TP.Instance.TerrainData.detailPrototypes.Length; i++)
                    {
                        if (i >= history[m_currentIndex - 1].detailData.Length) { break; }
                        TP.Instance.TerrainData.SetDetailLayer(0, 0, i, history[m_currentIndex - 1].detailData[i]);
                    }
                    break;
            }

        }
        /// <summary>
        /// Perform redo
        /// </summary>
        public void Redo()
        {
            if (m_currentIndex >= history.Count) { return; }
            m_currentIndex += 1;
            //TP.Instance.TerrainData.SetAlphamaps(0, 0, history[m_currentIndex - 1]);

            switch (history[m_currentIndex - 1].undoType)
            {
                case UndoType.Splatmap:
                    TP.Instance.TerrainData.SetAlphamaps(0, 0, history[m_currentIndex - 1].splatData);
                    break;
                case UndoType.DetailMap:
                    for (int i = 0; i < TP.Instance.TerrainData.detailPrototypes.Length; i++)
                    {
                        if (i >= history[m_currentIndex - 1].detailData.Length) { break; }
                        TP.Instance.TerrainData.SetDetailLayer(0, 0, i, history[m_currentIndex - 1].detailData[i]);
                    }
                    break;
                default:
                    if (history[m_currentIndex - 1].splatData != null)
                        TP.Instance.TerrainData.SetAlphamaps(0, 0, history[m_currentIndex - 1].splatData);
                    if (history[m_currentIndex - 1].detailData != null)
                        for (int i = 0; i < TP.Instance.TerrainData.detailPrototypes.Length; i++)
                        {
                            if (i >= history[m_currentIndex - 1].detailData.Length) { break; }
                            TP.Instance.TerrainData.SetDetailLayer(0, 0, i, history[m_currentIndex - 1].detailData[i]);
                        }
                    break;
            }
        }
    }
}
#endif
