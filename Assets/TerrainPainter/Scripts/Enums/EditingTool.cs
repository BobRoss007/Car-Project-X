////////////////////////////////////////////////////////////////////////////
//
//      Name:               EditingTool.cs
//      Author:             HOEKKII
//      
//      Description:        N/A
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;

namespace TerrainPainter
{
    [Serializable]
    public enum EditingTool
    {
        None = -1,
        Height,
        Texture,
        Foliage,
        Object,
        Settings
    }
}
#endif
