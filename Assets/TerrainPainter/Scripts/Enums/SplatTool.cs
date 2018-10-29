////////////////////////////////////////////////////////////////////////////
//
//      Name:               SplatTool.cs
//      Author:             HOEKKII
//      
//      Description:        N/A
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;

namespace TerrainPainter
{
    [Serializable] public enum SplatTool
    {
        None = -1,

        // Brushes
        OneTexture,
        HeightBased,
        AngleBased,
        Sharpen,
        Blur
    }
}
#endif
