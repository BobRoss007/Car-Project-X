////////////////////////////////////////////////////////////////////////////
//
//      Name:               BrushState.cs
//      Author:             HOEKKII
//      
//      Description:        N/A
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;

namespace TerrainPainter
{
    [Serializable] public enum BrushState
    {
        Disabled,
        Enabled,
        Paint
    }
}
#endif
