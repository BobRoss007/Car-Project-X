////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_ComponentBaseEditor.cs
//      Author:             HOEKKII
//      
//      Description:        The base of an editor specific for 
//                          TerrainPainerEditor Components.
//      
////////////////////////////////////////////////////////////////////////////

namespace TerrainPainter
{
    public abstract class tp_ComponentBaseEditor : tp_Editor
    {
        internal abstract void Paint(bool paintAll);
        internal abstract UndoType UndoType { get; }
    }
}
