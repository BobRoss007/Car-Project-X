////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Containers.cs
//      Author:             HOEKKII
//      
//      Description:        Containers for serializing
//      
////////////////////////////////////////////////////////////////////////////


using UnityEngine;

namespace TerrainPainter
{
	#if UNITY_EDITOR
	public class tp_Containers
    {
        public class SplatmapContainer
        {
            public float[][][] splat;

            public void SetSplatmap(float[,,] ts)
            {
                splat = new float[ts.GetLength(0)][][];
                for (int i = 0; i < ts.GetLength(0); i++)
                {
                    splat[i] = new float[ts.GetLength(1)][];
                    for (int j = 0; j < ts.GetLength(1); j++)
                    {
                        splat[i][j] = new float[ts.GetLength(2)];
                        for (int k = 0; k < ts.GetLength(2); k++)
                        {
                            splat[i][j][k] = ts[i, j, k];
                        }
                    }
                }
            }
            public float[,,] GetSplatmap()
            {
                int iLength = splat.Length;
                int jLength = splat[iLength - 1].Length;
                int kLength = splat[iLength - 1][jLength - 1].Length;

                float[,,] ts = new float[iLength, jLength, kLength];

                for (int i = 0; i < ts.GetLength(0); i++)
                {
                    for (int j = 0; j < ts.GetLength(1); j++)
                    {
                        for (int k = 0; k < ts.GetLength(2); k++)
                        {
                            ts[i, j, k] = splat[i][j][k];
                        }
                    }
                }
                return ts;
            }
        }
        public class SettingsContainer
        {
            public float opacity;
            public float strength;
            public float size;

            public tp_Height[] heights;
            public Vector2 ramp;

            public SettingsContainer()
            {
                opacity = 100.0f;
                strength = 100.0f;
                size = 100.0f;

                heights = new tp_Height[0];
                ramp = new Vector2(45.0f, 60.0f);
            }
        }
    }
	#endif
}
