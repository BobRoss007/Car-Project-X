
////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Utilities.cs
//      Author:             HOEKKII
//      
//      Description:        Some utilities/helpers
//      
////////////////////////////////////////////////////////////////////////////

using UnityEngine;

namespace TerrainPainter
{
    public class tp_Utilities
    {
		#if UNITY_EDITOR
		#region PaintUtilities
		/// <summary>
		/// World point to Splatmap point
		/// </summary>
		/// <param name="pos">position</param>
		/// <param name="terrainSize">terrain size</param>
		/// <param name="td">terrain data</param>
		/// <returns></returns>
		public static float WorldToSplatf(float pos, float terrainSize, TerrainData td)
        {
            return (pos / terrainSize) * td.alphamapResolution;
        }
        /// <summary>
        /// World point to Splatmap point
        /// </summary>
        /// <param name="pos">position</param>
        /// <param name="td">terrain data</param>
        /// <returns></returns>
        public static float WorldToSplatf(float pos, TerrainData td)
        {
            return WorldToSplatf(pos, td.size.x, td);
        }
        /// <summary>
        /// World point to Splatmap point
        /// </summary>
        /// <param name="pos">position</param>
        /// <param name="terrainSize">terrain size</param>
        /// <param name="td">terrain data</param>
        /// <returns></returns>
        public static int WorldToSplat(float pos, float terrainSize, TerrainData td)
        {
            return Mathf.RoundToInt(WorldToSplatf(pos, terrainSize, td));
        }
        /// <summary>
        /// World point to Splatmap point
        /// </summary>
        /// <param name="pos">position</param>
        /// <param name="td">terrain data</param>
        /// <returns></returns>
        public static int WorldToSplat(float pos, TerrainData td)
        {
            return Mathf.RoundToInt(WorldToSplatf(pos, td));
        }

        public static float WorldToDetailf(float pos, float terrainSize, TerrainData td)
        {
            return (pos / terrainSize) * td.detailResolution;
        }
        public static int WorldToDetail(float pos, float size, TerrainData td)
        {
            return Mathf.RoundToInt(WorldToDetailf(pos, size, td));
        }
        public static int WorldToDetail(float pos, TerrainData td)
        {
            return WorldToDetail(pos, td.size.x, td);
        }
        

        #endregion

        #region Splat Prototypes
        /// <summary>
        /// Edit splatprototype
        /// </summary>
        /// <param name="terrainData"></param>
        /// <param name="index"></param>
        /// <param name="main"></param>
        /// <param name="norm"></param>
        /// <param name="size"></param>
        /// <param name="offset"></param>
        /// <param name="metallic"></param>
        /// <param name="smoothness"></param>
        /// <param name="specularColor"></param>
        public static void EditSplatTexture(TerrainData terrainData, int index, Texture2D main, Texture2D norm, Vector2 size, Vector2 offset, float metallic, float smoothness, Color specularColor)
        {
            if (index < 0 || index >= terrainData.splatPrototypes.Length) { return; }

            int ttLength = terrainData.splatPrototypes.Length;
            SplatPrototype[] sp = new SplatPrototype[ttLength];

            for (int i = 0; i < ttLength; i++)
            {
                if (index == i)
                {
                    sp[index] = new SplatPrototype();
                    sp[index].texture = main;
                    sp[index].normalMap = norm;
                    sp[index].tileSize = size;
                    sp[index].tileOffset = offset;
                    sp[index].metallic = metallic;
                    sp[index].smoothness = smoothness;
                    sp[index].specular = specularColor;
                }
                else { sp[i] = terrainData.splatPrototypes[i]; }

            }

            terrainData.splatPrototypes = sp;
        }
        /// <summary>
        /// Add splatprototype
        /// </summary>
        /// <param name="terrainData"></param>
        /// <param name="main"></param>
        /// <param name="norm"></param>
        /// <param name="size"></param>
        /// <param name="offset"></param>
        /// <param name="metallic"></param>
        /// <param name="smoothness"></param>
        /// <param name="specularColor"></param>
        public static void AddSplatTexture(TerrainData terrainData, Texture2D main, Texture2D norm, Vector2 size, Vector2 offset, float metallic, float smoothness, Color specularColor)
        {
            int ttLength = terrainData.splatPrototypes.Length;
            SplatPrototype[] sp = new SplatPrototype[ttLength + 1];

            for (int i = 0; i < ttLength; i++)
            {
                sp[i] = terrainData.splatPrototypes[i];
            }

            sp[ttLength] = new SplatPrototype();
            sp[ttLength].texture = main;
            sp[ttLength].normalMap = norm;
            sp[ttLength].tileSize = size;
            sp[ttLength].tileOffset = offset;
            sp[ttLength].metallic = metallic;
            sp[ttLength].smoothness = smoothness;
            sp[ttLength].specular = specularColor;

            terrainData.splatPrototypes = sp;
        }
        /// <summary>
        /// Remove splatprototype
        /// </summary>
        /// <param name="td"></param>
        /// <param name="index"></param>
        public static void RemoveSplatTexture(TerrainData td, int index)
        {
            Point size = new Point(td.alphamapWidth, td.alphamapHeight);
            float[,,] alphamaps = td.GetAlphamaps(0, 0, size.x, size.y);
            float[,,] newAlphaMaps = new float[alphamaps.GetLength(0), alphamaps.GetLength(1), alphamaps.GetLength(2) - 1]; // Duplicate except the removed

            for (int i = 0; i < size.y; i++)
            {
                for (int j = 0; j < size.x; j++)
                {
                    for (int k = 0; k < index; k++) { newAlphaMaps[i, j, k] = alphamaps[i, j, k]; }
                    for (int l = index + 1; l < alphamaps.GetLength(2); l++) { newAlphaMaps[i, j, l - 1] = alphamaps[i, j, l]; }
                }
            }
            for (int i = 0; i < size.y; i++)
            {
                for (int j = 0; j < size.x; j++)
                {
                    float alpha = 0f;
                    for (int k = 0; k < alphamaps.GetLength(2) - 1; k++) { alpha += newAlphaMaps[i, j, k]; }
                    if (alpha >= 0.01f) { for (int k = 0; k < alphamaps.GetLength(2) - 1; k++) { newAlphaMaps[i, j, k] *= (1.0f / alpha); } }
                    else { for (int k = 0; k < alphamaps.GetLength(2) - 1; k++) { newAlphaMaps[i, j, k] = ((k != 0) ? 0f : 1f); } }
                }
            }

            SplatPrototype[] newSplat = new SplatPrototype[td.splatPrototypes.Length - 1];
            for (int i = 0; i < index; i++) { newSplat[i] = td.splatPrototypes[i]; }
            for (int i = index + 1; i < alphamaps.GetLength(2); i++) { newSplat[i - 1] = td.splatPrototypes[i]; }
            td.splatPrototypes = newSplat;
            td.SetAlphamaps(0, 0, newAlphaMaps);
        }
		#endregion
		#endif
	}
}
