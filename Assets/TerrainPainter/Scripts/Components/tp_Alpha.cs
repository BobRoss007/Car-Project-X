////////////////////////////////////////////////////////////////////////////
//
//      Name:               tp_Alpha.cs
//      Author:             HOEKKII
//      
//      Description:        This is used for the alpha textures (brushes)
//      
////////////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine;

using UnityEditor;
using UnityEditorInternal;

namespace TerrainPainter
{
    [Serializable] public class tp_Alpha
	{
		static readonly StringBuilder Builder = new StringBuilder();

		public List<Texture2D> alphaBrushes = new List<Texture2D>();    // All brushes from resources folder
        [SerializeField] public int selectedAlphaBrush = 0;             // Selected brush
        [SerializeField] private bool m_enabled = true;                 // Alpha menu is active and so the projector

        /// <summary>
        /// Is active
        /// </summary>
        public bool Enabled
        {
            get { return m_enabled; }
            set { m_enabled = value; }
        }
        /// <summary>
        /// Get alphamaps
        /// </summary>
        public void Update()
		{
            try
            { // Get all textures in .../Resources/.../TP_Brushes/
			  //Texture2D[] __texes  = Resources.LoadAll<Texture2D>(TerrainPainter.Settings.BRUSHES_FOLDER_NAME);
			  //alphaBrushes = __texes.ToList();

				List<Texture2D> alphas = new List<Texture2D>();

				// Hurray for reflection, listed all UnityEditorInternals and its properties :) to get the default brushes
				Type type = Assembly.GetAssembly(typeof(AssetStore))
									.GetTypes()
									.FirstOrDefault(x => x.Name == "EditorResourcesUtility");
				if (type != null)
				{
					// Get unity brushes
					string v = (string)type.GetProperty("brushesPath").GetValue(typeof(string), null);
					GetTextures(alphas, v, "builtin_brush_");
				}

				// Get custom brushes in resources
				GetTextures(alphas, "", "brush_");

				// Get terrain painter brushes
				GetTextures(alphas, "TerrainPainter/Brushes/", "brush_");

				// Apply
				alphaBrushes = alphas;
				selectedAlphaBrush = Mathf.Min(alphaBrushes.Count - 1, selectedAlphaBrush);
			}
            catch (Exception e) { throw new NullReferenceException(TerrainPainter.ErrorMessages.ALPHA_NOT_FOUND + " \n Error code: " + e.ToString()); }
        }

		/// <summary>
		/// Get textures in path
		/// </summary>
		/// <param name="alphas"></param>
		/// <param name="path"></param>
		/// <param name="prefix"></param>
		static void GetTextures(ICollection<Texture2D> alphas, string path, string prefix)
		{
			Texture2D texture;
			int index = 1;
			do // begin from ../Brush_1 to ../Brush_n until there is a file not found
			{
				// Build file path
				Builder.Length = 0;
				Builder.Append(path);
				Builder.Append(prefix);
				Builder.Append(index);
				Builder.Append(".png");

				// Increase index for next texture
				index++;

				// Add texture to list
				texture = (Texture2D)EditorGUIUtility.Load(Builder.ToString());
				if (texture != null)
					alphas.Add(texture);
			} while (texture != null);
		}

		/// <summary>
		/// Get the current selected texture index
		/// </summary>
		/// <param name="ab">current alpha</param>
		public static explicit operator int (tp_Alpha ab)
        {
            return ab.selectedAlphaBrush;
        }
        /// <summary>
        /// Gets the selected texture
        /// </summary>
        /// <param name="ab">current alpha</param>
        public static explicit operator Texture2D(tp_Alpha ab)
        {
            if (ab.selectedAlphaBrush < 0 || ab.selectedAlphaBrush >= ab.alphaBrushes.Count)
            {
                throw new IndexOutOfRangeException();
            }
            return ab.alphaBrushes[ab.selectedAlphaBrush];
        }
    }
}
#endif
