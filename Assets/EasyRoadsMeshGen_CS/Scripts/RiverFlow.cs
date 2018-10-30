using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[AddComponentMenu("Easy Roads Mesh Gen/River Flow")]
public class RiverFlow : MonoBehaviour {
	public Vector2 direction;

	public bool  bumpmap = true;

	private float x = 0;
	private float y = 0;

	/*private Mesh mesh;
	private Vector3[] verts;
	private Vector2[] uvs0;
	private Vector2[] uvs;
	private int size;

	void  Start (){
		mesh = GetComponent<MeshFilter>().mesh;
		verts = mesh.vertices;
		uvs0 = mesh.uv;
		size = verts.Length;
		uvs = new Vector2[size];
	}*/

	void  Update (){
			x += direction.x * Time.deltaTime;
			y += direction.y * Time.deltaTime;
			x = x%1.0f;
			y = y%1.0f;
			GetComponent<Renderer>().material.SetTextureOffset("_MainTex", new Vector2(x,y));
			if(bumpmap)
				GetComponent<Renderer>().material.SetTextureOffset("_BumpMap", new Vector2(x,y));
	}
}
