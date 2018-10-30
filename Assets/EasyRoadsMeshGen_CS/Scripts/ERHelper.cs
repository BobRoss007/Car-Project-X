using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[AddComponentMenu("Easy Roads Mesh Gen/ER Helper")]
public class ERHelper : MonoBehaviour {
	
	public ERMeshGen meshGen;
	
	public void Init () {
		if(!meshGen)
			if(GetComponent<ERMeshGen>())
				meshGen = GetComponent<ERMeshGen>();
			else
				meshGen = (ERMeshGen) GameObject.FindObjectOfType(typeof(ERMeshGen));
	}
	
	public void AutoFix () {
		FindNavPoints();
		FindBorders();
	}
	
	public void FindNavPoints(){
		meshGen.FindNavPoints();	
	}
	
	public void FindBorders () {
		meshGen.FindLeftBorder();
		meshGen.FindRightBorder();
	}
}
