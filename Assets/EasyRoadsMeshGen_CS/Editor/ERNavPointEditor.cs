using UnityEngine;
using UnityEditor;
using System.Collections;
using ERMG;

[CustomEditor(typeof(ERNavPoint))]
public class ERNavPointEditor : Editor {
	public override void OnInspectorGUI (){
		if(Application.isPlaying)
			return;
		
        DrawDefaultInspector();
		ERNavPoint myScirpt = (ERNavPoint)target;
		
		if(myScirpt.assignedMeshGen){
			GUI.color = Color.green;
			if(GUILayout.Button("Add New Nav Point"))
				myScirpt.NavPointAction(NavAction.Add);
		}
		
		if(myScirpt.assignedMeshGen){
			GUI.color = Color.white;
			string lockState = (myScirpt.lockSize) ? "(Locked!)" : "";
			if(GUILayout.Button("Lock Width " + lockState))
				myScirpt.LockSize(!myScirpt.lockSize);
		}
	}
}