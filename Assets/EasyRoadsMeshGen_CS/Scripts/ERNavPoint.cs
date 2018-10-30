using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif
using ERMG;

namespace ERMG {
	public enum NavAction {
		Add,
		Delete
	}
}
	
public class ERNavPoint : MonoBehaviour {
	float gizmoSize = 1f;
	[HideInInspector]
	public ERMeshGen assignedMeshGen; //assigned mesh gen scripts
	[HideInInspector]
	public bool lockSize = false;
	//[HideInInspector]
	public float lockedWidth = 0f;
	
	void  OnDrawGizmos (){
		#if UNITY_EDITOR
		Gizmos.DrawIcon(transform.position, "EasyRoadsMeshGen/waypoint_icon.png", true);
		
		if(!Selection.Contains (gameObject))
			return;
		//Gizmos.DrawSphere(transform.position,0.12f * gizmoSize);
		//Gizmos.DrawWireSphere(transform.position,0.12f * gizmoSize);

		Gizmos.color = Color.yellow;
		if(assignedMeshGen){
			Gizmos.DrawLine(transform.position,transform.position - transform.forward*transform.localScale.z);
				Gizmos.DrawLine(transform.position,transform.position + transform.forward*transform.localScale.z);
			Gizmos.DrawWireSphere(transform.position + transform.forward*transform.localScale.z,0.08f * gizmoSize);
			Gizmos.DrawWireSphere(transform.position - transform.forward*transform.localScale.z,0.08f * gizmoSize);
		}
		
		Gizmos.color = Color.red;
		Gizmos.DrawLine(transform.position,transform.position+ transform.right*gizmoSize/2f);
		Gizmos.color = Color.green;
		Gizmos.DrawLine(transform.position,transform.position+ transform.up*gizmoSize/2f);
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(transform.position,transform.position+ transform.forward*gizmoSize/2f);
		#endif
	}
	
	public void NavPointAction (NavAction act){
		if(act == NavAction.Add)
			assignedMeshGen.CreateNavPoint();
		if(act == NavAction.Delete)
			assignedMeshGen.DeleteNavPoint();
		
	}
	
	public void LockSize (bool state){
		#if UNITY_EDITOR
		SerializedObject so = new SerializedObject(this);
		SerializedProperty so_lockSize = so.FindProperty("lockSize");
		SerializedProperty so_lockedWidth = so.FindProperty("lockedWidth");
		
		if(!assignedMeshGen){
			Debug.Log("No MeshGen script assigned! Lock operation failed!");
			so_lockSize.boolValue = false;
			return;
		}
		
		so_lockSize.boolValue = state;
		so_lockedWidth.floatValue = assignedMeshGen.deltaWidth;
		so.ApplyModifiedProperties();
		#endif
	}
}
