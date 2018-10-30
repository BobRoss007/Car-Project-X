using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

[AddComponentMenu("Easy Roads Mesh Gen/Mesh Gen")]
public class ERMeshGen : MonoBehaviour {
	public static string NAV_POINT_NAMES = "Nav Point";
	public static string RIGHT_BORDER_NAME = "rightBorderMeshObj";
	public static string LEFT_BORDER_NAME = "leftBorderMeshObj";
	private int updateSkip = 3;
	
	public static ERMeshGen lastSelectedMeshGen;
		
	public Transform[] navPoints = new Transform[0];
	
	[HideInInspector]
	public float deltaWidth = 1.2f;
	[HideInInspector]
	public int subdivision = 1;
	[HideInInspector]
	public float uvScale = 1;
	[HideInInspector]
	public Vector3[] navPointsBeta_p = new Vector3[0]; //nav points positions after subdivision
	private Vector3[] newVertices = new Vector3[0];
	private Vector2[] newUV = new Vector2[0];
	private int[] newTriangles = new int[0];
	private int[] quadMatrix = {2,1,0,2,3,1};
	private int
		triCount = 0,
		uvSetCount = 0,
		updateLoopCount = 0;

	[HideInInspector]
	public float groundOffset = 0.1f;
	private int lastParentPointsUpd = 0;
	
	[HideInInspector]
	public int
		enableHelp = 1,
		curveControlState = 1, //manual, automatic 1;
		updatePointsMode = 0,
		parentPoints = 0,
		includeCollider = 1,
		uvSet = 2, //0 - per quad, 1 - cube projection, 2 - width-to-Length, 3 - stretch single texture
		borderUvSet = 0, //0 - per quad, 1 - top projection
		enableMeshBorders = 0;
		
	[HideInInspector]
	public AnimationCurve borderCurve = new AnimationCurve(new Keyframe(0, 0), new Keyframe(0.3f, 0.6f), new Keyframe(1, 0.6f)); //the points in 2d plane
	[HideInInspector]
	public GameObject
		leftBorderMeshObj,
		rightBorderMeshObj;

	private Vector2[] lBorderNavPoints; //the points in 2d plane
	private Vector2[] rBorderNavPoints;
	private Vector3[] leftBorderVertices;
	private Vector2[] leftUV = new Vector2[0];
	private int[] leftTriangles = new int[0];
	private Vector3[] rightBorderVertices;
	private Vector2[] rightUV = new Vector2[0];
	private int[] rightTriangles = new int[0];
	private int borderCount = 0;
	private int xRelative = 0;

	void  OnDrawGizmos (){
		if(Application.isPlaying)
			return;
			
		if(updatePointsMode == 1)
			return;
		
		#if UNITY_EDITOR
		if(updatePointsMode == 0){
			bool  preventUpdate = true;
			if(!Selection.Contains(gameObject)){
				for(int s= 0; s < navPoints.Length; s++)
					if(navPoints[s] != null) 
						if(Selection.Contains(navPoints[s].gameObject))
							preventUpdate = false;
						else
							if(navPoints[s].gameObject.GetComponent<ERPointSnap>())
								if(navPoints[s].GetComponent<ERPointSnap>().snapped)
									if(navPoints[s].GetComponent<ERPointSnap>().snappedToPoint != null)
										if(Selection.Contains(navPoints[s].GetComponent<ERPointSnap>().snappedToPoint.root.gameObject)
											||
										Selection.Contains(navPoints[s].GetComponent<ERPointSnap>().snappedToPoint.gameObject))
											preventUpdate = false;
								
			}else
				preventUpdate = false;
			
			if(preventUpdate)
				return;
		}
		#endif
		
		for(int v= 0; v < newVertices.Length; v++)
			Gizmos.DrawIcon(newVertices[v] + transform.position, "EasyRoadsMeshGen/vertex_icon.png", true);
		
		Gizmos.color = new Color(0,0,1,0.8f);

		subdivision = Mathf.Clamp(subdivision,1,999);
		
		if(updateLoopCount < updateSkip)
			updateLoopCount++;
		else if(subdivision > 0){
			if(updatePointsMode == 2)
				SetVerts();
			
			if(updatePointsMode == 3 || updatePointsMode == 0)
				UpdateData();
			updateLoopCount = 0;
		}
	}

	/*private void  DrawTextGizmo ( Vector3 pos ,   string str  ){
		FIXME_VAR_TYPE editorCamera= Camera.current;
		FIXME_VAR_TYPE characterWidth= 8;
		FIXME_VAR_TYPE screenPoint= editorCamera.WorldToScreenPoint(pos);
		for(int i = 0; i < str.Length; i++){
			FIXME_VAR_TYPE worldPoint= editorCamera.ScreenToWorldPoint(Vector3(screenPoint.x + characterWidth*i, screenPoint.y, screenPoint.z));
			Gizmos.DrawIcon(worldPoint, "gizmotext/" + str.ToLower().Substring(i,1) + ".png",false);
		}
	}*/
	

	public void  GenerateFirstMesh (){
		if(!GetComponent<MeshFilter>()){
			gameObject.AddComponent<MeshFilter>();
		}
		if(!GetComponent<MeshRenderer>()){
			gameObject.AddComponent<MeshRenderer>();
		}
		ResetMesh();
	}

	public void  UpdateData (){
		if(Application.isPlaying)
			return;
		
		if(lastParentPointsUpd != parentPoints){
			ReparentPoints(!(parentPoints == 0));
		}
		
		#if UNITY_EDITOR
		for(int _key = 0; _key < borderCurve.length; _key++){
			AnimationUtility.SetKeyLeftTangentMode(borderCurve,_key,AnimationUtility.TangentMode.Linear);
			AnimationUtility.SetKeyRightTangentMode(borderCurve,_key,AnimationUtility.TangentMode.Linear);
		}
		#endif
		
		GenerateMesh();
	}

	public void CreateNavPoint (){
		FindNavPoints();
		
		if(navPoints.Length > 0)
			if(navPoints[navPoints.Length-1].GetComponent<ERPointSnap>())
				if(navPoints[navPoints.Length-1].GetComponent<ERPointSnap>().snapped){
					Debug.LogWarning("Easy Roads: End point <" + navPoints[navPoints.Length-1].gameObject.name + "> is snapped. Unsnap it to add new points!");
					return;
				}
			
		GameObject navPointObject= new GameObject();
		navPointObject.name = NAV_POINT_NAMES + " " + navPoints.Length;
		if(parentPoints == 1 && navPoints.Length > 0)
			navPointObject.transform.parent = navPoints[navPoints.Length - 1];
		else
			navPointObject.transform.parent = transform;
		
		navPointObject.AddComponent<ERNavPoint>();
		navPointObject.GetComponent<ERNavPoint>().assignedMeshGen = this;
		navPointObject.AddComponent<ERPointSnap>();
		#if UNITY_EDITOR
		if(navPoints.Length >= 1)
			Selection.activeGameObject = navPointObject;
		#endif 
		if(navPoints.Length > 0){
			navPointObject.transform.position = navPoints[navPoints.Length - 1].forward*deltaWidth + navPoints[navPoints.Length - 1].position;
			navPointObject.transform.rotation = navPoints[navPoints.Length - 1].rotation;
		}else{
			navPointObject.transform.position = transform.position;
		}
		
		FindNavPoints();
		#if UNITY_EDITOR
		Undo.RegisterCreatedObjectUndo(navPointObject, "Nav Point");
		#endif
		UpdateData();
	}

	public void  DeleteNavPoint (){
		#if UNITY_EDITOR
		//delete last
		if(navPoints[navPoints.Length -1].GetComponent<ERPointSnap>())
			Undo.DestroyObjectImmediate(navPoints[navPoints.Length -1].GetComponent<ERPointSnap>());
		Undo.DestroyObjectImmediate(navPoints[navPoints.Length -1].gameObject);
		FindNavPoints();
		UpdateData();
		#endif
	}

	private void  GenerateMesh (){
		if(subdivision <= 0)
			return;
		
		subdivision = Mathf.Clamp(subdivision,1,999);
		
		ScaleControlNavPoints();
		SetVerts();
		SetTriangles();
		SetUVs();
		if(enableMeshBorders == 1)
			SetBorderUVs();
		
		//GetComponent<MeshFilter>().sharedMesh.Clear();
		Mesh mesh = new Mesh (); 
		if(GetComponent<MeshFilter>())
			GetComponent<MeshFilter>().sharedMesh = mesh;
		mesh.vertices = newVertices;
		mesh.uv = newUV;
		mesh.triangles = newTriangles;
		mesh.RecalculateNormals();
		
		EnableBorders((enableMeshBorders == 1));
		
		UpdateCollider(mesh);
	}
	
	public void OnDuplicationEvent () {
		for(int a= 0; a < navPoints.Length; a++){
			if(navPoints[a]){
				if(navPoints[a].GetComponent<ERPointSnap>() != null){
					navPoints[a].GetComponent<ERPointSnap>().ClearSnap();
				}
			}
		}
		
		//Debug.Log("Dupe event " + gameObject.name);
	}
	
	public void OnDuplicationEventSrc () {
		UpdateData();
		//Debug.Log("Dupe event SRC " + gameObject.name);
	}

	private void  ScaleControlNavPoints (){
		for(int a= 0; a < navPoints.Length; a++){
			if(navPoints[a]){
				Vector3 navPointLocalScl = new Vector3(Mathf.Clamp(navPoints[a].localScale.x,0.001f,Mathf.Infinity), Mathf.Clamp(navPoints[a].localScale.y,0.001f,Mathf.Infinity), Mathf.Clamp(navPoints[a].localScale.z,0.001f,Mathf.Infinity));
				navPoints[a].localScale = navPointLocalScl;
			}
		}
	}

	private void  SetVerts (){
			if(navPoints.Length > 0)
				navPointsBeta_p = new Vector3[int.Parse( "" + (navPoints.Length-1) * subdivision)];
			else
				navPointsBeta_p = new Vector3[0];
			newVertices = new Vector3[(navPointsBeta_p.Length + 1) * 2];
			//get borderNavPoints
			lBorderNavPoints = new Vector2[borderCurve.length];
			rBorderNavPoints = new Vector2[borderCurve.length];
			
			for(int v= 0; v < borderCurve.length; v++){ //assign the borderNavPoints : Vector2 values based on the borderCurve
				rBorderNavPoints[v] = new Vector2(borderCurve.keys[v].time, borderCurve.keys[v].value);
			}
			
			if(lBorderNavPoints.Length > 0)
				leftBorderVertices = new Vector3[borderCurve.length*navPointsBeta_p.Length + borderCurve.length];
			if(rBorderNavPoints.Length > 0)
				rightBorderVertices = new Vector3[borderCurve.length*navPointsBeta_p.Length + borderCurve.length];
			
			borderCount = 0;
			
			for(int a= 0; a < navPoints.Length; a++){
				int previous = 0; //previous point in array
				int next = 0; //next point in array
				
				//get previous and next points relationship
				if(a > 0)
					previous = a-1;
				else
					previous = 0;
				
				if(a < navPoints.Length - 1)
					next = a + 1;
				
				
				if(navPoints[previous] && navPoints[a] && navPoints[next]){
					
					//calculate xRelative (based on z-axis position) which will be used to tell the orientation of the points
					if(curveControlState != 0){
						if(navPoints[a] && navPoints[next])
							if(navPoints[a].position.z > navPoints[next].position.z)
								xRelative = -1;
							else
								xRelative = 1;
					}
					
					//calculate direction
					//if(navPoints[a]){
					Vector3 direction1 = navPoints[previous].position - navPoints[a].position;
					Vector3 direction2 = navPoints[a].position - navPoints[next].position;
					Vector3 midDirection = Vector3.zero;
					midDirection = Vector3.Lerp(direction1, direction2, 0.5f); //Vector3.Distance(navPoints[previous].position,navPoints[next].position)/Vector3.Distance(navPoints[previous].position,navPoints[a].position)
					Vector3 betaEuler = new Vector3(Vector3.Angle(midDirection,-Vector3.up) - 90,
					(Vector3.Angle(midDirection,Vector3.right) * xRelative  - 90),
					navPoints[a].localEulerAngles.z);
					
					//assign the point rotation
					if(curveControlState != 0){
						if(navPoints[a].GetComponent<ERPointSnap>()){ //Snap Points Check
							if(!navPoints[a].GetComponent<ERPointSnap>().snapped){
								if(a < navPoints.Length - 1)
									navPoints[a].localEulerAngles = betaEuler;
								else if(navPoints.Length > 1){
									if(navPoints[a].position.z < navPoints[previous].position.z)
										xRelative = -1;
									else
										xRelative = 1;
									navPoints[a].localEulerAngles = new Vector3(Vector3.Angle(direction1,Vector3.up) - 90,
									(Vector3.Angle(direction1,Vector3.right) * xRelative  - 90),
									navPoints[a].localEulerAngles.z);
								}
							}
						}
						else{
							if(a < navPoints.Length - 1)
									navPoints[a].localEulerAngles = betaEuler;
								else if(navPoints.Length > 1){
									if(navPoints[a].position.z < navPoints[previous].position.z)
										xRelative = -1;
									else
										xRelative = 1;
									navPoints[a].localEulerAngles = new Vector3(Vector3.Angle(direction1,Vector3.up) - 90,
									(Vector3.Angle(direction1,Vector3.right) * xRelative  - 90),
									navPoints[a].localEulerAngles.z);
								}
						}
					}
					
					//Subdivision points position
					if(a < navPoints.Length - 1){ //inbetween the points
						navPointsBeta_p[a*subdivision] = navPoints[a].position; //set overlapping points
						
						float xCof;
						for(int b= 1; b < subdivision; b++){ //in-between points
								xCof = float.Parse( "" + float.Parse( "" + b)/float.Parse( "" + subdivision));
								Vector3 ap; Vector3 bp; Vector3 cp; Vector3 dp; Vector3 ep; Vector3 fp;
								ap = Vector3.Lerp(navPoints[a].position, navPoints[a].forward * GetPointScale(a).z + navPoints[a].position, xCof);
								cp = Vector3.Lerp(-navPoints[next].forward * GetPointScale(next).z + navPoints[next].position, navPoints[next].position, xCof);
								bp = Vector3.Lerp(navPoints[a].forward * GetPointScale(a).z + navPoints[a].position, -navPoints[next].forward * GetPointScale(next).z + navPoints[next].position, xCof);
								dp = Vector3.Lerp(ap,bp, xCof);
								ep = Vector3.Lerp(bp,cp, xCof);
								fp = Vector3.Lerp(dp,ep, xCof);
								navPointsBeta_p[a*subdivision + b] = fp;
							
								//Post-Subdivision Vertices <<<----
								Vector3 gTng= GetTangent(dp,ep);
								Vector3 leftRight= -GetBinormal(gTng, navPoints[a].up, navPoints[next].up, xCof);
								newVertices[(a*subdivision + b)*2] = navPointsBeta_p[a*subdivision + b] - leftRight * (deltaWidth/2)
								* Mathf.Lerp(float.Parse( "" + GetPointScale(a).x),float.Parse( "" + GetPointScale(next).x), float.Parse( "" + b)/float.Parse( "" + subdivision)) - transform.position;
								newVertices[(a*subdivision + b)*2 + 1] = navPointsBeta_p[a*subdivision + b] + leftRight * (deltaWidth/2)
								* Mathf.Lerp(float.Parse( "" + GetPointScale(a).x),float.Parse( "" + GetPointScale(next).x), float.Parse( "" + b)/float.Parse( "" + subdivision)) - transform.position;
						}
					
						
						
					}//a-1 restriction:end
						for(int bord= 0; bord < subdivision; bord++){ //the beta points loop inside each segment
								float xCofb= float.Parse( "" + bord)/float.Parse( "" + subdivision);
								Vector3 apb; Vector3 bpb; Vector3 cpb; Vector3 dpb; Vector3 epb; Vector3 fpb;
								apb = Vector3.Lerp(navPoints[a].position, navPoints[a].forward * GetPointScale(a).z + navPoints[a].position, xCofb);
								cpb = Vector3.Lerp(-navPoints[next].forward * GetPointScale(next).z + navPoints[next].position, navPoints[next].position, xCofb);
								bpb = Vector3.Lerp(navPoints[a].forward * GetPointScale(a).z + navPoints[a].position, -navPoints[next].forward * GetPointScale(next).z + navPoints[next].position, xCofb);
								dpb = Vector3.Lerp(apb,bpb, xCofb);
								epb = Vector3.Lerp(bpb,cpb, xCofb);
								fpb = Vector3.Lerp(dpb,epb, xCofb);
								Vector3 leftRightb= GetBinormal(GetTangent(dpb,epb), navPoints[a].up, navPoints[next].up, xCofb);
							for(int cl= 0; cl < borderCurve.length; cl++){ //the points loop 0 1 2 3 0 1 2 3...
								
								if(borderCount*borderCurve.length + cl < rightBorderVertices.Length){
									rightBorderVertices[borderCount*borderCurve.length + cl] = fpb + leftRightb * (deltaWidth/2)
									* Mathf.Lerp(float.Parse( "" + GetPointScale(a).x),float.Parse( "" + GetPointScale(next).x), float.Parse( "" + bord)/float.Parse( "" + subdivision))
									+ leftRightb * rBorderNavPoints[cl].x
									+ GetNormal(leftRightb, GetTangent(dpb,epb)) * rBorderNavPoints[cl].y - transform.position;
									
									leftBorderVertices[borderCount*borderCurve.length + cl] = fpb - leftRightb * (deltaWidth/2)
									* Mathf.Lerp(float.Parse( "" + GetPointScale(a).x),float.Parse( "" + GetPointScale(next).x), float.Parse( "" + bord)/float.Parse( "" + subdivision))
									- leftRightb * rBorderNavPoints[cl].x
									+ GetNormal(leftRightb, GetTangent(dpb,epb)) * rBorderNavPoints[cl].y - transform.position;
								}
							}
							
							borderCount++;
						}
					//assign main vertices
					if(a < navPoints.Length - 1){
						newVertices[a * subdivision *2] = navPoints[a].position + navPoints[a].right * -(deltaWidth/2 * GetPointScale(a).x) - transform.position;
						newVertices[a * subdivision *2 + 1] = navPoints[a].position + navPoints[a].right * (deltaWidth/2 * GetPointScale(a).x) - transform.position;
					}else{
						newVertices[newVertices.Length - 2] = navPoints[navPoints.Length - 1].position + navPoints[navPoints.Length - 1].right * -(deltaWidth/2 * GetPointScale(navPoints.Length - 1).x) - transform.position;
						newVertices[newVertices.Length - 1] = navPoints[navPoints.Length - 1].position + navPoints[navPoints.Length - 1].right * (deltaWidth/2 * GetPointScale(navPoints.Length - 1).x) - transform.position;
					}
				}else
					FindNavPoints();
			}//a:end
	}
	
	private Vector3 GetPointScale (int index) {
		if(index < navPoints.Length){
			ERNavPoint npc = navPoints[index].GetComponent<ERNavPoint>();
			Vector3 npls = navPoints[index].localScale;
			if(npc){
				if(npc.lockSize){
					return new Vector3((npls.x*deltaWidth - deltaWidth + npc.lockedWidth)/deltaWidth, npls.y, npls.z);
				}
			}
			return npls;
		}
		return new Vector3(1f,1f,1f);
	}

	private void  SetTriangles (){
		int qCount= (navPoints.Length - 1) * subdivision + 1;
		if(navPoints.Length > 1) //if there is room for triangles to be drawn{
			newTriangles = new int[qCount * 6];
		else
			newTriangles = new int[0];
		
		for(int quad= 1; quad < qCount; quad++){
			//matrixCount = 0;
			for(int s2= 0; s2 < 6; s2++){
				//assign numbers
				newTriangles[(quad-1)*6 + s2] = quadMatrix[s2] + ((quad*2) - 2);
			}
		}
		
		//BORDER
		qCount = ((navPoints.Length - 1)*subdivision) * borderCurve.length;//*borderCurve.length;
		if(navPoints.Length > 1){ //if there is room for triangles to be drawn{
			if(enableMeshBorders == 1){
				rightTriangles = new int[qCount*6];
				leftTriangles = new int[qCount*6];
			}
		}else{
			rightTriangles = new int[0];
			leftTriangles = new int[0];
		}
		
		triCount = 0; //re use variable
		for(int ll= 0; ll < (navPoints.Length - 1)*subdivision; ll++) //Length index (horizontal index) X
			for(int lb= 0; lb < borderCurve.length - 1; lb++){//vertical index Y quad
				for(int bct= 0; bct < 6; bct++){ //under each quad - for matrix (assign tri points to curve points)
				int[] borderQuadMatrix = {1,borderCurve.length+1,0,borderCurve.length,0,borderCurve.length+1};
												  //1,3,0,2,0,3
				int[] borderQuadMatrixLeft = {0,borderCurve.length+1,1,borderCurve.length+1,0,borderCurve.length};
					if(triCount < rightTriangles.Length){
						rightTriangles[triCount] = borderQuadMatrix[bct]+lb+borderCurve.length*ll;
						leftTriangles[triCount] = borderQuadMatrixLeft[bct]+lb+borderCurve.length*ll;
						triCount++;
					}
				}
		}
	}

	private void  SetUVs (){
		int uvs_y_array = newVertices.Length/2;
		newUV = new Vector2[newVertices.Length];
		
		//get point-to-point distance and mesh Length
		float previousDistance = 0;
		float[] ptpDistance = new float[navPointsBeta_p.Length];
		for(int ptp= 0; ptp < ptpDistance.Length - 1; ptp++){
			ptpDistance[ptp] = Vector3.Distance(navPointsBeta_p[ptp],navPointsBeta_p[ptp + 1]);
		}
		
		switch (uvSet){
			case 0: //per segment
				uvSetCount = 0;
				for(int uvy= 0; uvy < uvs_y_array; uvy++){
					for(int uvx= 0; uvx < 2; uvx++){
						newUV[uvSetCount] = new Vector2(uvx * uvScale,uvy * uvScale);
						
						uvSetCount++;
					}
				}
			break;
			case 1: //top projection
				for(int uvp= 0; uvp < newUV.Length; uvp++){
					newUV[uvp] = new Vector2(newVertices[uvp].x * uvScale, newVertices[uvp].z * uvScale);
				}
			break;
			case 2: //width-to-Length (match width)
				
				uvSetCount = 0;
				previousDistance = 0;
				
				for(int uvny= 0; uvny < ptpDistance.Length && uvny < newUV.Length; uvny++){
					for(int uvnx= 0; uvnx < 2; uvnx++){
						newUV[uvSetCount] = new Vector2(uvnx * uvScale, previousDistance * uvScale / deltaWidth);
						uvSetCount++;
					}
					previousDistance += ptpDistance[uvny];
				}
				//fix last segment uvs
				if(newUV.Length >= 4 && navPointsBeta_p.Length > 0){
					float lastPointDistance = Vector3.Distance(navPointsBeta_p[navPointsBeta_p.Length-1],navPoints[navPoints.Length-1].position);
					newUV[newUV.Length-1] = newUV[newUV.Length-3] + new Vector2(0,1) * lastPointDistance * uvScale /deltaWidth;
					newUV[newUV.Length-2] = newUV[newUV.Length-4] + new Vector2(0,1) * lastPointDistance * uvScale /deltaWidth;
					
				}
				
			break;
			case 3: //stretch single texture
				for(int uvny= 0; uvny < navPointsBeta_p.Length + 1; uvny++){
					for(int uvnx= 0; uvnx < 2; uvnx++){
						int uvIndex = uvnx + uvny * 2;
						newUV[uvIndex] = new Vector2(uvnx * uvScale, 1f/navPointsBeta_p.Length * uvny * uvScale);
					}
				}
			break;
		}
	}

	private void  SetBorderUVs (){
		int uvs_y_array = newVertices.Length/2;
		rightUV = new Vector2[rightBorderVertices.Length];
		leftUV = new Vector2[rightBorderVertices.Length];
		
		//get point-to-point distance and mesh Length
		float previousDistance = 0f;
		float[] ptpDistance = new float[navPointsBeta_p.Length];
		for(int ptp= 0; ptp < ptpDistance.Length - 1; ptp++){
			ptpDistance[ptp] = Vector3.Distance(navPointsBeta_p[ptp],navPointsBeta_p[ptp + 1]);
		}
		
		switch (borderUvSet){
			case 0: //straight unwrap
				uvSetCount = 0;
				previousDistance = 0;
				
				for(int uvny= 0; uvny < ptpDistance.Length && uvny < rightUV.Length; uvny++){
					for(int uvnx= 0; uvnx < borderCurve.length; uvnx++){
						float keyDst = Mathf.Sqrt(Mathf.Pow(borderCurve.keys[uvnx].time,2) + Mathf.Pow(borderCurve.keys[uvnx].value,2)); //the distance between the vertices based on key time and value
						rightUV[uvSetCount] = new Vector2(keyDst * uvScale, previousDistance * uvScale);
						leftUV[uvSetCount] = new Vector2(keyDst * uvScale, previousDistance * uvScale);
						uvSetCount++;
					}
					previousDistance += ptpDistance[uvny];
				}
				//fix last segment uvs
				if(rightUV.Length >= borderCurve.length && leftUV.Length >= borderCurve.length && navPointsBeta_p.Length > 0){
					float lastPointDistance = Vector3.Distance(navPointsBeta_p[navPointsBeta_p.Length-1],navPoints[navPoints.Length-1].position);
					for(int uvnx1= 0; uvnx1 < borderCurve.length && uvnx1 < rightUV.Length; uvnx1++){
						rightUV[rightUV.Length-uvnx1-1] = rightUV[rightUV.Length-uvnx1-borderCurve.length-1] + new Vector2(0,1) * lastPointDistance * uvScale;
						leftUV[rightUV.Length-uvnx1-1] = leftUV[rightUV.Length-uvnx1-borderCurve.length-1] + new Vector2(0,1) * lastPointDistance * uvScale;
					}
				}
			break;
			case 1:
				for(int uvpb= 0; uvpb < rightUV.Length && uvpb < leftUV.Length; uvpb++){
					rightUV[uvpb] = new Vector2(rightBorderVertices[uvpb].x * uvScale, rightBorderVertices[uvpb].z * uvScale);
					leftUV[uvpb] = new Vector2(leftBorderVertices[uvpb].x * uvScale, leftBorderVertices[uvpb].z * uvScale);
				}
			break;
		}
	}

	public void  GroundPoints ( float offset  ){
		RaycastHit hit;
		Vector3[] pointPos = new Vector3[navPoints.Length]; //temporary variable used to store the position of the points to be used to cast a ray from there
		//save the position data
		for(int p= 0; p < pointPos.Length; p++){
			pointPos[p] = navPoints[p].position;
		}
		
		for(int vg= 0; vg < navPoints.Length; vg++){
			if(Physics.Raycast(pointPos[vg], Vector3.down, out hit)){
				navPoints[vg].position = hit.point + hit.normal*offset;
				Quaternion normalQuaternion = Quaternion.FromToRotation (Vector3.up, hit.normal);
				if(navPoints[vg].GetComponent<ERPointSnap>()){ //Snap Points Check
						if(!navPoints[vg].GetComponent<ERPointSnap>().snapped)
							navPoints[vg].eulerAngles = new Vector3(normalQuaternion.eulerAngles.x,navPoints[vg].eulerAngles.y,normalQuaternion.eulerAngles.z);
				}else{
					navPoints[vg].eulerAngles = new Vector3(normalQuaternion.eulerAngles.x,navPoints[vg].eulerAngles.y,normalQuaternion.eulerAngles.z);
				} 
			}
		}
		
		UpdateData();
	}

	public void  ResetMesh (){
		//if(EditorUtility.DisplayDialog("Reset?", "Are you sure you want to clear the current mesh and delete all Nav Points?", "No", "Yes"))
			//return;
		
		for(int nav= 0; nav < navPoints.Length; nav++){
			if(navPoints[nav] != null){
				DestroyImmediate(navPoints[nav].gameObject);
			}
		}
		
		navPoints = new Transform[0];
		newVertices = new Vector3[0];
		newUV = new Vector2[0];
		newTriangles = new int[0];
		
		CreateNavPoint();
		GenerateMesh();
	}

	private void  EnableBorders ( bool state  ){
		#if UNITY_EDITOR
		if(state == true){
			if(!FindLeftBorder()){
				leftBorderMeshObj = new GameObject();
				leftBorderMeshObj.name = LEFT_BORDER_NAME;
				leftBorderMeshObj.transform.position = transform.position;
				leftBorderMeshObj.transform.eulerAngles = Vector3.zero;
				leftBorderMeshObj.AddComponent<MeshFilter>();
				leftBorderMeshObj.AddComponent<MeshRenderer>();
				leftBorderMeshObj.transform.parent = this.transform;
			}
			if(!FindRightBorder()){
				rightBorderMeshObj = new GameObject();
				rightBorderMeshObj.name = RIGHT_BORDER_NAME;
				rightBorderMeshObj.transform.position = transform.position;
				rightBorderMeshObj.transform.eulerAngles = Vector3.zero;
				rightBorderMeshObj.AddComponent<MeshFilter>();
				rightBorderMeshObj.AddComponent<MeshRenderer>();
				rightBorderMeshObj.transform.parent = this.transform;
			}
			
			
			Mesh lMesh = new Mesh();
			Mesh rMesh = new Mesh();
			
			leftBorderMeshObj.GetComponent<MeshFilter>().sharedMesh = lMesh;
			lMesh.vertices = leftBorderVertices;
			lMesh.uv = leftUV;
			lMesh.triangles = leftTriangles;
			lMesh.RecalculateNormals();
			
			rightBorderMeshObj.GetComponent<MeshFilter>().sharedMesh = rMesh;
			rMesh.vertices = rightBorderVertices;
			rMesh.uv = rightUV;
			rMesh.triangles = rightTriangles;
			rMesh.RecalculateNormals();
			
		}else{
			if(FindRightBorder())
				Undo.DestroyObjectImmediate(rightBorderMeshObj);
			rightBorderMeshObj = null;
			if(FindLeftBorder())
				Undo.DestroyObjectImmediate(leftBorderMeshObj);
			leftBorderMeshObj = null;
		}
		#endif
	}
	
	public bool FindLeftBorder () {
		if(leftBorderMeshObj != null)
			return true;
		
		Transform t = transform.Find(ERMeshGen.LEFT_BORDER_NAME);
		if(t != null){
			leftBorderMeshObj = (GameObject) t.gameObject;
			return true;
		}
		
		//return false if border has not been found
		return false;
	}
	
	public bool FindRightBorder () {
		if(rightBorderMeshObj != null)
			return true;
		
		Transform t = transform.Find(ERMeshGen.RIGHT_BORDER_NAME);
		if(t != null){
			rightBorderMeshObj = (GameObject) t.gameObject;
			return true;
		}
		
		//return false if border has not been found
		return false;
	}
	
	public void  ReparentPoints ( bool reparent  ){
		 for(int p= 0; p < navPoints.Length; p++){
			if(reparent){
				if(p > 0){
					navPoints[p].parent = navPoints[p-1];
				}
			}else{
				navPoints[p].parent = transform;
			}
		}
		lastParentPointsUpd = parentPoints;
	}

	private void  UpdateCollider ( Mesh colMesh  ){
		#if UNITY_EDITOR
		if(includeCollider == 1){
			if(!GetComponent<MeshCollider>())
				gameObject.AddComponent<MeshCollider>();
		
			GetComponent<MeshCollider>().sharedMesh = colMesh; //assign the updated mesh to the collider;
			
			if(enableMeshBorders == 1){
				if(!rightBorderMeshObj)
					EnableBorders(true);
				
				if(!rightBorderMeshObj.GetComponent<MeshCollider>())
					rightBorderMeshObj.AddComponent<MeshCollider>();
				if(!leftBorderMeshObj.GetComponent<MeshCollider>())
					leftBorderMeshObj.AddComponent<MeshCollider>();
				
				if(rightBorderMeshObj.GetComponent<MeshCollider>()){
					rightBorderMeshObj.GetComponent<MeshCollider>().sharedMesh.Clear();
					leftBorderMeshObj.GetComponent<MeshCollider>().sharedMesh.Clear();
				
					rightBorderMeshObj.GetComponent<MeshCollider>().sharedMesh = rightBorderMeshObj.GetComponent<MeshFilter>().sharedMesh; //assign the updated mesh to the collider;
					leftBorderMeshObj.GetComponent<MeshCollider>().sharedMesh = leftBorderMeshObj.GetComponent<MeshFilter>().sharedMesh; //assign the updated mesh to the collider;
				}
			}
		}else{
			if(GetComponent<MeshCollider>())
				Undo.DestroyObjectImmediate(GetComponent<MeshCollider>());
			if(FindRightBorder())
			if(rightBorderMeshObj.GetComponent<MeshCollider>())
				Undo.DestroyObjectImmediate(rightBorderMeshObj.GetComponent<MeshCollider>());
			if(FindLeftBorder())
			if(leftBorderMeshObj.GetComponent<MeshCollider>())
				Undo.DestroyObjectImmediate(leftBorderMeshObj.GetComponent<MeshCollider>());
		}
		#endif
	}
	
	public void FindNavPoints () {
		bool foundAllNavPoints = false;
		int navPointCounter = 0;
		string navPointNames = NAV_POINT_NAMES + " ";
		Transform[] _navPoints = new Transform[0];
		
		while(!foundAllNavPoints){
			Transform point = transform.Find(navPointNames + navPointCounter);
			if(point != null){
				_navPoints = AddTransformArray(_navPoints, point);
				if(point.GetComponent<ERNavPoint>())
					point.GetComponent<ERNavPoint>().assignedMeshGen = (ERMeshGen)this;
				
				navPointCounter++;
			}
			else
				foundAllNavPoints = true;
		}
		
		navPoints = _navPoints;
	}

	public void  Finalise (){
		#if UNITY_EDITOR
		if(!EditorUtility.DisplayDialog("Finalise", "This action will delete all Nav Points and remove the ERMeshGen.cs script from the current object.\nThis action is irreversible!", "Continue", "Cancel"))
			return;
		
		for(int nav= 0; nav < navPoints.Length; nav++){
			if(navPoints[nav] != null){
				Undo.DestroyObjectImmediate(navPoints[nav].gameObject);
			}
		}
		
		Undo.DestroyObjectImmediate(this);
		#endif
	}

	private Vector3  GetTangent ( Vector3 d ,   Vector3 e  ){ //get the tangent of the subdivision curve
		return (d - e)/Vector3.Distance(d,e);
	}

	private Vector3  GetBinormal ( Vector3 tng ,   Vector3 upVectorA ,   Vector3 upVectorB ,   float cof  ){ //get the normal (y dir) of the subdivision curve
		Vector3 binormal = Vector3.Cross(Vector3.Lerp(upVectorA, upVectorB, cof), tng).normalized;
		return binormal; //Mathf.Cross(tng, binormal);
	}

	private Vector3  GetNormal ( Vector3 bnrm ,   Vector3 tng  ){
		return Vector3.Cross(tng, bnrm);
	}
	
	public static Transform[] AddTransformArray (Transform[] _arr, Transform tr) {
		Transform[] arr = new Transform[_arr.Length + 1];
		for (int i = 0; i < arr.Length; i++){
			if(i < _arr.Length)
				arr[i] = _arr[i];
			else
				arr[i] = tr;
		}
		return arr;
	}
	
	public static Transform[] RemoveTransformArray (Transform[] _arr, int n) {
		if(n > _arr.Length){
			Debug.Log("Elements to remove exceeded the array size! (array size: " + _arr.Length + ", tried to remove: " + n + ")");
			return new Transform[0];
		}
		
		Transform[] arr = new Transform[_arr.Length - n];
		for (int i = 0; i < arr.Length; i++)
			arr[i] = _arr[i];
		
		return arr;
	}
	
	public static Transform[] CombineTransformArray (Transform[] _arr1, Transform[] _arr2) {
		Transform[] arr = new Transform[_arr1.Length + _arr2.Length];
		for (int i = 0; i < arr.Length; i++){
			if(i < _arr1.Length)
				arr[i] = _arr1[i];
			else
				arr[i] = _arr2[i];
		}
		return arr;
	}
}
