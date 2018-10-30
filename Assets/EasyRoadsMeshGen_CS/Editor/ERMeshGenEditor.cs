using UnityEditor;
using UnityEngine;
using System.Collections;

[CustomEditor(typeof(ERMeshGen))]
public class ERMeshGenEditor : Editor {
	private string[] curveControlStateStrings = {"Full Manual", "Automatic Rotation"};
    private string[] constantVertsUpdateStr = {"Automatic (Recommended)", "Manual Update (Best Performance)", "Update Vertices Only (OK Performance)", "Realtime (Low Performance)"};
	private string[] parentPointsStr = {"Off", "On"};
	private string[] includeColliderStr = {"Off", "On"};
	private string[] enableHelpStr = {"Off", "On"};
	private string[] uvSetStr = {"Per Segment", "Top Projection", "Match Width", "Stretch Single Texture"};
	private string[] enableMeshBordersStr = {"Off", "On"};
	private string[] borderUvSetStr = {"Straight Unwrap", "Top Projection"};
	private bool tutorialMode = false;
	private int tutorialProgress = 0;
	
	static bool pendingDuplication = false;
	
	ERMeshGen meshGen;
	SerializedObject meshGen_;
	SerializedProperty meshGen_deltaWidth;
	SerializedProperty meshGen_subdivision;
	SerializedProperty meshGen_uvScale;
	SerializedProperty meshGen_groundOffset;
	SerializedProperty meshGen_curveControlState;
	SerializedProperty meshGen_vertsUpdate;
	SerializedProperty meshGen_parentPoints;
	SerializedProperty meshGen_includeCollider;
	SerializedProperty meshGen_enableHelp;
	SerializedProperty meshGen_uvSet;
	SerializedProperty meshGen_enableMeshBorders;
	SerializedProperty meshGen_borderCurve;
	SerializedProperty meshGen_borderUvSet;
	
	public void OnEnable () {
		Init();
	}
	
	public void Init () {
		
		meshGen = (ERMeshGen)target; 
		meshGen_ = new SerializedObject(meshGen);
		meshGen_deltaWidth = meshGen_.FindProperty("deltaWidth");
		meshGen_subdivision = meshGen_.FindProperty("subdivision");
		meshGen_uvScale = meshGen_.FindProperty("uvScale");
		meshGen_groundOffset = meshGen_.FindProperty("groundOffset");
		meshGen_curveControlState = meshGen_.FindProperty("curveControlState");
		meshGen_vertsUpdate = meshGen_.FindProperty("updatePointsMode");
		meshGen_parentPoints = meshGen_.FindProperty("parentPoints");
		meshGen_includeCollider = meshGen_.FindProperty("includeCollider");
		meshGen_enableHelp = meshGen_.FindProperty("enableHelp");
		meshGen_uvSet = meshGen_.FindProperty("uvSet");
		meshGen_enableMeshBorders = meshGen_.FindProperty("enableMeshBorders");
		meshGen_borderCurve = meshGen_.FindProperty("borderCurve");
		meshGen_borderUvSet = meshGen_.FindProperty("borderUvSet");
		
		DuplicationHandler();
	}
	
	public void OnSceneGUI () {
		Event e = Event.current;
			if(e != null)
				if (e.commandName == "Duplicate")
					pendingDuplication = true;
		DuplicationHandler();
	}
	
	void DuplicationHandler () {
							
		if(meshGen){
			//update ER
			if(ERMeshGen.lastSelectedMeshGen){
				if(meshGen.gameObject.GetInstanceID() != ERMeshGen.lastSelectedMeshGen.gameObject.GetInstanceID()){
					if(pendingDuplication){
						pendingDuplication = false;
						meshGen.OnDuplicationEvent();
					}
					ERMeshGen.lastSelectedMeshGen.OnDuplicationEventSrc();
					ERMeshGen.lastSelectedMeshGen = meshGen;
				}
			}else{
				ERMeshGen.lastSelectedMeshGen = meshGen;
			}
		}
	}
	
	public override void OnInspectorGUI() {
		if(Application.isPlaying)
			return;
		
		DuplicationHandler();

		if (!EditorPrefs.HasKey("ERMGTUT"))
			tutorialMode = false;
		else{
			tutorialMode = true;
			tutorialProgress = EditorPrefs.GetInt("ERMGTUT");
		}
			
		if(!tutorialMode){
			
		//if(meshGen.GetComponent<MeshFilter>())
		//	DrawDefaultInspector();
        
		if(meshGen.parentPoints == 1){
			meshGen_curveControlState.intValue = 0;
		}
		
		if(meshGen.GetComponent<MeshFilter>()){
			meshGen_deltaWidth.floatValue = EditorGUILayout.FloatField("Delta Width", meshGen.deltaWidth);
			meshGen_subdivision.intValue = EditorGUILayout.IntField("Subdivision", meshGen.subdivision);
			meshGen_uvScale.floatValue = EditorGUILayout.FloatField("UV Scale", meshGen.uvScale);
			GUI.color = new Color(meshGen.updatePointsMode/2f,1,0.3f);
			meshGen_vertsUpdate.intValue = EditorGUILayout.Popup("Update Mode", meshGen.updatePointsMode, constantVertsUpdateStr);
			GUI.color = Color.white;
			meshGen_parentPoints.intValue = EditorGUILayout.Popup("Parent Points", meshGen.parentPoints, parentPointsStr);
			if(meshGen.parentPoints != 1)
				meshGen_curveControlState.intValue = EditorGUILayout.Popup("Point Control", meshGen.curveControlState, curveControlStateStrings);
			meshGen_includeCollider.intValue = EditorGUILayout.Popup("Include Collider", meshGen.includeCollider, includeColliderStr);
			meshGen_uvSet.intValue = EditorGUILayout.Popup("UVs", meshGen.uvSet, uvSetStr);
			
			GUILayout.FlexibleSpace();
			meshGen_enableMeshBorders.intValue = EditorGUILayout.Popup("Borders Mesh", meshGen.enableMeshBorders, enableMeshBordersStr);
			
			if(meshGen.enableMeshBorders == 1){
				EditorGUI.indentLevel++;
				//if(GUILayout.Button("Configure Border"))
				//	enableWindow = true;
				meshGen_borderCurve.animationCurveValue = EditorGUILayout.CurveField("Border Shape", meshGen.borderCurve);
				meshGen_borderUvSet.intValue = EditorGUILayout.Popup("Border UVs", meshGen.borderUvSet, borderUvSetStr);
				EditorGUI.indentLevel--;
			}
			
			GUI.color = new Color(0.3f,1,0.1f);
			GUILayout.BeginHorizontal();
			if(GUILayout.Button("Add Nav Point"))
			{
				meshGen.CreateNavPoint();
			}
			GUI.color = new Color(1,1,0.1f);
			if(meshGen.navPoints.Length > 1)
				if(GUILayout.Button("Delete Nav Point")){
					meshGen.DeleteNavPoint();
				}
			GUILayout.EndHorizontal();
			GUI.color = new Color(0.3f,1,0.1f);
			if(meshGen.updatePointsMode > 0 && meshGen.updatePointsMode != 3)
				if(GUILayout.Button("Apply Changes"))
				{
					meshGen.UpdateData();
					meshGen.UpdateData(); //Temporary fix for the Manual Apply issue;
				}
				
			GUI.color = new Color(0.6f,0.6f,1);
			GUILayout.Box("Ground nav points to surface underneath.\nOffset: " + meshGen.groundOffset, EditorStyles.helpBox, GUILayout.MaxWidth(999));
			GUILayout.BeginHorizontal();
			
			meshGen_groundOffset.floatValue = GUILayout.HorizontalSlider(meshGen.groundOffset,0,1);
			
			GUILayout.EndHorizontal();
			
			if(GUILayout.Button("Ground Points")){
				meshGen.GroundPoints(meshGen.groundOffset);
			}
			
			
			GUI.color = new Color(1,1,0.3f);
			if(meshGen.enableHelp != 0)
				GUILayout.Box("Delete the Nav Points and remove the mesh generation script.", EditorStyles.helpBox);
			if(GUILayout.Button("Finalise"))
				meshGen.Finalise();
			
			GUI.color = new Color(1,0.2f,0.2f);
			if(GUILayout.Button("Reset"))
			{
				if(!EditorUtility.DisplayDialog("Reset?", "Are you sure you want to clear the current mesh and delete all Nav Points?", "No", "Yes"))
					meshGen.ResetMesh();
			}
		}
		else{
			GUI.color = new Color(0,1,0);
			GUILayout.Box("To start using the tool straight away select 'Begin'. If you are using the tool for the first time, select 'Tutorial'");
			GUI.color = new Color(1,1,1);
			if(GUILayout.Button("Tutorial")){
				tutorialMode = true;
				tutorialProgress = 0;
				EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
			}
			
			GUI.color = new Color(0,1,0);
			if(GUILayout.Button("Begin"))
			{
				meshGen.GenerateFirstMesh();
			}
		}
		
		}
		else{//TUTORIAL
			GUI.color = new Color(1,1,0);
			if(GUILayout.Button("Skip Tutorial >>")){
				tutorialMode = false;
				tutorialProgress = 0;
				EditorPrefs.DeleteKey("ERMGTUT");
				meshGen.GenerateFirstMesh();
			}
			GUI.color = Color.white;
			
			GUILayout.Label("Tutorial progress: " + tutorialProgress);
			
			switch (tutorialProgress){
				case 0:
					GUILayout.Box("Welcome to the built-in Easy Roads Mesh Gen tutorial. This tutorial will teach you the basics of how to use the tool and what the various elements do.");
					GUILayout.Box("Variables marked with * will preserve their value even after completing the tutorial.");
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 1:
					GUILayout.Box("The Delta Width determines how wide the generated mesh is.");
					
					GUILayout.BeginHorizontal();
					GUILayout.Label("Delta Width*");
					meshGen.deltaWidth = float.Parse(GUILayout.TextField("" + meshGen.deltaWidth));
					GUILayout.EndHorizontal();
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 2:
					GUILayout.Box("Subdivision adds extra vertices inside each segment between the Nav Points. This allows for smoother looking roads and meshes.");
					
					GUILayout.BeginHorizontal();
					GUILayout.Label("Subdivision*");
					meshGen.subdivision = int.Parse(GUILayout.TextField("" + meshGen.subdivision));
					GUILayout.EndHorizontal();
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 3:
					GUILayout.Box("UV Scale determines the scaling of the textures.");
					
					GUILayout.BeginHorizontal();
					GUILayout.Label("Uv Scale*");
					meshGen.uvScale = float.Parse(GUILayout.TextField("" + meshGen.uvScale));
					GUILayout.EndHorizontal();
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 4:
					GUILayout.Box("The Nav Points array contains the links to the Nav Points (objects). This variable is rarely assigned manually but it's left public to help in case some of the links are lost or need to be assigned by hand.");
					
					GUILayout.Label("► Nav Points");
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;

				case 5:
					GUILayout.Box("Similar to the 'Nav Points' variable, the 'Left Border Mesh' and 'Right Border Mesh' variables contain the links to the generated border meshes. They are not to be accessed directly but are left public in case they need to be assigned manually.");
					GUILayout.Label("Left Border Mesh");
					GUILayout.Label("Right Border Mesh");
					
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 6:
					GUILayout.Box("'Update Mode' controls how often the tool updates it's mesh and vertices. This is useful for increasing performance in the scene view. 'Manual Update' has little to no impact on the performance since the tool is updated only when you hit 'Apply Changes'. 'Vertices Only' visualises the position of the vertices but still requires to manually apply the changes. 'Realtime' is constantly updating the generated Mesh which makes it easier to work with, but is good to leave it off when you are done.");
					EditorGUILayout.Popup("Update Mode", meshGen.updatePointsMode, constantVertsUpdateStr);
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
					
				case 7:
					GUILayout.Box("'Parent Points' makes every next Nav Point a child of the previous. This option does not work with 'Automatic Rotation'. It is recommended that to keep that option turned off.");
					meshGen.parentPoints = EditorGUILayout.Popup("Parent Points*", meshGen.parentPoints, parentPointsStr);
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;	
				
				case 8:
					GUILayout.Box("You can pick how the UVs are generated by changing the 'UVs' option.");
					meshGen.uvSet = EditorGUILayout.Popup("UVs*", meshGen.uvSet, uvSetStr);
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
				
				case 9:
					GUILayout.Box("'Include Collider' generates a mesh collider.");
					meshGen.includeCollider = EditorGUILayout.Popup("Include Collider*", meshGen.includeCollider, includeColliderStr);
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
				
				case 10:
					GUILayout.Box("'Point Control' determines how much control you have over the Nav Points which is meant to speed up your workflow. When 'Automatic Rotation' is selected the tool automatically rotates the Nav Points, allowing you to quicky position points without having to worry about their orientation. 'Full Manual' on the other hand gives you full control over the orientation and position of the Nav Points, which is useful when you are using subdivision and need to fine-tune the bezier curve for each segment.");
					meshGen.curveControlState = EditorGUILayout.Popup("Point Control*", meshGen.curveControlState, curveControlStateStrings);
					GUI.color = Color.green;
					if(GUILayout.Button("Next >")){
						tutorialProgress++;
						EditorPrefs.SetInt("ERMGTUT",tutorialProgress);
					}
					break;
				case 11:
					GUILayout.Box("'Ground Points' snaps the Nav Points to the ground.");
					GUI.color = new Color(0.6f,0.6f,1);
					GUILayout.Box("Offset*: " + meshGen.groundOffset, GUILayout.MaxWidth(999));
					GUILayout.BeginHorizontal();
					
					meshGen.groundOffset = GUILayout.HorizontalSlider(meshGen.groundOffset,0,1);
					
					GUILayout.EndHorizontal();
					
					GUILayout.Button("Ground Points");
					
					GUI.color = Color.green;
					if(GUILayout.Button("Finish Tutorial >>")){
						tutorialMode = false;
						tutorialProgress = 0;
						EditorPrefs.DeleteKey("ERMGTUT");
						meshGen.GenerateFirstMesh();
					}
					break;
				
				default:
					GUILayout.Label("-This is a basic tutorial.-");
					break;
			}
		}

		GUI.color = new Color(1,1,1);
		meshGen_enableHelp.intValue = EditorGUILayout.Popup("Enable Tips", meshGen.enableHelp, enableHelpStr);
		if(meshGen.enableHelp != 0){
			GUI.color = new Color(1,1,0.7f);
			GUILayout.Box("Important: Object rotation should be (0,0,0) at all times!", EditorStyles.helpBox);
			GUI.color = new Color(1,0.2f,0.2f);
			if(meshGen.parentPoints == 1)
			GUILayout.Box("Important: Automatic Point Control is not allowed while Parent Points is on.", EditorStyles.helpBox);
			GUI.color = new Color(1,1,1);
			GUILayout.Box("Report any problems on e-mail: hropenmail01@abv.bg", EditorStyles.helpBox);
		}
		
		meshGen_.ApplyModifiedProperties();
	}
}
