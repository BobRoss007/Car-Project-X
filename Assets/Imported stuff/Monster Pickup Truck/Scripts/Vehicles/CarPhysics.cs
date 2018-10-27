using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarPhysics : MonoBehaviour {

	[System.Serializable]
	public class WheelSoundData
	{
		//circumference percentage per minute -> (circumference / deltaTime) * 60;
		///<summary>The wheel revolutions per minute.</summary>
		public float wheelRPM { get; private set; }

		//rHit.point but regenerated from (pivotPos + pivotDown * rHit.distance) to correctly interpolate;
		///<summary>The surface contact point of the wheel in world space.</summary>
		public Vector3 wheelContactPoint { get; private set; }

		//spin amount percentage (0 is stopped, 1 is rolling, 2 is spinning)
		/// <summary>Wheel rotation slip. 0 is full lock, 1 is rolling and 2 is spinning.</summary>
		public float longitudinalSlip { get; private set; }

		//slip angle percentage towards 90 ((angle - tPeak) / (90 - tPeak));
		/// <summary>Percentage from the peak angle to 90 degrees. Will be 0 when within the traction limit.</summary>
		public float lateralSlip { get; private set; }

		//brake percentage to full force (0,1) -> 0 if locked
		/// <summary>Percentage of the deceleration force. 0 is no force or locked, 1 is full force.</summary>
		public float brakeAmount { get; private set; }

		//damping force ((dist - dist) / deltaTime) * compDamp;
		/// <summary>The suspension change rate in metres. Does not take Rigidbody.mass into account.</summary>
		public float suspensionForce { get; private set; }

		//percentage of compression (extended = 1 -> upperLimit = 0) inverseLerp(upperLimit, maxDist, rHit.distance);
		/// <summary>Percentage of suspension distance. 0 is fully compressed, 1 is fully extended.</summary>
		public float suspensionLength { get; private set; }

		public void SetWheelRPM(float value) { wheelRPM = Mathf.Abs(value); }
		public void SetWheelContactPoint(Vector3 value) { wheelContactPoint = value; }
		public void SetWheelSlip(Vector2 value) { lateralSlip = value.x; longitudinalSlip = value.y; }
		public void SetWheelBraking(float value) { brakeAmount = value; }
		public void SetSuspensionForce(float value) { suspensionForce = value; }
		public void SetSuspensionLength(float value) { suspensionLength = value; }

	}

	[System.Serializable]
	public class SoundData
	{
		public WheelSoundData[] wheelSounds;

		//Max wheelspeed from all driving wheels. 0 is parked, 1 is top speed
		/// <summary>Percentage of velocity relative to current max speed. 0 is parked, 1 is full speed, >1 for rev limiter.</summary>
		public float engineRPM { get; private set; }

		//how much torque work is applied. Set to 1 while shifting gear.
		/// <summary>How much work the engine is doing. -2 for downhill engine braking, -1 for expected engine braking, 0 for minimal work, 1 for expected work, 2 for maximum work.</summary>
		public float engineLoad { get; private set; }

		//is it changing gear?
		/// <summary>True while the vehicle is changing gear.</summary>
		public bool isShifting { get; private set; }

		//how much is the clutch engaged?
		/// <summary>Between 0-1 when acceleration is attempted.</summary>
		public float isClutchEngaged { get; private set; }

		public void SetEngineRPM(float value) { engineRPM = value; }
		public void SetEngineLoad(float value) { engineLoad = value; }
		public void SetIsShifting(bool value) { isShifting = value; if (value) { engineRPM = 0; engineLoad = 0; } }
		public void SetIsClutchEngaged(float value) { isClutchEngaged = value; }
	}

	[System.Serializable]
	public class WheelData
	{
		[Tooltip("The pivot point for the wheel node.")]
		public Transform wheelPivot;
		[Tooltip("The raycast point for the wheel node. This can be the same as the pivot if no offset is desired.")]
		public Transform wheelCastPoint;
		[Tooltip("The radius of the wheel. Simulation velocities dont take this into account, so its main use is creating correct visuals.")]
		public float wheelRadius;

		[Tooltip("Toggles what side the wheel belongs to.")]
		public bool isLeft;
		[Tooltip("Toggles if it's a front or rear wheel.")]
		public bool isFront;
		[Tooltip("Toggles if the wheel can steer. Total number of these affects steering sensitivity.")]
		public bool canSteer;
		[Tooltip("Toggles if the wheel is 'passive'. Passive wheels can not steer and will not influence weight bias pre-calculation. Good for vehicles with a wheel in the centre of the wheelbase length.")]
		public bool isPassive;

		[HideInInspector]
		public int twin;

		[HideInInspector]
		public RaycastHit rHit;
		[HideInInspector]
		public float gripMult;
		[HideInInspector]
		public float springLerp02;
		[HideInInspector]
		public float slipAngle;
		[HideInInspector]
		public float camberAngle;

		[HideInInspector]
		public bool isGrounded;
		[HideInInspector]
		public float prevSuspDist;
		[HideInInspector]
		public Vector3[] lastWorldPoses;
		[HideInInspector]
		public Vector3 stopPosWorld;
		[HideInInspector]
		public Vector3 stopPosVector;
		[HideInInspector]
		public Vector3 posVelocityWorld;
		[HideInInspector]
		public float lastStopPosDist;
		[HideInInspector]
		public Vector3 forwardProjected;
		[HideInInspector]
		public Vector3 rightProjected;

		[HideInInspector]
		public Vector3 lastWorldPos;
		[HideInInspector]
		public Vector3 lastWorldDir;
		[HideInInspector]
		public float rotation;
		[HideInInspector]
		public float rpm;
		[HideInInspector]
		public float moveDistance;
		[HideInInspector]
		public bool isLocked;
		[HideInInspector]
		public float spinAmount;
		[HideInInspector]
		public float diffPower;

		////SOUND STUFF
		[HideInInspector]
		public float suspensionCompressionForce;
		[HideInInspector]
		public float wheelGroundVelocity;
		[HideInInspector]
		public Vector2 wheelSlide;
		[HideInInspector]
		public float wheelBrakeAmount;

		[HideInInspector]
		public float debugGripMult;
		[HideInInspector]
		public Vector3 debugSpringForce;
		[HideInInspector]
		public float debugDampingForce;
		[HideInInspector]
		public float debugRollbarForce;
	}

	[Header("---Inputs---")]
	public string steer = "Steering";
	public string forward = "Forward";
	public string reverse = "Reverse";
	public string handbrake = "Handbrake";
	//[Space]
	public bool isDriving = false;
	//public bool usedByPlayer = false;

	[Header("---Debugging")]
	public bool debugSuspension = false;
	public bool debugDifferentials = false;
	public bool debugFriction = false;
	public bool debugDriftAngle = false;
	public bool debugStopPoints = false;
	public bool debugWeight = false;
	public bool debugRaycast = false;
	public bool debugCollisions = false;
	[Space]
	public bool debugWheelSpeed = false;
	[Space]
	public bool debugEngineRPM = false;
	public bool debugAudioValues = false;

	[Header("---Physics Info---")]
	public WheelData[] wheels;
	public Collider chassisCollider;
	[Tooltip("Offset centre of mass, using wheelbase centre as origin.")]
	public Vector3 centreOfMass = Vector3.zero;
	[Tooltip("Multiplies rotational inertia. Higher values result in a heavier feeling vehicle.")]
	public Vector3 inertiaMod = Vector3.one;

	[Header("---Suspension---")]
	[Tooltip("How far the suspension can reach (Max raycast distance).")]
	public float sMaxDistance = 0.25f;
	[Tooltip("At what distance the suspension will stop compressing, starting at the wheel node.")]
	public float sUpperLimit = 0.1f;
	[Tooltip("How much the raycast origin will be moved up. This will not increase max distance, so it must be compensated manually.")]
	public float sVerticalOffset = 0.0f;
	[Tooltip("At what distance the suspension will come to a rest when the surface is level.")]
	public float sRestDistance = 0.1f;

	[Space]
	[Tooltip("Net maximum vertical G-force (rounded to 10ms^2) the spring will apply at full compression. Should be greater than 1.0 to ensure rest distance is reached.")]
	public float sSpring = 2.0f;
	[Tooltip("How resistant the suspension is to compress, proportional to the spring force for ease of tuning.")]
	public float sCompressionDamping = 1.0f;
	[Tooltip("How resistant the suspension is to extend, proportional to the spring force for ease of tuning.")]
	public float sReboundDamping = 1.5f;

	[Space]
	[Tooltip("Front rollbar strength, proportional to spring force. Increases cornering stability of front wheels but may sacrifice grip.")]
	public float sAntirollFront = 1.0f;
	[Tooltip("Rear rollbar strength, proportional to spring force. Increases cornering stability of rear wheels but may sacrifice grip.")]
	public float sAntirollRear = 1.0f;

	[Space]
	[Tooltip("Greater values will decrease total body roll when cornering and braking.")]
	public float sRollHeightFront = 0.3f;
	[Tooltip("Greater values will decrease total body roll when cornering and accelerating.")]
	public float sRollHeightRear = 0.4f;

	[Header("---Tyres---")]
	[Tooltip("Net tyre grip at rest, in G-force (rounded to 10ms^2).")]
	public float tMaxGrip = 1.0f;
	[Tooltip("Net tyre grip when the wheel is on its side, in G-force (rounded to 10ms^2).")]
	public float tCamberGrip = 0.1f;
	[Tooltip("Grip distribution between front and rear wheels. 0.5 will give equal grip to all wheels.")]
	[Range(0, 1)]
	public float tBiasFront = 0.495f;
	[Tooltip("How easily the vehicle will drift, decreasing when traveling at higher speeds and gears.")]
	[Range(0, 1)]
	public float tDriftAbility = 1.0f;

	[Tooltip("The travel angle where full grip force is applied. Higher angles will make the vehicle feel loose, possibly more forgiving. Recommended between 5.0 and 10.0")]
	public float tPeakAngle = 10.0f;
	[Tooltip("Controls low speed steering wiggle. Higher values feel as if the tyres are softer. Default is 0.1 TWEAK WITH CARE")]
	public float tSlipTrail = 0.1f;

	[Tooltip("How far the vehicle is allowed to move before desired stop position is recalculated. Default is 0.05 TWEAK WITH EXTRA CARE")]
	[Range(0.01f, 0.1f)]
	public float tStopPositionRadius = 0.05f;
	[Tooltip("The radius the vehicle will attempt to stay within when stationary. Default is 0.02 TWEAK WITH EXTRA CARE")]
	[Range(0.01f, 0.1f)]
	public float tStopSpringRadius = 0.02f;
	[Tooltip("How quickly the vehicle will come to a stationary rest. Lower values will introduce some wobble, looking like tyre flexing.")]
	public float tStopDamping = 10.0f;
	[Tooltip("Velocity-based damping used to reduce stop position 'orbiting'.")]
	[Range(0, 1)]
	public float tVelocityDamping = 0.1f;

	[Header("---Engine & Transmission---")]
	[Tooltip("Engine acceleration in G-force (rounded to 10ms^2), decreasing with speed and gears.")]
	public float mAcceleration = 1.0f;
	[Tooltip("Engine deceleration in G-force (rounded to 10ms^2) at no throttle input.")]
	public float mEngineDeceleration = 0.1f;
	[Tooltip("How much extra power can be applied if needed, maxing out at 2x the acceleration force.")]
	[Range(0, 1)]
	public float mTorqueAmount = 1.0f;
	[Tooltip("Power distribution between front and rear wheels. 0.5 sends equal power to all wheels.")]
	[Range(0, 1)]
	public float mBiasFront = 0.5f;
	[Space]
	[Tooltip("How power is distributed in front wheel pairs relative to grip. 0.0 powers least grippy more, 0.5 is equal distribution, 1.0 powers most grippy more.")]
	[Range(0, 1)]
	public float mDiffStiffnessFront = 0.0f;
	[Tooltip("How power is distributed in rear wheel pairs relative to grip. 0.0 powers least grippy more, 0.5 is equal distribution, 1.0 powers most grippy more.")]
	[Range(0, 1)]
	public float mDiffStiffnessRear = 0.0f;

	[Space]
	[Tooltip("Top speed going forward.")]
	public float mTopSpeedKmh = 150.0f;
	[Tooltip("Top speed going in reverse.")]
	public float mReverseKmh = 50.0f;

	[Space]
	[Tooltip("Number of gears the vehicle has. There will always only be a single reverse gear.")]
	public int mGears = 5;
	[Tooltip("Time in seconds to change gear.")]
	public float mShiftTime = 0.125f;
	[Tooltip("Percentage of how early in the current gear the vehicle will shift up.")]
	[Range(0, 1)]
	public float mGearUpshiftThreshold = 0.1f;
	[Tooltip("Percentage of how far to go into the previous gear before the vehicle will shift down. This is to prevent repeated shifting when driving uphill.")]
	[Range(0, 1)]
	public float mGearDownshiftThreshold = 0.2f;


	[Header("---Braking & Resistance---")]
	[Tooltip("Braking in G-force (rounded to 10ms^2).")]
	public float bDeceleration = 1.8f;
	[Tooltip("Braking distribution between front and rear wheels. 0.5 for equal force for all wheels.")]
	[Range(0, 1)]
	public float bBiasFront = 0.55f;

	[Header("---Controls---")]
	[Tooltip("Maximum steering angle by the inside wheel. Ackermann ratio is automatic. Decreases to peak angle at high speed.")]
	public float cSteerMax = 35.0f;
	[Tooltip("The steering speed, how quickly the wheels turn to the current input. Higher value is faster.")]
	public float cSteerSpeed = 8.0f;
	[Tooltip("How much the steering assist corrects travel angle difference.")]
	[Range(0, 1)]
	public float cSteerAngleCounter = 0.5f;
	[Tooltip("How much the steering assist corrects vehicle rotation.")]
	[Range(0, 1)]
	public float cSteerRotateCounter = 0.15f;
	[Tooltip("How much the steering decreases with speed. Recommended to keep close to 1.0")]
	[Range(0, 1)]
	public float cSteerAngleDecrease = 1.0f;
	[Tooltip("How much the front wheels can steer.")]
	[Range(0, 1)]
	public float cSteerFrontAmount = 1.0f;
	[Tooltip("How much the rear wheels can steer.")]
	[Range(0, 1)]
	public float cSteerRearAmount = 0.0f;
	[Tooltip("Toggles if rear wheels are allowed to use counter assists.")]
	public bool cRearWheelsCanCounter = true;
	[Tooltip("Toggles if the counter assists can use full steering lock, opposed to only within traction limit.")]
	public bool cFullCounterLock = false;
	[Tooltip("Toggles if steering assists should take action when reversing.")]
	public bool cCounterInReverse = false;
	[Tooltip("Toggles if steering should be inverted when reversing.")]
	public bool cInvertSteeringInReverse = false;
	[Space]
	[Tooltip("Toggles ABS (Anti-lock Braking System). Will rapidly pulse the brakes on and off if exceeding traction limit, opposed to leaving the wheels locked.")]
	public bool cHasABS = true;
	[Tooltip("Velocity margin in m/s where throttle input goes both directions. If no throttle is given the vehicle will automaticly brake.")]
	public float cFreeVelocity = 3.0f;

	[HideInInspector]
	public bool isBraking;
	[HideInInspector]
	public bool isReversing;
	[HideInInspector]
	public bool headlightsOn = false;

	Vector4 velocity;
	Rigidbody rBody;

	bool isGrounded;
	[HideInInspector]
	public Vector2 wheelbase = Vector2.zero;
	Vector3 wheelbaseCentre;
	Vector3 steerAngleLocationFront;
	Vector3 steerAngleLocationRear;
	float stationaryMult = 1;
	bool wantToMove = false;
	Vector3 lastChassisPos;
	Quaternion lastChassisRot;
	float counterAmount;
	float steerAngleFront = 0;
	float steerAngleRear = 0;
	float steerFactor = 0;
	float steerFactorReciprocated = 0;
	int frontWheels = 0;
	int rearWheels = 0;
	int passiveWheels = 0;
	int steeringWheels = 0;

	float frontWheelMult = 0;
	float rearWheelMult = 0;
	float passiveWheelMult = 0;

	///<summary>The shared acceleration multiplier for the vehicle</summary>
	[HideInInspector]
	public float accelMult = 0;
	float slopeAccelerationMult = 0;
	float clutchAmount = 0;
	int currentGear = 0;
	int targetGear = 0;
	float gearTimer = -1;
	bool isShifting = false;
	float velKmh = 0;
	float curTopSpeed = 0;
	float speedPerc = 0;
	float upShiftPoint = 0;
	float dnShiftPoint = 0;
	List<int> poweredWheels;

	float steerInput;
	float forwardInput;
	float reverseInput;
	bool handbrakeInput;

	float motorInput;
	float brakeInput;
	bool isDrifting = false;
	bool wantToDrift = false;
	bool absLock = false;

	public SoundData soundData = new SoundData();

	void Start ()
	{
		rBody = transform.root.GetComponent<Rigidbody>();
		rBody.inertiaTensor = Vector3.Scale(rBody.inertiaTensor, inertiaMod);
		//rBody.centerOfMass = centreOfMass;

		lastChassisPos = transform.position;
		lastChassisRot = transform.rotation;

		for(int i = 0; i < wheels.Length; i++)
		{
			wheels[i].twin = -1;
		}

		frontWheels = 0;
		rearWheels = 0;
		passiveWheels = 0;
		int pairs = 0;
		Vector2 wheelbaseWidth = Vector2.zero;
		Vector2 wheelbaseLength = Vector2.zero;
		poweredWheels = new List<int>();
		for(int i = 0; i < wheels.Length; i++)
		{
			wheels[i].lastWorldPoses = new Vector3[4];
			wheels[i].lastWorldPoses[0] = wheels[i].wheelPivot.position - wheels[i].wheelPivot.up * wheels[i].wheelRadius;
			wheels[i].lastWorldPoses[1] = wheels[i].wheelPivot.position - wheels[i].wheelPivot.up * wheels[i].wheelRadius;
			wheels[i].lastWorldPoses[2] = wheels[i].wheelPivot.position - wheels[i].wheelPivot.up * wheels[i].wheelRadius;
			wheels[i].lastWorldPoses[3] = wheels[i].wheelPivot.position - wheels[i].wheelPivot.up * wheels[i].wheelRadius;

			wheels[i].stopPosWorld = wheels[i].wheelPivot.position;

			if (wheels[i].isPassive)
			{
				passiveWheels++;
			}
			else
			{
				if (wheels[i].isFront) { frontWheels++; steerAngleLocationRear += wheels[i].wheelPivot.localPosition; } else { rearWheels++; steerAngleLocationFront += wheels[i].wheelPivot.localPosition; }

				if (wheels[i].canSteer)
				{
					steerFactor += (wheels[i].isFront ? cSteerFrontAmount : cSteerRearAmount);
					steeringWheels++;
				}

				wheelbase += new Vector2(Mathf.Abs(wheels[i].wheelPivot.localPosition.x), wheels[i].wheelPivot.localPosition.z);

				wheelbaseLength = new Vector2(Mathf.Min(wheels[i].wheelPivot.localPosition.z, wheelbaseLength.x), Mathf.Max(wheels[i].wheelPivot.localPosition.z, wheelbaseLength.y));
				wheelbaseWidth = new Vector2(Mathf.Min(wheels[i].wheelPivot.localPosition.x, wheelbaseWidth.x), Mathf.Max(wheels[i].wheelPivot.localPosition.x, wheelbaseWidth.y));

				if((wheels[i].isFront && mBiasFront > 0) || (!wheels[i].isFront && mBiasFront < 1))
				{
					poweredWheels.Add(i);
				}
			}



			for (int j = 0; j < wheels.Length; j++)
			{
				if (j == i) continue;

				if (wheels[i].twin != -1) continue;

				float wheelDistZ = Mathf.Abs(wheels[j].wheelPivot.localPosition.z - wheels[i].wheelPivot.localPosition.z);
				float averageRadius = (wheels[j].wheelRadius + wheels[j].wheelRadius) * 0.5f;

				if (wheelDistZ < averageRadius)
				{
					wheels[i].twin = j;
					wheels[j].twin = i;

					pairs++;
				}
			}
		}

		//Debug.Log("Wheel pairs " + gameObject.name + ": " + pairs);

		steerAngleLocationFront = new Vector3(0, steerAngleLocationFront.y / rearWheels, steerAngleLocationFront.z / rearWheels);
		steerAngleLocationRear = new Vector3(0, steerAngleLocationRear.y / frontWheels, steerAngleLocationRear.z / frontWheels);

		wheelbaseCentre = (steerAngleLocationFront + steerAngleLocationRear) * 0.5f;

		rBody.centerOfMass = centreOfMass + wheelbaseCentre;

		//steerFactor = 1f / steerFactor;
		steerFactorReciprocated = 1f / steerFactor;
		steerFactorReciprocated = (1f / ((steerFactor / steeringWheels) * wheels.Length));

		float wheelbaseLengthAbs = Mathf.Abs(wheelbaseLength.x) + Mathf.Abs(wheelbaseLength.y);
		float wheelbaseWidthAbs = Mathf.Abs(wheelbaseWidth.x) + Mathf.Abs(wheelbaseWidth.y);

		wheelbase = new Vector2(wheelbaseWidthAbs, wheelbaseLengthAbs);
		//wheelbase.x /= wheels.Length * 0.5f;

		frontWheelMult = 1f / (frontWheels * 2 + passiveWheels);//1f / (frontWheels * 2);
		rearWheelMult = 1f / (rearWheels * 2 + passiveWheels);//1f / (rearWheels * 2);
		passiveWheelMult = ((float)passiveWheels / wheels.Length) * 0.5f;//1f / (passiveWheels * 2);


		currentGear = 1;
		targetGear = 1;
		gearTimer = mShiftTime;

		soundData = new SoundData();
		soundData.wheelSounds = new WheelSoundData[wheels.Length];

		for(int i = 0; i < soundData.wheelSounds.Length; i++)
		{
			soundData.wheelSounds[i] = new WheelSoundData();
		}
	}

	private void OnDrawGizmos()
	{

		////DEBUGGING////

		//Quaternion testRot = Quaternion.Euler(Vector3.zero);
		//Debug.DrawRay(transform.position, testRot * rBody.rotation * Vector3.right, Color.red, 0, false);
		//Debug.DrawRay(transform.position, testRot * rBody.rotation * Vector3.up, Color.green, 0, false);
		//Debug.DrawRay(transform.position, testRot * rBody.rotation * Vector3.forward, Color.blue, 0, false);

		if (debugWeight)
		{
			//Points of interest
			HelperMath.DebugDrawCross(transform, steerAngleLocationFront, 0.1f, Color.white, 0);
			HelperMath.DebugDrawCross(transform, steerAngleLocationRear, 0.1f, Color.white, 0);
			HelperMath.DebugDrawCross(transform, wheelbaseCentre, 0.1f, Color.white, 0);
			HelperMath.DebugDrawCross(transform, rBody.centerOfMass, 0.1f, Color.magenta, 0);
		}


		if(wheels.Length != 0)
		{
			for (int i = 0; i < wheels.Length; i++)
			{
				if (debugStopPoints)
				{
					//Stop Position
					HelperMath.DebugDrawCross(wheels[i].stopPosWorld, 0.1f, Color.blue, 0);
					HelperMath.DebugDrawCross(wheels[i].wheelPivot.position, 0.1f, Color.magenta, 0);

					//Wheel trail
					HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[1], 0.03f, Color.Lerp(Color.yellow, Color.red, 0.333f), 0);
					HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[2], 0.03f, Color.Lerp(Color.yellow, Color.red, 0.666f), 0);
					HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[3], 0.03f, Color.red, 0);
				}

				if (debugSuspension)
				{
					Vector3 startPoint = wheels[i].wheelCastPoint.position - transform.up * wheels[i].wheelRadius + wheels[i].wheelCastPoint.up * sVerticalOffset;
					//Vector3 twinStartPoint = wheels[wheels[i].twin].wheelCastPoint.position - transform.up * wheels[wheels[i].twin].wheelRadius + wheels[i].wheelCastPoint.up * sVerticalOffset;
					Vector3 offsetDir = wheels[i].isLeft ? -wheels[i].wheelPivot.right : wheels[i].wheelPivot.right;

					//Suspension info
					Color gripColor = Color.magenta;
					if (wheels[i].debugGripMult >= 1)
					{
						gripColor = Color.Lerp(Color.black, Color.green, Mathf.Clamp01(wheels[i].debugGripMult - 1));
					}
					else
					{
						gripColor = Color.Lerp(Color.red, Color.black, Mathf.Clamp01(wheels[i].debugGripMult));
					}
					Debug.DrawRay(startPoint, wheels[i].debugSpringForce, gripColor, 0, false);
					Debug.DrawRay(startPoint + offsetDir * 0.05f, wheels[i].rHit.normal * wheels[i].debugDampingForce, new Color(0, 0.5f, 1, 0.25f), 0, false);

					Color rollColor = new Color(1, 0.5f, 0, 0.25f);
					HelperMath.DebugDrawCross(startPoint + offsetDir * 0.1f, 0.05f, rollColor * 0.5f, 0);
					Debug.DrawRay(startPoint + offsetDir * 0.1f, wheels[i].rHit.normal * wheels[i].debugRollbarForce, rollColor, 0, false);
				}

				if(debugDifferentials)
				{
					Debug.DrawRay(wheels[i].wheelPivot.position, wheels[i].wheelPivot.up * wheels[i].diffPower, Color.magenta, 0, false);
				}

				if (debugRaycast)
				{
					Color groundedColor = Color.green;// wheels[i].isGrounded ? Color.green : Color.black;

					HelperMath.DebugDrawCross(wheels[i].rHit.point, 0.1f, groundedColor, 0);
					HelperMath.DebugDrawCross(wheels[i].wheelCastPoint.position + -wheels[i].wheelCastPoint.up * sRestDistance + wheels[i].wheelCastPoint.up * sVerticalOffset, 0.05f, Color.white, 0);
					HelperMath.DebugDrawCross(wheels[i].wheelCastPoint.position + -wheels[i].wheelCastPoint.up * sUpperLimit + wheels[i].wheelCastPoint.up * sVerticalOffset, 0.05f, Color.cyan, 0);
					Debug.DrawRay(wheels[i].wheelCastPoint.position + wheels[i].wheelCastPoint.up * sVerticalOffset, -wheels[i].wheelCastPoint.up * sMaxDistance, groundedColor, 0, false);
				}

				if(debugWheelSpeed)
				{
					Color speedColor = wheels[i].isLocked ? new Color(0, 0.5f, 1.0f, 0.25f) : new Color(1, 0, 0.5f, 0.25f);
					float lineHeight = wheels[i].isLocked ? 0.5f : wheels[i].rpm * Time.deltaTime;
					Debug.DrawRay(wheels[i].wheelCastPoint.position, wheels[i].wheelCastPoint.up * lineHeight, speedColor, 0, false);
				}

			}
		}
		

		if(debugEngineRPM)
		{
			Vector3 speedPos = transform.position + -transform.right * wheelbase.x * 0.75f;
			Vector3 rpmPos = transform.position + -transform.right * wheelbase.x;

			//HelperMath.DebugDrawCross(speedPos, 0.1f, Color.grey, 0);
			for (int i = 0; i < mGears; i++)
			{
				int gearNum = i + 1;
				float offset = (1f / mGears) * gearNum;

				Vector3 pos = speedPos + transform.up * offset;

				Color gearColor = gearNum == currentGear ? Color.green : Color.white;
				//HelperMath.DebugDrawCross(speedPos + transform.up * offset, 0.1f, gearColor, 0);
				Debug.DrawRay(pos, transform.right * 0.1f, gearColor, 0, false);
				Debug.DrawRay(pos - transform.up * 0.01f, transform.right * 0.1f, Color.black, 0, false);
			}

			if(currentGear > 0)
			{
				float offset = velKmh / mTopSpeedKmh;
				Debug.DrawRay(speedPos + -transform.right * 0.05f, transform.up * offset, Color.yellow, 0, false);
				HelperMath.DebugDrawCross(speedPos + -transform.right * 0.05f + transform.up * offset, 0.05f, Color.black, 0);
			}
			else
			{
				float offset = (velKmh / mReverseKmh) * (mReverseKmh / mTopSpeedKmh);
				Debug.DrawRay(speedPos + -transform.right * 0.05f, transform.up * -offset, Color.yellow, 0, false);
				HelperMath.DebugDrawCross(speedPos + -transform.right * 0.05f + transform.up * -offset, 0.05f, Color.black, 0);
			}

			Color rpmCol = Color.Lerp(Color.green, Color.red, Mathf.Clamp01(soundData.engineRPM - 0.75f) * 4);
			Debug.DrawRay(rpmPos, transform.up * soundData.engineRPM, rpmCol, 0, false);

			//Debug.Log(soundData.engineRPM);
			
		}

		
	}

	//for debugging collisions
	private void OnCollisionEnter(Collision collision)
	{
		if(debugCollisions)
		{
			for (int i = 0; i < collision.contacts.Length; i++)
			{
				Debug.DrawRay(collision.contacts[i].point, collision.contacts[i].normal, Color.green, 0, false);
			}
		}
	}

	private void OnCollisionStay(Collision collision)
	{
		if (debugCollisions)
		{
			for (int i = 0; i < collision.contacts.Length; i++)
			{
				Debug.DrawRay(collision.contacts[i].point, collision.contacts[i].normal, Color.green, 0, false);
			}
		}
	}

	//private void OnGUI()
	//{
	//	for(int i = 0; i < wheels.Length; i++)
	//	{
	//		if(debugFriction)
	//		{
	//			GUILayout.Label("Grip of wheel " + i + ": " + wheels[i].gripMult.ToString());
	//		}
	//	}

	//	Vector2 debugSteer = GetAckermannAngles(steerInput * cSteerMax);

	//	GUILayout.Label("InsideSteer " + debugSteer.x.ToString() + "| OutsideSteer " + debugSteer.y.ToString());
	//	GUILayout.Label("WheelBaseCrossAngle " + (Mathf.Atan2(wheelbase.x * 2, wheelbase.y) * Mathf.Rad2Deg).ToString());
	//}


	void PollInputs()
	{
		//Vehicle movement & smoothing does not matter for the inputs, it will sort that out itself.
		//Only apply deadzone filtering for best results. Remove the hardcoded deadzone if that's sorted elsewhere.

		//You can rewrite all of this apart from the logic translator below.

		if(isDriving)
		{
			steerInput = Input.GetAxisRaw(steer);
			forwardInput = Input.GetAxisRaw(forward);
			reverseInput = Input.GetAxisRaw(reverse);
			handbrakeInput = Input.GetButton(handbrake);

			////HARDCODED DEADZONE FOR NOW////

			forwardInput = Mathf.Clamp01(forwardInput);
			reverseInput = Mathf.Clamp01(-1 * reverseInput);

			float sign = Mathf.Sign(steerInput);

			steerInput = Mathf.Clamp01((Mathf.Abs(steerInput) - 0.1f) / 0.9f);
			steerInput = Mathf.Pow(steerInput, 1.5f) * sign;
		}
		else
		{
			steerInput = 0;
			forwardInput = 0;
			reverseInput = 0;
			handbrakeInput = true;
		}

		//////////////////////////////////////////////////
		//	Vehicle input logic translator, no touchy!	//
		//////////////////////////////////////////////////

		if (velocity.z > cFreeVelocity)
		{
			motorInput = forwardInput;
			brakeInput = reverseInput;
		}
		else if(velocity.z < -cFreeVelocity)
		{
			motorInput = -reverseInput;
			brakeInput = forwardInput;
		}
		else
		{
			if(forwardInput != 0 || reverseInput != 0 || !isGrounded)
			{
				motorInput = (forwardInput - reverseInput);
				brakeInput = forwardInput > 0.9f && reverseInput > 0.9f ? 1 : 0;
			}
			else
			{
				motorInput = 0;
				brakeInput = 1;
			}

		}

		if(cInvertSteeringInReverse)
		{
			if(velocity.z < 0)
				steerInput *= -1;
		}

		isBraking = brakeInput > 0;
		isReversing = currentGear < 0;		

		//Debug.DrawRay(transform.position, transform.forward * forwardInput, Color.green, 0, false);
		//Debug.DrawRay(transform.position, -transform.forward * reverseInput, Color.red, 0, false);
	}

	void CalculateSound()
	{

		if(!isShifting)
		{
			for(int i = 0; i < wheels.Length; i++)
			{
				Vector3 contactPoint = wheels[i].wheelCastPoint.position + -wheels[i].wheelCastPoint.up * (wheels[i].rHit.distance - sVerticalOffset);

				soundData.wheelSounds[i].SetWheelRPM(wheels[i].rpm);
				soundData.wheelSounds[i].SetWheelContactPoint(contactPoint);
				soundData.wheelSounds[i].SetWheelSlip(wheels[i].wheelSlide);
				soundData.wheelSounds[i].SetWheelBraking(wheels[i].wheelBrakeAmount);
				soundData.wheelSounds[i].SetSuspensionForce(wheels[i].suspensionCompressionForce);
				soundData.wheelSounds[i].SetSuspensionLength(wheels[i].rHit.distance);
			}

			float engineRPM = 0;
			float wheelSpeed = 0;
			for (int i = 0; i < poweredWheels.Count; i++)
			{
				wheelSpeed = AbsoluteLargest(wheelSpeed, wheels[poweredWheels[i]].moveDistance);
			}
			wheelSpeed /= Time.deltaTime / 3.6f;

			engineRPM = Mathf.Max(0, Mathf.Abs(wheelSpeed) / curTopSpeed);
			if (engineRPM >= 0.99f && Mathf.Abs(motorInput) > 0.99f) { engineRPM = 2; }

			//clutchAmount = Mathf.Abs(wheelSpeed) > 0 ? Mathf.Pow(Mathf.Abs(motorInput), 0.5f) : 0;
			clutchAmount = Mathf.Abs(wheelSpeed) > 0 ? Mathf.Clamp01(Mathf.Abs(motorInput) * 2.0f) : 0;

			soundData.SetEngineRPM(engineRPM);
			soundData.SetEngineLoad(slopeAccelerationMult);
			soundData.SetIsShifting(false);
			soundData.SetIsClutchEngaged(clutchAmount);
		}
		else
		{
			soundData.SetEngineRPM(0);
			soundData.SetEngineLoad(0);
			soundData.SetIsShifting(true);
			soundData.SetIsClutchEngaged(0);
		}
		
	}

	void Update()
	{
		PollInputs();

		////VISUALS INFO////

		
		for (int i = 0; i < wheels.Length; i++)
		{
			
			float motorBiasMult = 0;
			float brakeBiasMult = 0;
			if(wheels[i].isFront)
			{
				motorBiasMult = mBiasFront;
				brakeBiasMult = bBiasFront;
			}
			else
			{
				motorBiasMult = 1 - mBiasFront;
				brakeBiasMult = 1 - bBiasFront;
			}
			float motorAmountAbs = Mathf.Abs(motorInput * mAcceleration * motorBiasMult);
			//float motorAmount = motorInput * mAcceleration * motorBiasMult;
			float brakeAmount = brakeInput * bDeceleration * brakeBiasMult;


			if (wheels[i].isLocked)
			{
				wheels[i].rpm = 0;

				wheels[i].moveDistance = 0;
			}
			else if (wheels[i].isGrounded)
			{
				float dirVel = -wheels[i].wheelCastPoint.InverseTransformPoint(wheels[i].lastWorldPos).z;
				float spinVel = (wheels[i].spinAmount >= 0 ? curTopSpeed / 3.6f : -mReverseKmh / 3.6f) * Time.deltaTime;

				float wheelVel = Mathf.Lerp(dirVel, spinVel, Mathf.Abs(wheels[i].spinAmount));
				//float rotation = 360 * dirVel * wheels[i].wheelRadius * Mathf.PI;
				float rotation = (wheelVel / (wheels[i].wheelRadius * Mathf.PI * 2)) * 360;

				wheels[i].rpm = rotation;

				wheels[i].rotation += rotation;

				wheels[i].moveDistance = wheelVel;
			}
			else
			{

				if(motorAmountAbs >= brakeAmount)
				{
					float wheelVel = curTopSpeed * motorInput * Mathf.Ceil(motorBiasMult);
					float rpm = ((wheelVel / (wheels[i].wheelRadius * Mathf.PI * 2)) * 360) * Time.deltaTime;
					//wheels[i].rpm = (mReverseKmh / (wheels[i].wheelRadius * Mathf.PI * 2)) * 360 * (motorAmountAbs / mAcceleration);
					
					if(Mathf.Abs(rpm) < Mathf.Abs(wheels[i].rpm))
					{
						float decay = 0.98f;
						if((wheels[i].isFront && mBiasFront > 0) || (!wheels[i].isFront && mBiasFront < 1))
						{
							decay = (mAcceleration / (mAcceleration - mEngineDeceleration * 0.1f));
							decay = Mathf.Min(0.98f, decay);
						}

						wheels[i].rpm *= decay;

						wheels[i].moveDistance *= decay;
					}
					else
					{
						float rpmMult = (1f / (wheels[i].wheelRadius * Mathf.PI * 2)) * 360 * Time.deltaTime;
						wheels[i].rpm = Mathf.Clamp(wheels[i].rpm + rpm * Mathf.Abs(motorInput), -mReverseKmh / 3.6f * rpmMult, curTopSpeed / 3.6f * rpmMult);

						wheels[i].moveDistance = wheelVel;
					}

				}
				else
				{
					float decay = 0.98f;
					if ((wheels[i].isFront && mBiasFront > 0) || (!wheels[i].isFront && mBiasFront < 1))
					{
						decay = (mAcceleration / (mAcceleration - mEngineDeceleration * 0.1f));
						decay = Mathf.Min(0.98f, decay);
					}

					wheels[i].rpm *= decay;

					wheels[i].moveDistance *= decay;
				}

				wheels[i].rotation += wheels[i].rpm;

			}

			wheels[i].lastWorldPos = wheels[i].wheelCastPoint.position;
		}

		CalculateSound();
	}



	void FixedUpdate()
	{
		Vector3 vel = transform.InverseTransformDirection(rBody.velocity);
		velocity = new Vector4(vel.x, vel.y, vel.z, vel.magnitude);
		stationaryMult = 1 - Mathf.Clamp01(velocity.w - 4);
		//if(forwardInput > 0.1f || reverseInput > 0.1f)
		//{
		//	stationaryMult = 0;
		//}

		wantToMove = (forwardInput != 0) || (reverseInput != 0); 

		CalculateChassisStop();

		CalculateSteering();

		RaycastWheels();

		absLock = cHasABS ? !absLock : true;

		accelMult = CalculateGearing(velocity.z, velocity.w);

		accelMult = CalculateSlopeCompensation(accelMult);

		// Mathf.Lerp(-mEngineDeceleration * Mathf.Clamp(vel.z, -1, 1), mAcceleration * accelMult * motorInput, Mathf.Abs(motorInput));
		float engineBraking = Mathf.Lerp(mEngineDeceleration, 0, Mathf.Ceil(Mathf.Abs(motorInput)));

		float accelForce = mAcceleration * accelMult * motorInput;
		float brakeForce = brakeInput * bDeceleration;

		//Debug.Log("Velocity " + velocity.w * 3.6f + " | AccelMult " + accelMult);

		if ((stationaryMult > 0 && !wantToMove))
		{
			accelForce = 0;
			brakeForce = bDeceleration;
		}

		for (int i = 0; i < wheels.Length; i++)
		{
			float gripBias = wheels[i].isFront ? tBiasFront * 2 : (1 - tBiasFront) * 2;

			float wheelMult = wheels[i].isPassive ? passiveWheelMult : (wheels[i].isFront ? frontWheelMult : rearWheelMult);

			RunSuspension(i, gripBias, wheels[i].isFront ? sRollHeightFront : sRollHeightRear, wheelMult);
		}

		for (int i = 0; i < wheels.Length; i++)
		{
			////PREP DATA////

			ShiftWorldPoses(i, wheels[i].wheelPivot.position);
			CalculateFrictionVectors(i);

			////SIMULATION EXECUTION////

			float thisGrip = wheels[i].gripMult;
			float twinGrip = wheels[wheels[i].twin].gripMult;

			float gripDiff = (thisGrip - twinGrip) / thisGrip;
			if (gripDiff > 0) { gripDiff *= 2; }
			gripDiff = (gripDiff + 1) * 0.5f;

			float openPower = 1 - gripDiff;
			float lockedPower = gripDiff;

			//if (wheels[i].isGrounded)
			{
				float acc = 0;
				float brk = 0;
				float rollHeight = 0;
				if(wheels[i].isFront)
				{
					acc = accelForce * mBiasFront;
					brk = brakeForce * bBiasFront + engineBraking * mBiasFront * accelMult;

					wheels[i].wheelBrakeAmount = wheels[i].isLocked ? 0 : mBiasFront * brakeInput;

					rollHeight = sRollHeightFront;

					float diffMult = Mathf.Lerp(openPower, lockedPower, mDiffStiffnessFront) * 2;

					acc *= diffMult;
					//brk *= diffMult;

					wheels[i].diffPower = diffMult;
				}
				else
				{
					acc = accelForce * (1 - mBiasFront);

					if(handbrakeInput)
					{
						brk = tMaxGrip * 10;
					}
					else
					{
						brk = brakeForce * (1 - bBiasFront) + engineBraking * (1 - mBiasFront) * accelMult;
					}

					wheels[i].wheelBrakeAmount = wheels[i].isLocked ? 0 : (1 - mBiasFront) * brakeInput;					

					rollHeight = sRollHeightRear;

					float diffMult = Mathf.Lerp(openPower, lockedPower, mDiffStiffnessRear) * 2;

					acc *= diffMult;
					//brk *= diffMult;

					wheels[i].diffPower = diffMult;
				}

				float wheelMult = wheels[i].isPassive ? passiveWheelMult : (wheels[i].isFront ? frontWheelMult : rearWheelMult);

				RunFriction(i, acc, brk, rollHeight, wheelMult);
			}

			////SET INFO FOR NEXT FRAME////

			wheels[i].prevSuspDist = wheels[i].rHit.distance;

		}

		for (int i = 0; i < wheels.Length; i++)
		{
			if (wheels[i].rHit.distance <= sUpperLimit)
			{
				wheels[i].rHit.distance = sUpperLimit;
			}

			wheels[i].rHit.distance -= sVerticalOffset;
		}
	}

	private Vector2 GetAckermannAngles(float angle)
	{
		float wheelBaseCrossAngle = Mathf.Atan2(wheelbase.x, wheelbase.y);
		float insideSteer = 1 - Mathf.Cos((Mathf.Abs(cSteerMax) / 180) * Mathf.PI);

		float leftRatio = (wheelBaseCrossAngle / (Mathf.Abs(insideSteer) + Mathf.Epsilon));
		//leftRatio = ((Mathf.Abs(insideSteer)) / wheelBaseCrossAngle);

		//Debug.Log(leftRatio);
		float insideSteerMult = Mathf.Abs(angle) / cSteerMax;

		float outsideSteer = 90 - Mathf.Abs(Mathf.Atan2(leftRatio * wheelbase.x + wheelbase.x, wheelbase.y) * Mathf.Rad2Deg);
		//90 - Mathf.Abs(Mathf.Atan2(wheelbase.y, leftRatio * wheelbase.x) * Mathf.Rad2Deg);

		return new Vector2(angle, outsideSteer * insideSteerMult * Mathf.Sign(angle));
	}

	void CalculateSteering()
	{
		Vector3 angularVelocity = rBody.angularVelocity * Mathf.Rad2Deg;// * steerFactorReciprocated;
																		//angularVelocity = transform.InverseTransformDirection(angularVelocity);

		Vector3 localVelocityFront = transform.InverseTransformDirection(rBody.GetPointVelocity(transform.TransformPoint(steerAngleLocationFront) ));
		float chassisSteerAngleFront = Mathf.Atan2(localVelocityFront.x, localVelocityFront.z) * Mathf.Rad2Deg * inertiaMod.y;
		//chassisSteerAngleFront = Mathf.Clamp(chassisSteerAngleFront, -tPeakAngle, tPeakAngle);

		wantToDrift = wantToDrift && Mathf.Abs(motorInput) > 0.5f && Mathf.Abs(chassisSteerAngleFront) >= tPeakAngle;

		if(!wantToDrift)
		{
			if (stationaryMult == 0 && Mathf.Abs(motorInput) > 0.5f && Mathf.Abs(chassisSteerAngleFront) >= tPeakAngle)
			{
				wantToDrift = true;
				isDrifting = true;
			}
		}

		isDrifting = isDrifting && wantToDrift;

		Vector3 localVelocityRear = transform.InverseTransformDirection(rBody.GetPointVelocity(transform.TransformPoint(steerAngleLocationRear)));
		float chassisSteerAngleRear = Mathf.Atan2(localVelocityRear.x, localVelocityRear.z) * Mathf.Rad2Deg * inertiaMod.y;
		//chassisSteerAngleRear = Mathf.Clamp(chassisSteerAngleRear, -tPeakAngle, tPeakAngle);

		float counterLimit = cFullCounterLock ? cSteerMax : tPeakAngle;

		float chassisRotationAngle = -angularVelocity.y * inertiaMod.y * cSteerRotateCounter;
		chassisRotationAngle = Mathf.Clamp(chassisRotationAngle, -counterLimit, counterLimit);

		float counterAngleFront = Mathf.Clamp(chassisSteerAngleFront, -counterLimit * cSteerAngleCounter, counterLimit * cSteerAngleCounter);
		float counterAngleRear = Mathf.Clamp(chassisSteerAngleRear, -counterLimit * cSteerAngleCounter, counterLimit * cSteerAngleCounter);


		float finalCounterAngleFront = AbsoluteLargest(chassisRotationAngle, counterAngleFront) * (1 - stationaryMult);
		float finalCounterAngleRear = AbsoluteLargest(chassisRotationAngle, counterAngleRear) * (1 - stationaryMult);


		if(velocity.z < cFreeVelocity)
		{
			if (cCounterInReverse)
			{
				finalCounterAngleFront = -finalCounterAngleFront;
				finalCounterAngleRear = -finalCounterAngleRear;
			}
			else
			{
				finalCounterAngleFront = 0;
				finalCounterAngleRear = 0;
			}
		}

		//if (handbrakeInput)
		//{
		//	chassisSteerAngleFront = 0;
		//	chassisSteerAngleRear = 0;
		//}

		float steeringTangentFactor = tPeakAngle * Mathf.Deg2Rad;
		steeringTangentFactor = steeringTangentFactor / Mathf.Tan(steeringTangentFactor);

		float avgForwardVelocity = (localVelocityFront.z + localVelocityRear.z) * (1 - stationaryMult) * 0.5f * steerFactorReciprocated;
		float maxSteering = (steeringTangentFactor / Mathf.Clamp(avgForwardVelocity, steeringTangentFactor, Mathf.Infinity)) * 90;
		maxSteering = Mathf.Lerp(maxSteering, maxSteering * 0.5f, Mathf.Max(motorInput) * mBiasFront);
		//float frontMaxSteering = Mathf.Clamp(maxSteering, tPeakAngle / Mathf.Lerp(1, frontWheels, cSteerAngleCounter), cSteerMax);
		//float rearMaxSteering = Mathf.Clamp(maxSteering, tPeakAngle / Mathf.Lerp(1, rearWheels, cSteerAngleCounter), cSteerMax);
		float frontMaxSteering = Mathf.Clamp(maxSteering, tPeakAngle, cSteerMax);
		float rearMaxSteering = Mathf.Clamp(maxSteering, tPeakAngle, cSteerMax);

		counterAmount = 0;
		if(Mathf.Sign(steerInput) == Mathf.Sign(chassisSteerAngleFront))
		{
			frontMaxSteering = Mathf.Clamp(Mathf.Abs(chassisSteerAngleFront), frontMaxSteering, cSteerMax);
			rearMaxSteering = Mathf.Clamp(Mathf.Abs(chassisSteerAngleRear), rearMaxSteering, cSteerMax);

			//if (Mathf.Abs(chassisSteerAngleFront) > frontMaxSteering)
			//{
				
			//	if (cRearWheelsCanCounter)
			//	{
			//		rearMaxSteering = cSteerMax;
			//	}

			//}
			
			if(localVelocityFront.z > 1)
			{
				counterAmount = 0;//Mathf.Abs(steerInput);
			}
			
		}
		//steerAngle = Mathf.Lerp(steerAngle, steerInput * maxSteering, Time.deltaTime * cSteerSpeed);
		frontMaxSteering = Mathf.Lerp(cSteerMax, frontMaxSteering, cSteerAngleDecrease);
		rearMaxSteering = Mathf.Lerp(cSteerMax, rearMaxSteering, cSteerAngleDecrease);

		if(frontMaxSteering < tPeakAngle)
		{
			Debug.LogError(gameObject.name + "'s steering angle is too low");
		}

		float assistSteeringFront = finalCounterAngleFront * (1 - Mathf.Abs(steerInput));
		float finalSteeringFront = Mathf.Clamp(assistSteeringFront + ((frontMaxSteering) * steerInput * cSteerFrontAmount), -frontMaxSteering, frontMaxSteering);
		//finalSteering = Mathf.Clamp(assistSteering * (1 - Mathf.Abs(steerInput)) + steerAngle, -maxSteering, maxSteering);
		//finalSteering = Mathf.Lerp(assistSteering, maxSteering * Mathf.Sign(steerInput), Mathf.Abs(steerInput));

		float assistSteeringRear = finalCounterAngleFront * (1 - Mathf.Abs(steerInput));
		float rearSteeringAngle = ((rearMaxSteering) * steerInput * cSteerRearAmount);
		float finalSteeringRear = 0;//Mathf.Clamp((maxSteering * steerInput) - (Mathf.Abs(rearAssistAngle) * Mathf.Sign(steerInput)), -maxSteering, maxSteering);
		if (cRearWheelsCanCounter)
		{
			finalSteeringRear = Mathf.Clamp(rearSteeringAngle + assistSteeringRear, -rearMaxSteering, rearMaxSteering);
		}
		else
		{
			if (Mathf.Abs(assistSteeringRear) > Mathf.Abs(rearSteeringAngle))
			{
				finalSteeringRear = 0;
			}
			else
			{
				finalSteeringRear = Mathf.Max(0, Mathf.Abs(rearSteeringAngle) - Mathf.Abs(assistSteeringRear)) * Mathf.Sign(rearSteeringAngle);
				//finalSteeringRear = Mathf.Lerp(rearSteeringAngle, (Mathf.Abs(rearSteeringAngle) - Mathf.Abs(rearAssistAngle)) * Mathf.Sign(rearSteeringAngle), Mathf.Abs(rearAssistAngle) / tPeakAngle);
				finalSteeringRear = Mathf.Clamp(finalSteeringRear, -rearMaxSteering, rearMaxSteering);
			}

			finalSteeringRear = Mathf.Lerp(finalSteeringRear, 0, counterAmount);
		}



		

		steerAngleFront = Mathf.Lerp(steerAngleFront, finalSteeringFront, Time.deltaTime * cSteerSpeed);
		steerAngleRear = Mathf.Lerp(steerAngleRear, finalSteeringRear, Time.deltaTime * cSteerSpeed);

		Vector2 ackermannFront = GetAckermannAngles(steerAngleFront);
		Vector2 ackermannRear = GetAckermannAngles(steerAngleRear);
		for (int i = 0; i < wheels.Length; i++)
		{
			if (wheels[i].canSteer)
			{
				if (wheels[i].isFront)
				{
					if(wheels[i].isLeft)
					{
						if(steerAngleFront > 0)
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, ackermannFront.y, 0);
						}
						else
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, ackermannFront.x, 0);
						}	
					}
					else
					{
						if (steerAngleFront > 0)
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, ackermannFront.x, 0);
						}
						else
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, ackermannFront.y, 0);
						}
					}
				}
				else
				{
					if (wheels[i].isLeft)
					{
						if (steerAngleRear > 0)
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, -ackermannRear.y, 0);
						}
						else
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, -ackermannRear.x, 0);
						}
					}
					else
					{
						if (steerAngleRear > 0)
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, -ackermannRear.x, 0);
						}
						else
						{
							wheels[i].wheelPivot.localEulerAngles = new Vector3(0, -ackermannRear.y, 0);
						}
					}
				}
			}
		}
	}


	void CalculateChassisStop()
	{
		bool recalc = false;

		//TODO: find better solution to the moving car problem.
		bool wantToMove = false;// motorInput != 0 && !handbrakeInput;

		float chassisDistDelta = Vector3.Distance(rBody.position, lastChassisPos);
		if (chassisDistDelta > tStopPositionRadius || wantToMove)
		{
			//Vector3 chassisDir = -(rBody.worldCenterOfMass - lastChassisPos).normalized * tStopRadius;
			//lastChassisPos = rBody.worldCenterOfMass + chassisDir;

			if(!wantToMove)
			{
				Vector3 chassisDir = -(rBody.position - lastChassisPos).normalized * tStopPositionRadius;
				lastChassisPos = rBody.position + chassisDir;
			}
			else
			{
				lastChassisPos = rBody.position;
			}
			
			recalc = true;
		}

		float wheelbaseVelocity = rBody.GetPointVelocity(rBody.worldCenterOfMass).magnitude;
		float refPeakAngle = (tPeakAngle / (wheelbase.x + wheelbase.y)) * (1 - Mathf.Clamp01(wheelbaseVelocity - 0.65f));
		//float refDistance = chassisDistDelta / tStopPositionRadius;

		//if (isDriving) Debug.Log( 1 - Mathf.Clamp01(velocity.w) );

		//float chassisAngle = Quaternion.Angle(lastChassisRot, rBody.rotation);
		//if (chassisAngle > refPeakAngle || refDistance > 2 || wantToMove)
		{
			Vector2 checkAngles = Vector2.zero;
			Quaternion newAngles = rBody.rotation;
			//if(!wantToMove)
			{
				Vector3 localChassisForward = transform.InverseTransformDirection(lastChassisRot * Vector3.forward);
				Vector3 localChassisUp = transform.InverseTransformDirection(lastChassisRot * Vector3.up);
				float chassisVerticalAngle = Mathf.Atan2(localChassisForward.x, localChassisForward.z) * Mathf.Rad2Deg;
				//float chassisHorizontalAngle = Mathf.Atan2(localChassisForward.y, localChassisForward.z) * Mathf.Rad2Deg;
				float chassisLongitudinalAngle = Mathf.Atan2(localChassisUp.x, localChassisUp.y) * Mathf.Rad2Deg;

				//if (isDriving) Debug.Log(chassisAngle > refPeakAngle);//Debug.Log(chassisVerticalAngle + " " + chassisLongitudinalAngle);

				newAngles = Quaternion.Euler(0, Mathf.Clamp(chassisVerticalAngle, -refPeakAngle, refPeakAngle), Mathf.Clamp(chassisLongitudinalAngle, -refPeakAngle, refPeakAngle));

				newAngles = newAngles * rBody.rotation;

				checkAngles = new Vector2(Mathf.Abs(chassisVerticalAngle), Mathf.Abs(chassisLongitudinalAngle));
				recalc = checkAngles.x > refPeakAngle || checkAngles.y > refPeakAngle || recalc;
			}
			//else
			//{
			//	newAngles = rBody.rotation;
			//}

			lastChassisRot = newAngles;

			
			//recalc = true;
		}

		if (recalc)
		{
			Vector3 side = lastChassisRot * Vector3.right;
			Vector3 up = lastChassisRot * Vector3.up;
			Vector3 fwd = lastChassisRot * Vector3.forward;
			for (int i = 0; i < wheels.Length; i++)
			{
				if(wheels[i].isFront)
				{
					Vector3 newPos = wheels[i].wheelPivot.localPosition.x /* 0.99f*/ * side + wheels[i].wheelPivot.localPosition.y * up + wheels[i].wheelPivot.localPosition.z /* 0.99f*/ * fwd;
					wheels[i].stopPosWorld = lastChassisPos + newPos;// FlattenLocalYPosition(wheels[i].wheelPivot, lastChassisPos + newPos);
				}
				else
				{
					Vector3 newPos = wheels[i].wheelPivot.localPosition.x /* 1.01f*/ * side + wheels[i].wheelPivot.localPosition.y * up + wheels[i].wheelPivot.localPosition.z /* 0.99f*/ * fwd;
					wheels[i].stopPosWorld = lastChassisPos + newPos;// FlattenLocalYPosition(wheels[i].wheelPivot, lastChassisPos + newPos);
				}
				

				//HelperMath.DebugDrawCross(rBody.position + newPos, 0.25f, Color.magenta, Time.fixedDeltaTime);
			}
		}
	}

	void ShiftWorldPoses(int i, Vector3 newPos)
	{
		float distanceToLastPos = Vector3.Distance(wheels[i].wheelPivot.position, wheels[i].lastWorldPos);
		float trailDist = Mathf.Max(tSlipTrail, distanceToLastPos);

		wheels[i].lastWorldPoses[0] = newPos;

		Vector3 dir01 = (wheels[i].lastWorldPoses[0] - wheels[i].lastWorldPoses[1]).normalized;
		dir01 = Vector3.Slerp(dir01, -transform.up, wantToMove ? Time.fixedDeltaTime * 5 : 0);
		Vector3 pos1 = wheels[i].lastWorldPoses[0] + -dir01 * trailDist * 0.33f;
		wheels[i].lastWorldPoses[1] = pos1;

		Vector3 dir12 = (wheels[i].lastWorldPoses[1] - wheels[i].lastWorldPoses[2]).normalized;
		dir12 = Vector3.Slerp(dir12, -transform.up, wantToMove ? Time.fixedDeltaTime * 5 : 0);
		Vector3 pos2 = wheels[i].lastWorldPoses[1] + -dir12 * trailDist * 0.33f;
		wheels[i].lastWorldPoses[2] = pos2;

		Vector3 dir23 = (wheels[i].lastWorldPoses[2] - wheels[i].lastWorldPoses[3]).normalized;
		dir23 = Vector3.Slerp(dir23, -transform.up, wantToMove ? Time.fixedDeltaTime * 5 : 0);
		Vector3 pos3 = wheels[i].lastWorldPoses[2] + -dir23 * trailDist * 0.33f;
		wheels[i].lastWorldPoses[3] = pos3;


		//wheels[i].lastWorldPoses[0] = FlattenLocalYPosition(wheels[i].wheelCollider, wheels[i].lastWorldPoses[0]);
		//wheels[i].lastWorldPoses[1] = FlattenLocalYPosition(wheels[i].wheelCollider, wheels[i].lastWorldPoses[1]);
		//wheels[i].lastWorldPoses[2] = FlattenLocalYPosition(wheels[i].wheelCollider, wheels[i].lastWorldPoses[2]);
		//wheels[i].lastWorldPoses[3] = FlattenLocalYPosition(wheels[i].wheelCollider, wheels[i].lastWorldPoses[3]);


		//HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[0], 0.03f, Color.yellow, 0);
		//HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[1], 0.03f, Color.Lerp(Color.yellow, Color.red, 0.333f), 0);
		//HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[2], 0.03f, Color.Lerp(Color.yellow, Color.red, 0.666f), 0);
		//HelperMath.DebugDrawCross(wheels[i].lastWorldPoses[3], 0.03f, Color.red, 0);
	}

	void RaycastWheels()
	{
		float numGroundedWheels = 0;
		for (int i = 0; i < wheels.Length; i++)
		{
			RaycastHit[] rHits = Physics.RaycastAll(wheels[i].wheelCastPoint.position + wheels[i].wheelCastPoint.up * sVerticalOffset, -rBody.transform.up, sMaxDistance);

			if(rHits.Length == 0)
			{
				//Debug.DrawRay(wheels[i].wheelCollider.position, -wheels[i].wheelCollider.up * sMaxDistance, Color.red, 0, false);

				wheels[i].isGrounded = false;
				wheels[i].rHit.distance = sMaxDistance;
				continue;
			}

			wheels[i].rHit.distance = -1;
			for (int j = 0; j < rHits.Length; j++)
			{
				if (rHits[j].collider != chassisCollider && !rHits[j].collider.isTrigger)
				{
					if(wheels[i].rHit.distance == -1)
					{
						wheels[i].rHit = rHits[j];
					}
					else if (rHits[j].distance < wheels[i].rHit.distance)
					{
						wheels[i].rHit = rHits[j];
					}
				}
			}

			wheels[i].isGrounded = wheels[i].rHit.distance != -1;

			if (wheels[i].isGrounded)
			{
				numGroundedWheels++;
				//Debug.DrawLine(wheels[i].rHit.point, wheels[i].wheelCollider.position, Color.green, 0, false);
			}

		}

		isGrounded = numGroundedWheels > Mathf.CeilToInt(wheels.Length * 0.5f);
	}

	

	float CalculateGearing(float velocityLocal, float velocityMagnitude)
	{
		velKmh = velocityMagnitude * 3.6f;

		float gearSplit = 1f / (mGears);
		float speedSplit = gearSplit * mTopSpeedKmh;

		//float speedFrac = (velKmh % speedSplit) / speedSplit;
		speedPerc = velKmh / speedSplit;

		int speedPercGear = Mathf.CeilToInt(speedPerc);

		float absMotorInput = Mathf.Abs(motorInput);

		float shortShiftMult = Mathf.Lerp(mGearDownshiftThreshold, mGearUpshiftThreshold, absMotorInput);

		upShiftPoint = currentGear - (shortShiftMult);
		dnShiftPoint = currentGear - 1 - (mGearDownshiftThreshold);

		if (isShifting)
		{
			if(gearTimer > 0)
			{
				gearTimer -= Time.fixedDeltaTime;
				return 0;
			}
			else
			{
				isShifting = false;

				currentGear = targetGear;
			}
		}
		else if(!isDrifting)
		{
			if(velocity.w <= cFreeVelocity)
			{
				if(motorInput != 0)
				{
					currentGear = (int)Mathf.Sign(motorInput);
					targetGear = currentGear;
				}
			}
			else if(velocity.z < -cFreeVelocity && currentGear != -1)
			{
				isShifting = true;
				gearTimer = mShiftTime;

				targetGear = -1;
			}
			else if(velocity.z > cFreeVelocity)
			{
				if(speedPerc < dnShiftPoint)
				{
					targetGear = speedPercGear;
				}
				else if(speedPerc >= upShiftPoint)
				{
					targetGear = Mathf.Min(speedPercGear + 1, mGears);
				}

				if (targetGear != currentGear)
				{
					isShifting = true;
					gearTimer = mShiftTime;

					return 0;
				}


			}

		}

		curTopSpeed = (currentGear > 0 ? (mTopSpeedKmh * ((float)currentGear / mGears)) : mReverseKmh);
		float accelRatio = 1 - ((1f / mGears) * (currentGear > 0 ? currentGear - 1 : 0));
		//Debug.Log( 1 - ((1f / mGears) * (currentGear > 0 ? currentGear - 1 : 0)) );

		float speedMult = velKmh / curTopSpeed;
		speedMult = speedMult > 1 ? 0 : accelRatio;
		//speedMult = 1 - Mathf.Pow(speedMult, 2);
		//speedMult = Mathf.Lerp(1, 1f / (mGears + 1), speedMult);

		return speedMult;
	}

	float CalculateSlopeCompensation(float accMult)
	{
		Vector3 averageNormal = Vector3.zero;

		for (int i = 0; i < wheels.Length; i++)
		{
			averageNormal += wheels[i].forwardProjected;
		}
		averageNormal.Normalize();

		float dot = Vector3.Dot(currentGear > 0 ? Vector3.up : -Vector3.up, averageNormal);
		float powMult = 1 + Mathf.Max(0, dot) * mTorqueAmount;

		slopeAccelerationMult = powMult * Mathf.Abs(motorInput);

		return accMult * powMult;
	}


	void RunSuspension(int i, float gripBias, float rollHeight, float wheelMult)
	{
		////USEFUL VALUES////

		////0-1 is extended, 1-2 is compressed
		float springLerp02 = 0;
		if (wheels[i].rHit.distance > sRestDistance)
		{
			springLerp02 = Mathf.InverseLerp(sMaxDistance, sRestDistance, wheels[i].rHit.distance);
			//gripMult = Mathf.Lerp(0, 1, springLerp02);

		}
		else
		{
			if(wheels[i].rHit.distance < sUpperLimit)
			{
				//float upperLimitMult = sUpperLimit / sMaxDistance;

				springLerp02 = Mathf.InverseLerp(sUpperLimit, 0, wheels[i].rHit.distance);
				springLerp02 = 1 + springLerp02 + Mathf.InverseLerp(sRestDistance, 0, wheels[i].rHit.distance);
				springLerp02 = Mathf.Clamp(springLerp02, 0, 2);
			}
			else
			{
				springLerp02 = 1 + Mathf.InverseLerp(sRestDistance, 0, wheels[i].rHit.distance);
			}
			//gripMult = Mathf.Lerp(1, (sSpring / (wheels.Length * 0.5f)) + 1, springLerp02 - 1);
		}
		wheels[i].springLerp02 = springLerp02;
		//Debug.DrawRay(wheels[i].rHit.point + Vector3.forward * 0.1f, Vector3.up * springLerp02, Color.green, 0, false);


		////CALCULATE SPRING FORCE////

		//float springMult = Mathf.Clamp01(wheels[i].rHit.distance / sMaxDistance);
		float springForce = 0;// springMult * 10 * sSpring;
		//if(wheels[i].rHit.distance <= sUpperLimit)
		//{
		//	springForce = sSpring * (1 - (sUpperLimit / sMaxDistance));
		//}
		if (springLerp02 >= 1)
		{
			springForce = Mathf.Lerp(1, sSpring, springLerp02 - 1);
		}
		else
		{
			springForce = Mathf.Lerp(0, 1, springLerp02);
		}
		float gripSpringForce = springForce;
		springForce = (springForce * 10) * wheelMult;

		float antirollForce = 0;
		float barBend = Mathf.Clamp(-(wheels[i].rHit.distance - wheels[wheels[i].twin].rHit.distance) / sMaxDistance, -1, 1);
		if (i > 9) Debug.Log("Aw shiet it is out of range!");
		if (wheels[i].isFront)
		{
			
			antirollForce = barBend * sAntirollFront;
		}
		else
		{
			antirollForce = barBend * sAntirollRear;
		}
		//Debug.DrawRay(wheels[i].rHit.point, wheels[i].rHit.normal * barBend, Color.green, 0, false);

		float wheelVelocity = -((wheels[i].rHit.distance - wheels[i].prevSuspDist) / sMaxDistance) / Time.fixedDeltaTime;
		wheelVelocity *= wheelMult;
		float springDamping = wheelVelocity * (wheelVelocity < 0 ? sReboundDamping : sCompressionDamping) * sSpring;

		float springResult = Mathf.Max(0, springForce + antirollForce * sSpring + springDamping);

		Vector3 forceDir = (wheels[i].rHit.normal + wheels[i].wheelPivot.up * Mathf.Max(springLerp02, 1)).normalized;

		Vector3 localNormal = wheels[i].wheelPivot.InverseTransformDirection(wheels[i].rHit.normal);
		float camberAngle = Mathf.Atan2(localNormal.x, localNormal.y) * Mathf.Rad2Deg;
		float rHeight = rollHeight * Mathf.Clamp01(Mathf.InverseLerp(90, tPeakAngle, Mathf.Abs(camberAngle)));
		//Debug.DrawRay(wheels[i].rHit.point, wheels[i].rHit.normal * springResult, Color.blue, 0, false);
		rBody.AddForceAtPosition(forceDir * springResult, wheels[i].rHit.point + wheels[i].wheelPivot.up * rHeight, ForceMode.Acceleration);

		wheels[i].camberAngle = camberAngle;
		wheels[i].suspensionCompressionForce = Mathf.Max(0, springDamping);
		wheels[i].debugSpringForce = forceDir * springForce;
		wheels[i].debugDampingForce = springDamping;
		wheels[i].debugRollbarForce = antirollForce *sSpring;

		////CALCULATE GRIP LEVEL////
		float gripMult = 0;

		float gripExtended = sRestDistance / sMaxDistance;
		float gripCompressed = sSpring;// * (1 + (1 - gripExtended));


		float antirollCompMult = wheels[wheels[i].twin].rHit.distance == sMaxDistance ? 1.0f : 0.5f;
		float gripSpringMult = Mathf.Clamp((gripSpringForce + antirollForce * antirollCompMult) / gripSpringForce, 0, 2);
		//if (gripSpringMult < 1)
		//{
			//gripSpringMult = Mathf.Lerp(1, gripSpringMult, (wheels[i].isFront ? sAntirollFront : sAntirollRear) / sSpring);
		//}

		//if (i == 2) Debug.Log(gripSpringMult); //Debug.Log(((gripSpringMult * 0.5f) + 0.5f));
		
		if (springLerp02 > 1)
		{
			//if (wheels[i].rHit.distance < sUpperLimit)
			//{
			//	gripMult = gripCompressed;
			//}
			//else
			{
				gripMult = Mathf.Lerp(1, gripCompressed, springLerp02 - 1);
			}

		}
		else
		{
			gripMult = Mathf.Lerp(gripExtended, 1, springLerp02);
		}

		gripMult = Mathf.Clamp(gripMult * gripSpringMult, 0, gripCompressed);

		//gripMult = springLerp02;
		wheels[i].gripMult = gripMult * gripBias;

		if (wheels[i].gripMult <= 0)
		{
			wheels[i].isGrounded = false;
		}

		if(!wheels[i].isGrounded)
		{
			wheels[i].gripMult = 0;
			gripMult = 0;
		}

		////DEBUGGING////

		wheels[i].debugGripMult = gripMult;
		//Color gripColor = Color.magenta;
		//if (gripMult >= 1)
		//{
		//	gripColor = Color.Lerp(Color.black, Color.white, Mathf.Clamp01(gripMult - 1));
		//}
		//else
		//{
		//	gripColor = Color.Lerp(Color.red, Color.black, Mathf.Clamp01(gripMult));
		//}
		//Debug.DrawRay(wheels[i].rHit.point, wheels[i].rHit.normal * gripMult, gripColor, 0, false);
	}


	void CalculateFrictionVectors(int i)
	{
		////WHEEL TRAIL////

		Vector3 vel01 = wheels[i].lastWorldPoses[0] - wheels[i].lastWorldPoses[1];
		Vector3 vel12 = wheels[i].lastWorldPoses[1] - wheels[i].lastWorldPoses[2];
		Vector3 vel23 = wheels[i].lastWorldPoses[2] - wheels[i].lastWorldPoses[3];

		Vector3 avgVel = (vel01 + vel12 + vel23);

		////TRAILING STOP POSITION////

		Vector3 pointVel = rBody.GetPointVelocity(wheels[i].rHit.point);

		//Vector3 avgPos = (wheels[i].lastWorldPoses[1] + wheels[i].lastWorldPoses[2] + wheels[i].lastWorldPoses[3]) * 0.33333f;
		//Vector3 flatStopPos = FlattenLocalYPosition(wheels[i].wheelPivot, wheels[i].stopPosWorld);
		//Vector3 stopDir = Vector3.ProjectOnPlane((wheels[i].wheelPivot.position - flatStopPos), wheels[i].rHit.normal).normalized;
		float totalWheelMult = (1f / wheels.Length);
		Vector3 stopDir = (wheels[i].wheelPivot.position - wheels[i].stopPosWorld).normalized;// + Vector3.ClampMagnitude(pointVel * 1, 0.05f);

		float stopPosDist = Vector3.Distance(wheels[i].wheelPivot.position, FlattenLocalYPosition(wheels[i].wheelPivot, wheels[i].stopPosWorld));//Vector3.Distance(Vector3.zero, wheels[i].wheelCollider.InverseTransformPoint(flatStopPos));
		float springDamp = -(Mathf.Clamp01(wheels[i].lastStopPosDist / tStopPositionRadius) - Mathf.Clamp01(stopPosDist / tStopPositionRadius));
		springDamp = Mathf.Clamp(springDamp * tStopDamping, -2, 2);
		wheels[i].lastStopPosDist = stopPosDist;

		float avgStopMult = Mathf.Clamp01(stopPosDist / (tStopSpringRadius));
		Vector3 finalStopVector = stopDir * Mathf.Clamp(avgStopMult + springDamp, -1, 1);
		finalStopVector = Vector3.ClampMagnitude(finalStopVector + Vector3.ClampMagnitude(pointVel * tVelocityDamping, totalWheelMult * tVelocityDamping), 1);

		Vector3 finalVel = Vector3.Slerp(avgVel, pointVel, 1 - stationaryMult);

		wheels[i].stopPosVector = FlattenLocalYVector(wheels[i].wheelPivot, finalStopVector);
		wheels[i].posVelocityWorld = FlattenLocalYVector(wheels[i].wheelPivot, finalVel);

		wheels[i].forwardProjected = Vector3.ProjectOnPlane(wheels[i].wheelPivot.forward, wheels[i].rHit.normal).normalized;
		wheels[i].rightProjected = Vector3.ProjectOnPlane(wheels[i].wheelPivot.right, wheels[i].rHit.normal).normalized;

		if (debugFriction)
		{
			Debug.DrawRay(wheels[i].wheelPivot.position, wheels[i].posVelocityWorld.normalized, new Color(0, 1, 0, 0.1f), 0, false);
		}
	}

	Vector2 CalculateDriftAmount(int index, float velDir, float driftAngle, float driftSlip, float wheelForce, float friction)
	{
		/*
		float usePredict = Mathf.Clamp01((longForce / friction));
		float driftAngleTarget = Mathf.Max(0, driftAngle, tPeakAngle * (tStaticFriction / mAcceleration));
		driftAngleTarget = Mathf.Lerp(90, (90 - tPeakAngle) * 0.5f, usePredict);
		driftAngle = Mathf.Lerp(90 - tPeakAngle, 0, usePredict);

		driftUse = (Mathf.Abs(gripSlipAngle) - driftAngle) / Mathf.Clamp(driftAngleTarget, 0, 90);
		driftUse = Mathf.Clamp01(Mathf.InverseLerp(driftAngle, driftAngleTarget, Mathf.Abs(gripSlipAngle)));
		*/

		float longUse = (wheelForce / friction);
		float longUse01 = Mathf.Clamp01(longUse * 0.5f);

		//float latSlipMult = 1f / latSlip;
		//latSlipMult = Mathf.Lerp(1, latSlipMult, tDriftAbility);

		float latSlipMult = Mathf.Clamp(driftSlip - 1, 0, tDriftAbility * Mathf.Clamp01(longUse));


		float usePredict = Mathf.Clamp01(longUse);
		//usePredict = Mathf.Clamp(usePredict, 0, tDriftAbility);
		float driftAngleOuter = Mathf.Lerp(90, (90 - tPeakAngle) * 0.5f, usePredict);
		//driftAngleOuter = Mathf.Lerp(driftAngleOuter, tPeakAngle * 2, Mathf.Clamp01(usePredict - 1));
		float driftAngleInner = Mathf.Lerp((90 - tPeakAngle) * 0.5f, 0, usePredict);

		driftAngleOuter = Mathf.Lerp(90, tPeakAngle, usePredict);
		driftAngleInner = Mathf.Lerp(tPeakAngle, 0, usePredict);

		

		float dirSign = Mathf.Sign(currentGear);
		bool isReverseDrifting = (velDir == dirSign);
		float angle = (isReverseDrifting ? driftAngle : 90) * (1 - stationaryMult);

		//driftUse = (Mathf.Abs(driftAngle) - driftAngleInner) / Mathf.Clamp(driftAngleOuter, 0, 90);
		float driftUse = Mathf.Clamp01(Mathf.InverseLerp(driftAngleInner, driftAngleOuter, Mathf.Abs(angle) ));

		
		////SIMPLIFIED METHOD based on lateral tyre velocity. In combination with slip angle friction offers loose and very predictable handling.
		////Notes: Makes for slightly less sensitive handling that still offers easily predictable drifting.

		{
			float speedMult = 1 - Mathf.Clamp01(velKmh / curTopSpeed);

			float driftInput = Mathf.Clamp01(driftSlip) * usePredict * tDriftAbility * speedMult;

			driftUse = Mathf.Clamp(driftInput, 0, tDriftAbility);
		}
		

		////OLD METHOD based on wheelspin RPM.
		////Notes: Good for very drifty handling. Bad for feeling consistent, as it's prone to wheelspin spikes.


		/*
		{
			float oldDriftUse = Mathf.Clamp(driftUse * usePredict, 0, tDriftAbility);

			//driftUse = Mathf.Pow(driftUse, 1) * usePredict;
			driftUse = Mathf.Max(driftUse, latSlipMult);
			driftUse = Mathf.Clamp(driftUse, 0, longUse01);// longUse * tDriftAbility * Mathf.Clamp01(Mathf.Ceil(latSlip - 1)));
			driftUse = Mathf.Clamp01(Mathf.Max(driftUse, longUse - 1));

			float wheelSpinSpeed = Mathf.Lerp(velKmh, curTopSpeed, driftUse);
			float spinAmount = wheelSpinSpeed / velKmh;
			spinAmount = Mathf.Lerp(1, spinAmount, tDriftAbility);

			//driftUse = Mathf.Clamp(spinAmount - 1, 0, tDriftAbility * 0.5f);
			//driftUse = Mathf.Max(driftUse, longUse - 1);

			spinAmount = Mathf.Max(spinAmount, 1);
			spinAmount = 1 - (1f / spinAmount);
			driftUse = Mathf.Clamp(spinAmount, 0, accelMult * Mathf.Clamp01(tDriftAbility * 2) );

			//driftUse = (driftUse + oldDriftUse) * 0.5f;
		}
		*/

		if (debugDriftAngle)
		{
			//DEBUGGING DRIFT ANGLE
			Color invis = new Color(1, 0, 0, 0.5f);
			HelperMath.DebugDrawCone(wheels[index].wheelPivot, Vector3.zero, driftAngleInner, 0.25f, Color.Lerp(invis, Color.red, usePredict), 0);
			HelperMath.DebugDrawCone(wheels[index].wheelPivot, Vector3.zero, driftAngleOuter, 0.35f, Color.Lerp(invis, Color.red, driftUse * usePredict), 0);
		}


		return new Vector2(driftUse, Mathf.Clamp01(longUse - 1));
	}

	void RunFriction(int i, float accelForce, float brakeForce, float rollHeight, float wheelMult)
	{
		if (!wheels[i].isGrounded)
		{
			if(!wheels[i].isFront && !wheels[i].isPassive && handbrakeInput)
			{
				wheels[i].isLocked = true;
			}
			else
			{
				wheels[i].isLocked = brakeForce > Mathf.Abs(accelForce) && brakeForce > 0;
			}

			

			wheels[i].slipAngle = 0;
			wheels[i].spinAmount = Mathf.Abs(motorInput);

			return;
		}

		//Vector3 velocity = rBody.GetPointVelocity(wheels[i].rHit.point);
		//Vector3 localVelocity = wheels[i].wheelCollider.InverseTransformDirection(velocity).normalized;

		Vector3 posVelocityLocal = wheels[i].wheelPivot.InverseTransformDirection(wheels[i].posVelocityWorld);
		Vector3 vel = wheels[i].wheelPivot.InverseTransformDirection(rBody.GetPointVelocity(wheels[i].rHit.point));

		//Vector3 localNormal = wheels[i].wheelPivot.InverseTransformDirection(wheels[i].rHit.normal);
		//float camberAngle = Mathf.Atan2(localNormal.x, localNormal.y) * Mathf.Rad2Deg;
		float camberFrictionMult = Mathf.Clamp01(Mathf.InverseLerp(tPeakAngle, 90, Mathf.Abs(wheels[i].camberAngle)));
		//camberFrictionMult = Mathf.Pow(camberFrictionMult, 0.5f);
		//camberFrictionMult *= Mathf.Pow(Mathf.Min(1, 1f / wheels[i].gripMult), 2);
		camberFrictionMult = Mathf.Pow(camberFrictionMult, 0.75f);

		float magClamp = Mathf.Max(Mathf.Sqrt(vel.x * vel.x + vel.z * vel.z), 1);
		float friction = Mathf.Lerp(wheels[i].gripMult * tMaxGrip, tCamberGrip, camberFrictionMult);

		wheels[i].gripMult = Mathf.Lerp(wheels[i].gripMult, 0, camberFrictionMult);

		bool isLocked = brakeForce > friction;
		bool absLockCorrect = cHasABS ? (wheels[i].isLeft ? absLock : !absLock) : false;

		float newBrakeForce = brakeForce;
		if(isLocked && absLockCorrect)
		{
			newBrakeForce = friction * 0.5f;
			isLocked = false;
		}

		if(handbrakeInput && !wheels[i].isFront && !wheels[i].isPassive)
		{
			isLocked = true;
		}

		float totLongForce = Mathf.Abs(accelForce) - newBrakeForce;
		float wheelForce = 0;
		float driftForce = 0;
		if(totLongForce <= 0)
		{
			wheelForce = Mathf.Abs(totLongForce) * -Mathf.Sign(vel.z);
		}
		else
		{
			wheelForce = totLongForce * Mathf.Sign(accelForce);
			driftForce = totLongForce;
		}

		//Vector3 localLastWorldDir = wheels[i].wheelPivot.InverseTransformDirection(wheels[i].lastWorldDir);
		//float worldDirAngle = Mathf.Atan2(localLastWorldDir.x, Mathf.Abs(localLastWorldDir.z)) * Mathf.Rad2Deg;
		//float worldVelPosDistPercent = Mathf.Floor(1 - Mathf.Clamp01(Mathf.Abs(posVelocityLocal.z) / tSlipTrail));
		float frictionWheelMulted = (friction * 10 * wheelMult);


		float lateralSlip = Mathf.Max(0, Mathf.Abs(vel.x) / frictionWheelMulted);
		lateralSlip = Mathf.Max(0, Mathf.Abs(vel.x) / friction);

		float slipAngleClamp = 1f / Mathf.Min(wheels[i].gripMult, 1);

		float slipAngle = Mathf.Atan2(-vel.x, Mathf.Abs(vel.z)) * Mathf.Rad2Deg;
		float steerForce = Mathf.Clamp((slipAngle * slipAngleClamp) / tPeakAngle, -1, 1);
		steerForce = Mathf.Lerp(steerForce, Mathf.Clamp(-vel.x / friction, -1, 1), stationaryMult);

		//TODO: Maybe introduce friction curve?
		

		float angleTo90 = Mathf.Clamp01(Mathf.Abs(slipAngle) / 90);

		wheels[i].slipAngle = slipAngle * (1 - stationaryMult);

		float driftSlip = Mathf.Clamp01(Mathf.Max( (Mathf.Sign(currentGear) * -vel.z) / frictionWheelMulted - 1, lateralSlip));
		Vector2 driftResult = CalculateDriftAmount(i, Mathf.Sign(vel.z), slipAngle, driftSlip, driftForce, friction);
		float driftUse = driftResult.x;

		float spin = tDriftAbility > 0 ? Mathf.Clamp01(driftUse / tDriftAbility) : 0;
		spin = Mathf.Max(spin, driftResult.y);

		wheelForce = Mathf.Lerp(wheelForce, Mathf.Sign(wheelForce) * friction, driftUse);

		wheels[i].spinAmount = spin * Mathf.Sign(accelForce);
		wheels[i].isLocked = isLocked;
		
		float angleSlip = Mathf.Clamp01(Mathf.InverseLerp(tPeakAngle, 90, Mathf.Abs(wheels[i].slipAngle)));

		wheels[i].wheelSlide = new Vector2(Mathf.Max(driftResult.x, angleSlip), driftResult.y);
		

		float lateralMult = 1 - driftUse;

		float counterSteerMult = 1; //wheels[i].canSteer && wheels[i].isFront ? Mathf.Lerp(1, 0.5f, counterAmount) : 1;

		//Vector3 steerForceVector = Vector3.ProjectOnPlane(wheels[i].wheelPivot.right, wheels[i].rHit.normal).normalized;
		float steerForceMagnitude = Mathf.Clamp(steerForce, -lateralMult, lateralMult) * Mathf.Min(friction, magClamp) * counterSteerMult;
		Vector3 finalSteerForce = wheels[i].rightProjected * steerForceMagnitude;

		wheelForce *= 1 - ( (Mathf.Abs(slipAngle) / 90) * (1 - Mathf.Max(driftUse, mTorqueAmount)) );

		Vector3 longitudinalForce = wheels[i].forwardProjected;
		float longForceMagnitude = Mathf.Clamp(wheelForce * (1 + driftUse), -friction * (1 + angleTo90), friction * (1 + angleTo90));
		longitudinalForce *= longForceMagnitude;

		//finalForce = Vector3.ClampMagnitude(lateralForce * driftMod + longitudinalForce, friction * (2 - driftUse));

		Vector3 finalStopForce = -wheels[i].stopPosVector * Mathf.Min(friction, magClamp);

		Vector3 finalForce = Vector3.ClampMagnitude(finalSteerForce, magClamp) + longitudinalForce;
		if(isLocked)
		{
			//Vector3.LerpUnclamped(finalSteerForce, finalStopForce, stationaryMult)
			//if(handbrakeInput && !wheels[i].isFront && stationaryMult == 0)
			{
				Vector3 pointVel = -rBody.GetPointVelocity(wheels[i].wheelCastPoint.position);
				pointVel = Vector3.ProjectOnPlane(pointVel, wheels[i].rHit.normal).normalized * Mathf.Min(friction, magClamp);

				finalStopForce = Vector3.Slerp(pointVel, finalStopForce, stationaryMult);
				//finalStopForce = Vector3.ClampMagnitude(finalPointVel, Mathf.Min(friction, magClamp));
			}
			finalForce = finalStopForce;
			driftUse = 0;
		}
		//else if(wantToMove)
		//{
		//	Vector3 newfinalStopForce = wheels[i].wheelPivot.InverseTransformVector(finalStopForce);
		//	newfinalStopForce = new Vector3(newfinalStopForce.x, 0, longForceMagnitude);
		//	newfinalStopForce = wheels[i].wheelPivot.TransformVector(newfinalStopForce);

		//	finalForce = Vector3.LerpUnclamped(finalForce, newfinalStopForce, stationaryMult);
		//}
		else if(!wantToMove)
		{
			finalForce = Vector3.LerpUnclamped(finalForce, finalStopForce, stationaryMult);
		}

		//finalForce *= (tMaxGrip * wheels[i].gripMult * 10) / wheels.Length;
		finalForce = Vector3.ClampMagnitude(finalForce, friction * 2);
		finalForce *= 10f * wheelMult;

		if (debugFriction)
		{
			//DEBUG FRICTION
			Color debugFrictionColor = Color.Lerp(new Color(1, 1, 1, 0.5f), new Color(1, 0, 0, 0.5f), driftUse);
			if (isLocked) debugFrictionColor = new Color(0, 0.5f, 1, 0.5f);

			Debug.DrawRay(wheels[i].rHit.point, finalForce, debugFrictionColor, 0, false);
		}

		Vector3 localFriction = wheels[i].wheelCastPoint.InverseTransformVector(finalForce);
		//wheels[i].rHit.point + transform.up * rollHeight
		Vector3 forcePoint = wheels[i].wheelCastPoint.position + -wheels[i].wheelCastPoint.up * Mathf.Max(-centreOfMass.y, sRestDistance - rollHeight - sVerticalOffset);

		rBody.AddForceAtPosition(finalForce, forcePoint, ForceMode.Acceleration);
		

		wheels[i].lastWorldDir = wheels[i].wheelPivot.forward;

		////OBJECT REACTION FORCE////
		if(wheels[i].rHit.rigidbody)
		{
			float forceFric = 1;
			if (wheels[i].rHit.collider.sharedMaterial)
			{
				forceFric = Mathf.Lerp(wheels[i].rHit.collider.sharedMaterial.staticFriction, wheels[i].rHit.collider.sharedMaterial.dynamicFriction, driftUse);
			}

			Vector3 reactForce = wheels[i].wheelPivot.InverseTransformDirection(finalForce);
			reactForce.x = 0;
			reactForce = wheels[i].wheelPivot.TransformDirection(reactForce);

			//wheels[i].rHit.rigidbody.AddForceAtPosition(-reactForce * (wheels[i].rHit.rigidbody.mass / rBody.mass), wheels[i].rHit.point, ForceMode.Acceleration)
			wheels[i].rHit.rigidbody.AddForceAtPosition(Vector3.ClampMagnitude(-finalForce, 1) * forceFric * Mathf.Min(10, rBody.mass / wheels[i].rHit.rigidbody.mass), wheels[i].rHit.point, ForceMode.Acceleration);
		}
		//rBody.AddForce(transform.forward * (forwardInput - reverseInput) * 10, ForceMode.Acceleration);
	}


	Vector3 FlattenLocalYPosition(Transform trans, Vector3 pos)
	{
		Vector3 local = trans.InverseTransformPoint(pos);
		local.y = 0;

		Vector3 result = trans.TransformPoint(local);

		return result;
	}

	Vector3 FlattenLocalYVector(Transform trans, Vector3 vec)
	{
		Vector3 local = trans.InverseTransformDirection(vec);
		local.y = 0;

		Vector3 result = trans.TransformDirection(local);

		return result;
	}

	///<summary>Returns the number furthest from 0. </summary>
	float AbsoluteLargest(float a, float b)
	{
		float absA = Mathf.Abs(a);
		float absB = Mathf.Abs(b);

		return absA > absB ? a : b;

	}
}
