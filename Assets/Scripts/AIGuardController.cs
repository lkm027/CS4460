using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIGuardController : MonoBehaviour {


	public Transform [] resources; 

	public GameObject [] prisoners; 

	public Transform[] walkingPoints;

//	public Transform [] waypointSetC; 
//
//	public Transform waypointE;
//
//	public GameObject [] prisoners;

	private bool armed;

	private bool isWalking;

	private Transform closestPrisoner;



	public enum State {

		WALKAROUND, CHASEPRISONER, GETAMMO
//		A,B,C,D,E

	}


	public State state = State.WALKAROUND;

	public float waitTime = 5f;

	protected float beginWaitTime;


	AINavSteeringController aiSteer;


	// Use this for initialization
	void Start () {

		aiSteer = GetComponent<AINavSteeringController>();

		aiSteer.Init();

		aiSteer.waypointLoop = false;
		aiSteer.stopAtNextWaypoint = false;

//		transitionToStateA();

		transitionToWalkingState ();

		//guards don't have throwables at start
		armed = false;

	}

	void transitionToWalkingState() {
		state = State.WALKAROUND;
		isWalking = false;
	}

	void transitionToChase (GameObject prisoner) {
		setSpeed (1.0f);
	}

	void transitionToGetAmmo () {
		print ("Transitioning to get Ammo");
		state = State.GETAMMO;

		Transform ammo = findClosestAmmo ();

		aiSteer.setWayPoint (ammo);
		aiSteer.useNavMeshPathPlanning = true;
	}


//	void transitionToStateA() {
//
//		print("Transition to state A");
//
//		state = State.A;
//
//		//calculate the closest resource to the guard
//		Transform closestResource = resources [0];
//		float distance = calcDistance (closestResource.position);
//		for (int i = 1; i < resources.Length; i++) {
//			float calc = calcDistance (resources [i].position);
//			if (calc < distance) {
//				closestResource = resources[i];
//				distance = calc;
//			}
//		}
//
//		aiSteer.setWayPoint (closestResource);
////		aiSteer.setWayPoints(waypointSetA);
//
////		aiSteer.setWayPoint (prisoners [0].transform);
//
//		aiSteer.useNavMeshPathPlanning = true;
//
//
//	}

	float calcDistance(Vector3 res) {
		Vector3 tran = transform.position;
		float distance = Mathf.Sqrt (Mathf.Pow (res.x - tran.x, 2) +
			Mathf.Pow (res.y - tran.y, 2) + Mathf.Pow (res.z - tran.z, 2));

		return distance;
	}




//	void transitionToStateB() {
//
//		print("Transition to state B");
//
//		state = State.B;
//
//		float closestPrisonerDist = calcDistance (prisoners [0].position);
//		closestPrisoner = prisoners [0];
//
//		foreach (Transform prisoner in prisoners) {
//			float distToPrisoner = calcDistance (prisoner.position);
//			if (distToPrisoner < closestPrisonerDist) {
//				closestPrisonerDist = distToPrisoner;
//				closestPrisoner = prisoner;
//			}
//		}
//
//		aiSteer.setWayPoint (closestPrisoner);
////		aiSteer.setWayPoints(waypointSetB);
//
//		aiSteer.useNavMeshPathPlanning = true;
//	}

//
//	void transitionToStateC() {
//
//		print("Transition to state C");
//
//		state = State.C;
//
//		aiSteer.setWayPoints(waypointSetC);
//
//		aiSteer.useNavMeshPathPlanning = false;
//
//	}
//
//	void transitionToStateD() {
//
//		print("Transition to state D");
//
//		state = State.D;
//
//		beginWaitTime = Time.timeSinceLevelLoad;
//
//		aiSteer.clearWaypoints ();
//
//		aiSteer.useNavMeshPathPlanning = true;
//
//	}
//
//
//	void transitionToStateE() {
//
//		print("Transition to state E");
//
//		state = State.E;
//
//		aiSteer.setWayPoint (waypointE);
//
//		aiSteer.useNavMeshPathPlanning = true;
//
//	}

	// Update is called once per frame
	void Update () {
		
		switch (state)
		{

		case State.WALKAROUND:
			GameObject closestPrisoner = findClosestPrisoner ();
			if (closestPrisoner != null) {
				transitionToChase (closestPrisoner);
			} else if (!armed) {
				transitionToGetAmmo ();
			} else if (!isWalking) {
				int random = Random.Range (0, walkingPoints.Length);
				aiSteer.setWayPoint (walkingPoints [random]);
				setSpeed (0.25f);
				isWalking = true;
			}

			if (aiSteer.waypointsComplete()) {
				isWalking = false;
			}
			break;

		case State.GETAMMO:
			if (aiSteer.waypointsComplete ()) {
				armed = true;
				transitionToWalkingState ();
			}
			break;
		//Gather resources
//		case State.A:
//			if (aiSteer.waypointsComplete()) {
//				armed = true;
//				transitionToStateB();
//			}
//				
//			break;
//
//		//Chase Prisoner
//		case State.B:
////			aiSteer.setWayPoint (prisoners [0].transform);
//			//changed the guard's destination to the prisoners destination
//
////			Transform dest = prisoners [0].GetComponent<AIPrisonerController> ().getDestination ();
////			aiSteer.setWayPoint (dest);
//			aiSteer.setWayPoint(closestPrisoner);
//			if (aiSteer.waypointsComplete())
//				transitionToStateA();
//			break;
//
//		case State.C:
//			if (aiSteer.waypointsComplete())
//				transitionToStateD();
//			break;
//
//		case State.D:
//			if (Time.timeSinceLevelLoad - beginWaitTime > waitTime)
//				transitionToStateE();
//			break;
//
//		case State.E:
//			if (aiSteer.waypointsComplete ())
//				break;
//			//					transitionToStateA();
//			break;
		default:

			print("Weird?");
			break;
		}


	}

	void OnTriggerEnter(Collider other) {
		if (other.transform.tag.Equals("Prisoner")) {
			print ("Prisoner has been tagged");
			AIPrisonerController prisonScript = other.transform.gameObject
				.GetComponent<AIPrisonerController>();
			prisonScript.hasBeenTagged ();
		}
	}

	public bool isArmed() {
		return armed;
	}

	GameObject findClosestPrisoner() {
		float distance = float.MaxValue;
		GameObject closest = null;
		foreach (GameObject prisoner in prisoners) {
			float distanceToPrisoner = calcDistance (prisoner.transform.position);
			if (distanceToPrisoner < distance && distanceToPrisoner < 15.0f) {
				distance = distanceToPrisoner;
				closest = prisoner;
			}	
		}
		return closest;
	}

	Transform findClosestAmmo () {
		float distance = float.MaxValue;
		Transform closest = resources[0];
		foreach (Transform ammo in resources) {
			float distanceToAmmo = calcDistance (ammo.position);
			if (distanceToAmmo < distance) {
				closest = ammo;
				distance = distanceToAmmo;
			}
		}
		return closest;
	}

	private void setSpeed(float speed) {
		GetComponent<AINavSteeringController> ().mecanimInputForwardSpeedCap = speed;
	}
}
