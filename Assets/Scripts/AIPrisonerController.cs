using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIPrisonerController : MonoBehaviour {


	public Transform [] goals; 

	public Transform[] taggedZones;

	public Transform[] safeZones;

	public GameObject[] guards;

//	public Transform [] waypointSetB; 
//
//	public Transform [] waypointSetC; 

//	public Transform waypointE;

//	private bool tagged;

	private Transform destination;

	private Transform oldSafeZone;

	private Transform oldGoal;

	private bool isBeingChased;

	private bool tagged;

	private bool safe;



	public enum State {
		A, TAGGED, TONEWGOAL, SAFEZONE

//		A,B,C,D,E

	}


	public State state = State.A;

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
		transitionToNewGoal();

		oldSafeZone = transform;
		destination = transform;

	}

	void transitionToNewGoal() {
		print ("Transition to Goal");
		state = State.TONEWGOAL;
		oldGoal = findClosestGoal ();

		if (oldGoal == goals[0]) {
			aiSteer.setWayPoint (goals [1]);
		} else {
			aiSteer.setWayPoint (goals [0]);
		}
		aiSteer.useNavMeshPathPlanning = true;
	}

	void transitionToNearestSafeZone() {
		print ("Transition to nearest Safe Zone");
		state = State.SAFEZONE;

		Transform closestZone = findClosestSafeZone ();
		aiSteer.setWayPoint (closestZone);
		aiSteer.useNavMeshPathPlanning = true;
	}


	void transitionToStateA() {

		print("Transition to state A: Get to Destination Zone");

		state = State.A;

		bool dest = false;

		foreach (Transform pos in goals) {
			if (oldSafeZone != pos && checkIfZoneIsReachable(pos)) {
				dest = true;
				destination = pos;
			}
		}

		if (!dest) {
			foreach (Transform pos in safeZones) {
				if(oldSafeZone != pos && checkIfZoneIsReachable(pos)) {
					dest = true;
					destination = pos;
				}
			}
		}
			
		aiSteer.setWayPoint (destination);
		oldSafeZone = destination;
		aiSteer.useNavMeshPathPlanning = true;


//		//flip our destinations
//		if (oldSafeZone == null || oldSafeZone.Equals(waypointSetA[0])) {
//			oldSafeZone = waypointSetA [1];
//		} else {
//			oldSafeZone = waypointSetA [0];
//		}
//		aiSteer.setWayPoint(oldSafeZone);
//		destination = oldSafeZone;
//
//		aiSteer.useNavMeshPathPlanning = true;


	}

	void transitionToStateTAGGED() {
		print ("Transitioning to Tagged State");
		state = State.TAGGED;
		tagged = true;

		Transform closestZone = taggedZones [0];
		float distance = calcDistance (closestZone.position, transform.position);
		for (int i = 1; i < taggedZones.Length; i++) {
			float calc = calcDistance (taggedZones [i].position, transform.position);
			if (calc < distance) {
				closestZone = taggedZones[i];
				distance = calc;
			}
		}

		aiSteer.setWayPoint (closestZone);
	}

//	float calcDistance(Vector3 res) {
//		Vector3 tran = transform.position;
//		float distance = Mathf.Sqrt (Mathf.Pow (res.x - tran.x, 2) +
//			Mathf.Pow (res.y - tran.y, 2) + Mathf.Pow (res.z - tran.z, 2));
//
//		return distance;
//	}

//
//	void transitionToStateB() {
//
//		print("Transition to state B");
//
//		state = State.B;
//
//		aiSteer.setWayPoints(waypointSetB);
//
//		aiSteer.useNavMeshPathPlanning = true;
//	}
//
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
		case State.A:
			if (aiSteer.waypointsComplete ())
				transitionToStateA ();
			break;

		case State.TONEWGOAL:
			float closestGuard = findDistToClosestGuard ();
			if (isBeingChased && closestGuard < 15.0f) {
				transitionToNearestSafeZone ();
			} else if (closestGuard < 10.0f) {
				transitionToNearestSafeZone (); 
			} else if (aiSteer.waypointsComplete()) {
				transitionToNewGoal ();
			}
			break;

		case State.SAFEZONE:
			break;

		case State.TAGGED:
//			transitionToStateTAGGED ();
			break;


		default:

			print("Weird?");
			break;
		}


	}


	void OnTriggerExit(Collider other) {
		if (other.transform.tag.Equals("Safe")) {
			safe = false;
			print ("NotSafe");
		}
	}

	void OnTriggerEnter(Collider other) {
		if (other.transform.tag.Equals("Safe")) {
			print ("Safe");
			safe = true;
		}
	}

	public void hasBeenTagged() {
		state = State.TAGGED;
		transitionToStateTAGGED ();
	}

	public Transform getDestination() {
		return destination;
	}

	//Check if a zone is reachable
	// 1. Either the player can get there first or if guards don't have a throwable
	private bool checkIfZoneIsReachable(Transform zone) {
		bool canMakeIt = true;
		float prisonerDistance = calcDistance (zone.position, transform.position);
		for (int i = 0; i < guards.Length; i++) {
			if (guards[i].GetComponent<AIGuardController>().isArmed()) {
				if (calcDistance(zone.position, guards[i].transform.position) >= prisonerDistance) {
					canMakeIt = false;
				}
			}
		}
		return canMakeIt;
	}

	float calcDistance(Vector3 res, Vector3 tran) {
		float distance = Mathf.Sqrt (Mathf.Pow (res.x - tran.x, 2) +
			Mathf.Pow (res.y - tran.y, 2) + Mathf.Pow (res.z - tran.z, 2));

		return distance;
	}

	//Finds the closest goal to the prisoner
	private Transform findClosestGoal() {
		float goal1 = calcDistance (goals [0].position, transform.position);
		float goal2 = calcDistance (goals [1].position, transform.position);
		if (goal1 < goal2)
			return goals[0];
		return goals[1];
	}

	//Finds the closest safezone to the prisoner
	private Transform findClosestSafeZone() {
		float safeZones1 = calcDistance (safeZones [0].position, transform.position);
		float safeZones2 = calcDistance (safeZones [1].position, transform.position);
		if (safeZones1 < safeZones2)
			return safeZones [0];
		return safeZones[1];
	}

	private float findDistToClosestGuard() {
		float closest = float.MaxValue;
		foreach (GameObject guard in guards) {
			float distanceFromGuard = calcDistance (guard.transform.position, transform.position);
			if (distanceFromGuard < closest) {
				closest = distanceFromGuard;
			}
		}
		return closest;
	}

	public bool isTagged() {
		return tagged;
	}

	public bool isSafe() {
		return safe;
	}
}
