using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIPrisonerController : MonoBehaviour {


	public Transform [] goals; 

	public Transform[] taggedZones;

	public Transform[] safeZones;

	public GameObject[] guards;

	//what is destination
	private Transform destination;

	//what is oldGoal
	private Transform oldGoal;

	private bool tagged;

	private bool safe;

	private bool enteredZone;

	//Time a player waits in a safeZone before running
	private float SafeZoneWaitTime = 2.5f;



	public enum State {
		TAGGED, TONEWGOAL, SAFEZONE
		//Add evade state maybe

	}


	public State state = State.TONEWGOAL;

	public float waitTime = 5f;

	protected float beginWaitTime;


	AINavSteeringController aiSteer;


	// Use this for initialization
	void Start () {

		aiSteer = GetComponent<AINavSteeringController>();

		aiSteer.Init();

		aiSteer.waypointLoop = false;
		aiSteer.stopAtNextWaypoint = false;

		transitionToNewGoal();


		destination = transform;

	}

	//Finds the goal a player should transition to
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

	//finds the closest safezone that a player should transition to
	void transitionToNearestSafeZone() {
		print ("Transition to nearest Safe Zone");
		state = State.SAFEZONE;

		Transform closestZone = findClosestSafeZone ();
		if (closestZone == null) {
			state = State.TONEWGOAL;
			closestZone = findClosestGoal ();
		}
		aiSteer.setWayPoint (closestZone);
		aiSteer.useNavMeshPathPlanning = true;
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



	// Update is called once per frame
	void Update () {

		switch (state)
		{
		//Player should run to the furthest goal unless a guard is near
		case State.TONEWGOAL:
			float closestGuard = findDistToClosestGuard ();
			if (closestGuard < 10.0f) {
				transitionToNearestSafeZone (); 
			} else if (aiSteer.waypointsComplete()) {
				transitionToNewGoal ();
			}
			break;
		
		//Player should run towards the closest safeZone
		case State.SAFEZONE:
			if (aiSteer.waypointsComplete () && !enteredZone) {
				enteredZone = true;
				beginWaitTime = Time.timeSinceLevelLoad;
			}
			if (aiSteer.waypointsComplete() &&  Time.timeSinceLevelLoad - beginWaitTime > SafeZoneWaitTime) {
				transitionToNewGoal ();
				enteredZone = false;
			}
			break;
		
			//player is tagged and out of play
		case State.TAGGED:
			if (aiSteer.waypointsComplete()) {
				GameObject levelManager = GameObject.Find ("LevelManager");
				levelManager.GetComponent<LevelManager> ().addTagged ();
			}
			break;


		default:

			print("Weird?");
			break;
		}


	}

	//On exiting a safe zone, a player is no longer considered safe and can be chased
	void OnTriggerExit(Collider other) {
		if (other.transform.tag.Equals("Safe")) {
			safe = false;
		}
	}

	//When entering a safe zone a player is considered safe and cannot be chased
	void OnTriggerEnter(Collider other) {
		if (other.transform.tag.Equals("Safe")) {
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

//	//Check if a zone is reachable
//	private bool checkIfZoneIsReachable(Transform zone) {
//		bool canMakeIt = true;
//		float prisonerDistance = calcDistance (zone.position, transform.position);
//		for (int i = 0; i < guards.Length; i++) {
//			if (guards[i].GetComponent<AIGuardController>().isArmed()) {
//				if (calcDistance(zone.position, guards[i].transform.position) >= prisonerDistance) {
//					canMakeIt = false;
//				}
//			}
//		}
//		return canMakeIt;
//	}

	//Finds the distance between two objects
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

	//Finds the closest safezone to the prisoner that is not full
	private Transform findClosestSafeZone() {
		//The amount of prisoners currently in a safezone
		//If a safeZone is full, a prisoner will not try to go to it
		bool zone1Full = safeZones [0].GetComponent<SafeZoneController> ().isFull ();
		bool zone2Full = safeZones [1].GetComponent<SafeZoneController> ().isFull ();


		float safeZones1 = calcDistance (safeZones [0].position, transform.position);
		float safeZones2 = calcDistance (safeZones [1].position, transform.position);

		if (safeZones1 < safeZones2) {
			if (zone1Full) {
				if (zone2Full) {
					return null;
				} else {
					return safeZones [1];
				}
			}
			return safeZones [0];
		}

		if (zone2Full) {
			if (zone1Full) {
				return null;
			}
			return safeZones [0];
		}
		return safeZones [1];
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

	//returns if the players is tagged
	public bool isTagged() {
		return tagged;
	}

	//returns if the player is currently safe
	public bool isSafe() {
		return safe;
	}
}
