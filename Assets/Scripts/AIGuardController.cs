using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIGuardController : MonoBehaviour {


	public Transform [] resources; 

	public GameObject [] prisoners; 

	public Transform[] walkingPoints;

	public GameObject weapon;

//	public Transform [] waypointSetC; 
//
//	public Transform waypointE;
//
//	public GameObject [] prisoners;

	private bool armed;

	private bool isWalking;

	private GameObject closestPrisoner;



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
		closestPrisoner = prisoner;
		state = State.CHASEPRISONER;
	}

	void transitionToThrowandChase(GameObject prisoner) {
		if (armed) {
			shootBullet ();
		}
		closestPrisoner = prisoner;
	}

	void transitionToGetAmmo () {
		print ("Transitioning to get Ammo");
		state = State.GETAMMO;

		Transform ammo = findClosestAmmo ();

		aiSteer.setWayPoint (ammo);
		aiSteer.useNavMeshPathPlanning = true;
	}



		

	// Update is called once per frame
	void Update () {
		
		switch (state)
		{

		case State.WALKAROUND:
			closestPrisoner = findClosestPrisoner ();
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
				addAmmo ();
				transitionToWalkingState ();
			}
			break;

		case State.CHASEPRISONER:
			if (closestPrisoner.GetComponent<AIPrisonerController>().isSafe()) {
				closestPrisoner = null;
				shootBullet ();
				state = State.WALKAROUND;
			} else {
				GameObject nearPrisoner = findClosestPrisoner ();
				if (nearPrisoner != closestPrisoner) {
					transitionToThrowandChase (nearPrisoner);
				}
				aiSteer.setWayPoint (closestPrisoner.transform);
			}

			break;
	

		default:

			print("Weird?");
			break;
		}


	}

	void OnTriggerEnter(Collider other) {
		if (other.transform.tag.Equals("Prisoner")) {
			print ("Prisoner has been tagged");
			state = State.WALKAROUND;

			AIPrisonerController prisonScript = other.transform.gameObject
				.GetComponent<AIPrisonerController>();
			prisonScript.hasBeenTagged ();
		} else if (other.transform.tag.Equals("Ammo")) {
			addAmmo ();
		}
	}



	//Calculate the distance between this object and another object
	float calcDistance(Vector3 res) {
		Vector3 tran = transform.position;
		float distance = Mathf.Sqrt (Mathf.Pow (res.x - tran.x, 2) +
			Mathf.Pow (res.y - tran.y, 2) + Mathf.Pow (res.z - tran.z, 2));

		return distance;
	}

	public bool isArmed() {
		return armed;
	}

	private void shootBullet() {
		armed = false;
		weapon.SetActive (false);
	}

	private void addAmmo() {
		armed = true;
		print ("Added Ammo");
		weapon.SetActive (true);
	}

	GameObject findClosestPrisoner() {
		float distance = float.MaxValue;
		GameObject closest = null;
		foreach (GameObject prisoner in prisoners) {
			bool tagged = prisoner.GetComponent<AIPrisonerController> ().isTagged ();
			bool safe = prisoner.GetComponent<AIPrisonerController> ().isSafe ();
			if (!tagged && !safe) {
				float distanceToPrisoner = calcDistance (prisoner.transform.position);
				if (distanceToPrisoner < distance && distanceToPrisoner < 15.0f) {
					distance = distanceToPrisoner;
					closest = prisoner;
				}
			}
		}
		print (closest);
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
