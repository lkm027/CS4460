using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIGuardController : MonoBehaviour {


	public Transform [] resources; 

	public GameObject [] prisoners; 

	public Transform[] walkingPoints;

	public GameObject weapon;

	private bool armed;

	private bool isWalking;

	private GameObject closestPrisoner;

	private GameObject bullet;



	public enum State {

		WALKAROUND, CHASEPRISONER, GETAMMO

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

		transitionToWalkingState ();

		//guards don't have throwables at start
		armed = false;

	}

	//Guards should walk from guard post to guard post, unless they do not have ammo, or are near a prisoner
	void transitionToWalkingState() {
		state = State.WALKAROUND;
		isWalking = false;
	}

	//Speed should increase and they should chase
	void transitionToChase (GameObject prisoner) {
		setSpeed (1.0f);
		closestPrisoner = prisoner;
		state = State.CHASEPRISONER;
	}

	//A closer prisoner has been found so they should shoot at the old prisoner and chase the newer one
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
		//Walk from post to post unless they do not have ammo or are near a prisoner
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
			//if the closest prisoner has transition to a safe state then we should shoot and walk around
			if (closestPrisoner != null && closestPrisoner.GetComponent<AIPrisonerController>().isSafe()) {
				shootBullet ();
				closestPrisoner = null;
				state = State.WALKAROUND;
				break;
			} else {
				//if we find a new closer prisoner then we should shoot at the old  
				//one and chase the new closer prisoner
				GameObject nearPrisoner = findClosestPrisoner ();
				if (nearPrisoner != closestPrisoner) {
					transitionToThrowandChase (nearPrisoner);
				}
				//Chase the old/current closest prisoner
				if (closestPrisoner != null)
					aiSteer.setWayPoint (closestPrisoner.transform);
			}

			break;
	

		default:

			print("Weird?");
			break;
		}


	}


	void OnTriggerEnter(Collider other) {
		//Colliding with a prisoner tags the prisoner
		if (other.transform.tag.Equals("Prisoner")) {
			print ("Prisoner has been tagged");
			state = State.WALKAROUND;

			AIPrisonerController prisonScript = other.transform.gameObject
				.GetComponent<AIPrisonerController>();
			prisonScript.hasBeenTagged ();

		//Colliding with ammo reloads the ammo of the guard
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

	//Returns if the guard has ammo
	public bool isArmed() {
		return armed;
	}

	//If the guard has ammo, they shoot a bullet
	private void shootBullet() {
		if (armed) {
			print ("shooting");
			Vector3 pos = transform.position;

			GameObject bull = Instantiate (weapon,new Vector3(pos.x, pos.y + 1.0f, pos.z), Quaternion.identity);
			//
			Vector3 targetVelocity = closestPrisoner.GetComponent<Rigidbody> ().velocity;
			Vector3 targetPosition = closestPrisoner.transform.position;
			Vector3 bulletPosition = new Vector3(pos.x, pos.y + 1.0f, pos.z);
			float time = 1.0f;

			Vector3 bulletVel = targetVelocity + ((targetPosition - bulletPosition) / time);
			bulletVel.y = 0;
			bull.GetComponent<Rigidbody> ().velocity = bulletVel;

			Destroy (bullet);
		}
			
		armed = false;
	}

	//Give the guard ammo
	private void addAmmo() {
		if (!armed) {
			armed = true;
			print ("Added Ammo");
			Vector3 pos = transform.position;
			bullet = Instantiate (weapon, new Vector3(pos.x, pos.y + 5.0f, pos.z), Quaternion.identity);
			bullet.transform.parent = transform;
		}

	}

	//Finds the closest prisoner
	GameObject findClosestPrisoner() {
		float distance = float.MaxValue;
		GameObject closest = null;
		foreach (GameObject prisoner in prisoners) {
			//make sure the prisoner is not tagged or in a safe zone
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
		return closest;
	}

	//Finds the closest source of ammo
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

	//Sets the speed of the guard
	private void setSpeed(float speed) {
		GetComponent<AINavSteeringController> ().mecanimInputForwardSpeedCap = speed;
	}
}
