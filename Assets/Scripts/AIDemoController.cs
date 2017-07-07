using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[RequireComponent (typeof(AINavSteeringController))]
public class AIDemoController : MonoBehaviour {


    public Transform [] waypointSetA; 

    public Transform [] waypointSetB; 

    public Transform [] waypointSetC; 

	public Transform waypointE;


    public enum State {

        A,B,C,D,E

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

        transitionToStateA();
		
	}
	

    void transitionToStateA() {

        print("Transition to state A");

        state = State.A;

        aiSteer.setWayPoints(waypointSetA);

        aiSteer.useNavMeshPathPlanning = true;


    }


    void transitionToStateB() {

        print("Transition to state B");

        state = State.B;

        aiSteer.setWayPoints(waypointSetB);

        aiSteer.useNavMeshPathPlanning = true;
    }


    void transitionToStateC() {

        print("Transition to state C");

        state = State.C;

        aiSteer.setWayPoints(waypointSetC);

        aiSteer.useNavMeshPathPlanning = false;

    }

    void transitionToStateD() {

        print("Transition to state D");

        state = State.D;

        beginWaitTime = Time.timeSinceLevelLoad;

		aiSteer.clearWaypoints ();

		aiSteer.useNavMeshPathPlanning = true;

    }


	void transitionToStateE() {

		print("Transition to state E");

		state = State.E;
	
		aiSteer.setWayPoint (waypointE);

		aiSteer.useNavMeshPathPlanning = true;

	}

	// Update is called once per frame
	void Update () {
		
        switch (state)
        {
            case State.A:
                
                if (aiSteer.waypointsComplete())
                    transitionToStateB();
                break;

            case State.B:
                if (aiSteer.waypointsComplete())
                    transitionToStateC();
                break;

            case State.C:
                if (aiSteer.waypointsComplete())
                    transitionToStateD();
                break;

            case State.D:
                if (Time.timeSinceLevelLoad - beginWaitTime > waitTime)
                    transitionToStateE();
                break;

		case State.E:
			if (aiSteer.waypointsComplete ())
				break;
//					transitionToStateA();
			break;
            default:

                print("Weird?");
                break;
        }


	}
}
