//#define USE_CHARACTER_CONTROLLER


using UnityEngine;
using System.Collections;
using UnityEngine.AI;


/// <summary>
/// AI script that integrates NavMeshAgent with a Mecanim Animator with two params (forward+turn left/right).
/// Author: Jeff Wilson (jeff@imtc.gatech.edu)
/// Interactive Media Technology Center
/// Georgia Insitute of Technology
/// </summary>


// Require these components when using this script
[RequireComponent (typeof(Animator))]
#if USE_CHARACTER_CONTROLLER
[RequireComponent (typeof(CharacterController))]
#else
[RequireComponent (typeof(CapsuleCollider))]
#endif
[RequireComponent (typeof(Rigidbody))]
[RequireComponent (typeof(NavMeshAgent))]
public class AINavSteeringController : MonoBehaviour
{

	//Component Refs
	protected Animator anim;

	#if USE_CHARACTER_CONTROLLER
	protected CharacterController charController;
	#else
	protected CapsuleCollider cap;
	#endif

	protected NavMeshAgent agent;
	

    //used to test for bad vectors that are too short to give a good heading; affected by float precision induced error
	protected const float VectorLengthThreshold = 0.001f;
	protected const float VectorLengthThresholdSqr = VectorLengthThreshold * VectorLengthThreshold;

	//Mecanim related:

	//Animator param names that agent will control
    [Tooltip("mecanimInputParamForward: Name of the forward Mecanim parameter. Expected to support range [0,1] with 0 stopped and 1 full speed forward. Ok if your Animator uses range [-1,0) for backward movement, but will be ignored here.")]
	public string mecanimInputParamForward = "";
    [Tooltip("mecanimInputParamTurn: Name of the turning Mecanim parameter. Expected to support range [-1, 1] with -1 full left turn, 1 full right turn, and 0 no turn.")]
	public string mecanimInputParamTurn = "";

    //hash of above params
  
    protected bool mecanimInputParamForwardIsValid = false;
    protected int mecanimInputParamForwardHash;
    protected bool mecanimInputParamTurnIsValid = false;
    protected int mecanimInputParamTurnHash;

	//use to force mecanim to slow down even if AI would otherwise want to go faster
    [Tooltip("mecanimInputForwardSpeedCap: Maximum normalized speed in range [0, 1] allowed to be sent to the Animator. All higher values are clamped to this limit.")]
    [Range(0f,1f)]
    public float mecanimInputForwardSpeedCap = 1f;

	//These params should be set by looking at your fastest animations and seeing translation and rot speeds
	//It's important that you look these up and set manually! It will make tuning much easier if you do
    [Tooltip("mecanimMaxSpeed: The maximum speed the Animator will go in m/sec. You can look at your fastest forward animation in the inspector for average forward speed. IT'S IMPORTANT TO SET THIS VALUE CORRECTLY!")]
    public float mecanimMaxSpeed = 5f; //m/sec
    [Tooltip("mecanimMaxTurn: The maximum turn rate the Animator will go in degrees/sec. You can look at your fastest turning animation (that is also going forward) in the inspector for average turn rate. IT'S IMPORTANT TO SET THIS VALUE CORRECTLY!")]
	public float mecanimMaxTurn = 180f;  //degrees/sec

	//AI related member vars:

	//Where is AI going?
	protected int currWayPoint = 0;
    [Tooltip("waypoints: A list of waypoints that will be visited in order by the AI (and then start over from beginning). Make sure all can be reached and the NavMesh is baked properly!")]
    public Transform [] waypoints = new Transform[0]; 

    [Tooltip("stopAtNextWaypoint: Will the agent stop at the waypoint, or will the agent immediately go to the next one?")]
	public bool stopAtNextWaypoint = false;
    [Tooltip("waypointLoop: Will waypoints be restarted?")]
    public bool waypointLoop = true;

    [Tooltip("useNavMeshPathPlanning: Determines if the nav mesh agent's path planning is used. Otherwise, simple steer behavior is followed.")]
    public bool useNavMeshPathPlanning = true;


	//how close to does character need to get to say that the waypoint has been visited?
    [Tooltip("waypointCaptureRadius: How close does the character need to get to the waypoint for it to be counted as visited and then move on to the next?")]
    public float waypointCaptureRadius = 1f;
    protected float waypointCaptureRadiusSqr = 1f;

	//NavMeshAgent related:

	//Set with trial and error to get agent to approx match mecanim
    [Tooltip("agentMaxAccel: The maximum acceleration the agent can go in m/sec^2")]
    public float agentMaxAccel = 8f; //m/sec^2
    [Tooltip("agentUseBrakesToStopOnTarget: Will the agent slow down as the waypoint is approached?")]
	public bool agentUseBrakesToStopOnTarget = true;
    [Tooltip("agentBrakingDistance: At what distance from target will the agent start braking?")]
	public float agentBrakingDistance = 0.8f; //m

    //used to tweak normalization of agent heading such that we get a nice range of values [-1, 1]
    [Tooltip("agentAngleNormalizationFactor: This value is used to scale the observed turn rate such that the resulting value is useful to pass on to the Animator. THIS IS PROBABLY MOST IMPORTANT TO ADJUST AFTER MECANIM PARAMS ABOVE!")]
    public float agentAngleNormalizationFactor = 10f; //THIS IS PROBABLY MOST IMPORTANT TO ADJUST AFTER MECANIM PARAMS ABOVE!
    [Tooltip("agentAngleNormalizationPower: Used to curve the angle normalization function up or down. Essentially, the interpolation becomes non-linear.")]
    public float agentAngleNormalizationPower = 1f;

    //If a full speed turn is being applied, how much should we slow down forward speed?
    //This speed will be linearly interpolated up to full speed as the turn angle diminishes
    [Tooltip("maxNormForwardSpeedAtMaxTurn: Does your character turn too wide a radius at full forward speed? Try slowing down the AI when turning using this value.")]
    [Range(0f,1f)]
    public float maxNormForwardSpeedAtMaxTurn = 0.8f; //Turns too wide? Slow down with this setting

	//Input smoothing
    [Tooltip("inputDoSmoothing: Toggle smoothing of input values. You most likely want it on, but it's useful to turn off while tuning agent settings.")]
	public bool inputDoSmoothing = true;
	protected float inputVelForward = 0f;
    [Tooltip("inputMaxAccelForward: Acceleration (normalized units per sec^2) setting for how fast mecanim forward input parameter moves towards the new values coming in.")]
	public float inputMaxAccelForward = 500f;
	protected float inputVelTurn = 0f;
    [Tooltip("inputMaxAccelTurn: Acceleration (normalized units per sec^2) setting for how fast mecanim turn input parameter moves towards the new values coming in.")]
	public float inputMaxAccelTurn = 800f;
    [Tooltip("inputFilterPowerForward: Used to curve the mecanim input forward normalization function up or down. Essentially, the interpolation becomes non-linear.")]
	public float inputFilterPowerForward = 1f;
    [Tooltip("inputFilterPowerTurn: Used to curve the mecanim input turn normalization function up or down. Essentially, the interpolation becomes non-linear.")]
	public float inputFilterPowerTurn = 1f;

	//smoothing of agent pos

	public enum AgentPosCorrectionTypes
	{
		NoCorrection,
		StrictCorrection,
		CorrectionWithSmoothing
	}

    [Tooltip("agentPosCorrection: The type of correction used on the navMeshAgent postion. NoCorrection - Do nothing. StrictCorrection - Every frame agent is set to Animator root motion position and velocity. CorrectionWithSmoothing - Every frame agent is moved towards Animator root position (velocity is directly copied)")]
	public AgentPosCorrectionTypes agentPosCorrection = AgentPosCorrectionTypes.StrictCorrection;
	protected Vector3 agentPosVel;
	//max dist before corrections applied
    [Tooltip("agentPosMaxDistanceBeforeCorrection: Only applies to agentPosCorrection=CorrectionWithSmoothing. Specifies how far the navMeshAgent is allowed to be from the Animator root motion position before correction is applied.")]
    public float agentPosMaxDistanceBeforeCorrection = 0.5f;
    protected float agentPosMaxDistanceBeforeCorrectionSqr = 0.5f * 0.5f;
    [Tooltip("agentPosMaxAccelToCorrection: The maximum acceleration allowed in correcting agent to Animator root motion position.")]
    public float agentPosMaxAccelToCorrection = 200f;


    protected Vector3 prevPosition;



    //Following used for correcting failure case of navMeshAgent and character getting separated
    [Tooltip("expAvgTimeWindow: Amount of time our exponential moving average is meant to approximate.")]
    public float expAvgTimeWindow = 5f;
    [Tooltip("expAvgPowerFactor: Tune by hand such that a constant measure over timeWindow samples results in the avgVel being within a desired error bounds of the measure.")]
    public float expAvgPowerFactor = 0.75f;
    [Tooltip("expAvgVelTowardsSteeringTarget: Meant to be readonly!")]
    public float expAvgVelTowardsSteeringTarget;
    [Tooltip("minAvgVelTowardsSteeringTarget: If the avg vel ever drops below this value then we assume character got separated from nav mesh agent and agent position cannot be set normally to fix. Therefore, we reset character to the agent.")]
    public float minAvgVelTowardsSteeringTarget = 0.05f;
    [Tooltip("positionCorrectionOverTime: Approximate amount of time that position of character will take to be corrected to a stopped agent.")]
    public float positionCorrectionOverTime = 5f;


    protected bool isComplete = false;
    protected bool isTargetedForStop = false;

    protected bool isInit = false;

	void Start ()
	{
      
        Init();

	}


    public void Init() {

        if (isInit)
            return;

        anim = GetComponent<Animator> (); 

		#if USE_CHARACTER_CONTROLLER
		charController = GetComponent<CharacterController>();
		#else
        cap = GetComponent<CapsuleCollider> ();
		#endif
        agent = GetComponent<UnityEngine.AI.NavMeshAgent>();

        //check if mecanim params are good and if so, remember the hash code for each
        checkMecanimParam(ref mecanimInputParamForward, ref mecanimInputParamForwardHash, ref mecanimInputParamForwardIsValid);
        checkMecanimParam(ref mecanimInputParamTurn, ref mecanimInputParamTurnHash, ref mecanimInputParamTurnIsValid);


        //make sure agent is working
        agent.enabled = true;


        //don't allow agent to move the character. we want mecanim to do it
        agent.updatePosition = false;
        agent.updateRotation = false;

        isComplete = false;

        //prep first waypoint
        internalSetDestination();

        setNavMeshAgent ();


        waypointCaptureRadiusSqr = waypointCaptureRadius * waypointCaptureRadius;

        agentPosMaxDistanceBeforeCorrectionSqr = agentPosMaxDistanceBeforeCorrection * agentPosMaxDistanceBeforeCorrection;


        prevPosition = this.transform.position;

        expAvgVelTowardsSteeringTarget = mecanimMaxSpeed;

        isInit = true;
    }





    protected void internalSetDestination() {

        currWayPoint = 0;
        if (waypoints.Length > 0 && useNavMeshPathPlanning)
        {
            if(agent != null)
				agent.SetDestination(waypoints[currWayPoint].position);
        }

        ///nudge in the dir mecanim wants to go. not sure if it helps...
        if(agent != null)
            agent.velocity = this.transform.forward * 0.1f;

    }


	//params regularly synced from this script with navMeshAgent
	protected void setNavMeshAgent() {

		//agent inherits settings from AIScript for centralized control. Set in realtime for debugging purposes
		agent.acceleration = agentMaxAccel;
		agent.speed = mecanimMaxSpeed;
		agent.angularSpeed = mecanimMaxTurn;
		agent.autoBraking = agentUseBrakesToStopOnTarget;
		agent.stoppingDistance = agentBrakingDistance;

		#if USE_CHARACTER_CONTROLLER
		agent.radius = charController.radius;
		agent.height = charController.height;
		#else
		agent.radius = cap.radius;
		agent.height = cap.height;
		#endif

	}


    public void setWayPoints(Transform [] newWaypoints) {

        if (newWaypoints == null)
            return;

        currWayPoint = 0;

        waypoints = new Transform[newWaypoints.Length];

        for (int i = 0; i < waypoints.Length; ++i)
        {
            waypoints[i] = newWaypoints[i];
        }

        internalSetDestination();

        isComplete = false;
        isTargetedForStop = false;

    }

    public void setWayPoint(Transform newWaypoint) {

        currWayPoint = 0;

		waypoints = new Transform[1];

        waypoints[0] = newWaypoint;

        internalSetDestination();

        isComplete = false;
        isTargetedForStop = false;
    }


    public void clearWaypoints() {

        currWayPoint = 0;

        waypoints = new Transform[0];

        isComplete = true;
        isTargetedForStop = true;

    }


    public bool waypointsComplete() {

        if (waypoints.Length < 0)
            return true;

        if (waypointLoop)
            return false; //never complete

        return isComplete;

    }

    public bool targettedForStop() {
        return isTargetedForStop;
    }


	void FixedUpdate() {

        if(anim.updateMode == AnimatorUpdateMode.AnimatePhysics)
		    AIControlled_Update ();

	}


	void Update() {

        if(anim.updateMode != AnimatorUpdateMode.AnimatePhysics)
            AIControlled_Update ();

        #if UNITY_EDITOR
		if(debugOutput)
			DebugDrawPath ();
        #endif
	}

 

	//This should be called from Update() or FixedUpdate()
	//I think it should match the update mode of Animator
	void AIControlled_Update()
	{

        float currAnimValueTurn = 0f;
        float currAnimValueForward = 0f;

        if(mecanimInputParamForwardIsValid)
            currAnimValueForward = anim.GetFloat (mecanimInputParamForwardHash);
        
        if(mecanimInputParamTurnIsValid)
            currAnimValueTurn= anim.GetFloat (mecanimInputParamTurnHash);


        isTargetedForStop = false;

		//have we reached our waypoint?

        if (waypoints.Length > 0)
        {

            Vector3 targetPos;

            if (useNavMeshPathPlanning)
                targetPos = agent.destination;
            else
                targetPos = waypoints[currWayPoint].position;

            bool waypointReached = ((this.transform.position - targetPos).sqrMagnitude < waypointCaptureRadiusSqr);
                      

            if (waypointReached)
            {

                if (stopAtNextWaypoint)
                {
                    agent.ResetPath();
                    isTargetedForStop = true;

                    if (currWayPoint >= waypoints.Length - 1)
                        isComplete = true;
                }
                else
                {
                    if (currWayPoint < waypoints.Length - 1)
                    {
                        ++currWayPoint;

                        //now head for next waypoint
                        if(useNavMeshPathPlanning)
                            agent.SetDestination(waypoints[currWayPoint].position);  

                    }
                    else
                    {
                        if (waypointLoop)
                        {
                            currWayPoint = 0;

                            //now head for next waypoint
                            if(useNavMeshPathPlanning)
                                agent.SetDestination(waypoints[currWayPoint].position);
                        }
                        else
                        {
                            agent.ResetPath(); 
                            isTargetedForStop = true;
                            isComplete = true;
                        }
                    }                       
                }
            }
        }

             
        float agentDesiredSpeed = 0f;

        if (waypoints.Length > 0)
        {
            if (useNavMeshPathPlanning)
                agentDesiredSpeed = agent.desiredVelocity.magnitude;
            else
            {
            
                //for now, we just go full speed unless stopping is set
                if(!isTargetedForStop)
                    agentDesiredSpeed = agent.speed;
            }

        }


		//normalize to [0, MAX_SPEED] so we can create a mecanim input value
		//I don't think the agent ever goes backward
		float agentNormForward = agentDesiredSpeed / agent.speed;

		//just in case agent going faster than the speed limit
		agentNormForward = Mathf.Clamp(agentNormForward, 0f, 1f);

        Vector3 candidateAgentDesiredHeading = transform.forward;

        if (waypoints.Length > 0)
        {

            if (useNavMeshPathPlanning)
            {
                candidateAgentDesiredHeading = agent.desiredVelocity;

                if (candidateAgentDesiredHeading.sqrMagnitude < VectorLengthThresholdSqr)
                {
                    //the candidate is very short, maybe from some obstacle avoidance or something
                    //try another

                    #if UNITY_EDITOR
                    if (debugOutput)
                        print("(path plan) bad vector 1");
                    #endif

                    candidateAgentDesiredHeading = agent.steeringTarget - agent.nextPosition;

                    if (candidateAgentDesiredHeading.sqrMagnitude < VectorLengthThresholdSqr)
                    {
                        //another too short

                        #if UNITY_EDITOR
                        if (debugOutput)
                            print("(path plan) bad vector 2");
                        #endif

                        //use something reasonable, can safely assume it's a unit vector w/o noisy direction
                        candidateAgentDesiredHeading = transform.forward;
                    }                  
                }
            }
            else
            {
                //Just doing simple steering

                candidateAgentDesiredHeading = waypoints[currWayPoint].position - transform.position;

                if (candidateAgentDesiredHeading.sqrMagnitude < VectorLengthThresholdSqr)
                {
                    //the candidate is very short, maybe from some obstacle avoidance or something
                    //try another

                    #if UNITY_EDITOR
                    if (debugOutput)
                        print("(steering) bad vector 1");
                    #endif

                    candidateAgentDesiredHeading = transform.forward;

                }                    
            }
        }



		//Start working towards a normalized turn value for mecanim 

		Vector3 agentDesiredHeading = candidateAgentDesiredHeading; //don't think I need to normalize to unit vector
		Vector2 desiredHeading2D = new Vector2 (agentDesiredHeading.x, agentDesiredHeading.z);


		Vector3 currHeading = this.transform.forward;
		Vector2 currHeading2D = new Vector2 (currHeading.x, currHeading.z);


		float angle = SignedAngle (currHeading2D, desiredHeading2D); //correction angle needed to match agent

	
		//Note use of angleNormalizationFactor to avoid full turn for small angle corrections
		float agentNormAngle = (angle/Time.deltaTime) / (agent.angularSpeed * agentAngleNormalizationFactor);

		//clamp to range [-1, 1]
		agentNormAngle = Mathf.Clamp (agentNormAngle, -1f, 1f);

		float agentNormAngleSign = Mathf.Sign (agentNormAngle); //pay no attention to this imaginary workaround
		//use of power allows lerp param to be non-linear and curve to bow up or down depending on power 
		agentNormAngle =  Mathf.Pow (Mathf.Abs(agentNormAngle), agentAngleNormalizationPower);
		agentNormAngle *= agentNormAngleSign;


        //speed limit when turning?
        float speedAdj = maxNormForwardSpeedAtMaxTurn + (1f - Mathf.Abs (agentNormAngle)) * (1f - maxNormForwardSpeedAtMaxTurn);
        agentNormForward = speedAdj * agentNormForward;


       
		//Here, we optionally apply input filtering/smoothing similar to what Unity's InputManager is capable of

		//prime with good values in case we don't run input smoothing
		float mecanimInputTurn = agentNormAngle;
		float mecanimInputForward = agentNormForward;

		if (inputDoSmoothing) {

			//simple physics based model for input changes
			float dt = Time.deltaTime;
			float deltaSquared = dt*dt;

			//our accel from current to new value is faster if the values are further away
			//technique sometimes called poor man's Kalman Filter
			//conceptually, treats values that are slightly different than current as noise that can largely be ignored
			float filteredAccelForward = filterAccel (currAnimValueForward, agentNormForward, 1f, inputFilterPowerForward, inputMaxAccelForward);

			//now we use filtering based on physics to move from curr input value toward target. probably won't get there immediately due
			//to physics contraints. But will eventually after a few updates()
			mecanimInputForward = filter (currAnimValueForward, agentNormForward, dt, deltaSquared, 
				filteredAccelForward, ref inputVelForward);

			//same filtering approach as above for forward Direction now applied to turning
			float filteredAccelTurn = filterAccel (currAnimValueTurn, agentNormAngle, 2f, inputFilterPowerTurn, inputMaxAccelTurn);

			mecanimInputTurn = filter (currAnimValueTurn, agentNormAngle, dt, deltaSquared, 
				filteredAccelTurn, ref inputVelTurn);

		}
	
		//allow override of top speed in case we want to slow down the action
		float mecanimSpeedForward = Mathf.Min (mecanimInputForwardSpeedCap, mecanimInputForward);


		//Tell mecanim what to do based off of agent
        if(mecanimInputParamForwardIsValid) 
            anim.SetFloat (mecanimInputParamForwardHash, mecanimSpeedForward);

        if(mecanimInputParamTurnIsValid)
    		anim.SetFloat (mecanimInputParamTurnHash, mecanimInputTurn); 				


   
		////////////////////////////////////
		//Some debugging output to inspector follows
		////////////////////////////////////

        #if UNITY_EDITOR
		if (debugOutput) {

			//draw to scene view in editor
            Debug.DrawRay (this.transform.position, agentDesiredHeading.normalized * 5f, Color.green);
			Debug.DrawRay (this.transform.position, currHeading * 5f, Color.blue);


			Vector2 target;

			if(waypoints.Length > 0)
				target = new Vector2(waypoints[currWayPoint].position.x, waypoints[currWayPoint].position.z);
			else
				target = new Vector2(this.transform.position.x, this.transform.position.z);

			float targDist = Vector2.Distance (new Vector2(this.transform.position.x, this.transform.position.z), 
													target);


			float distToSteeringTarget = Vector2.Distance (new Vector2(this.transform.position.x, this.transform.position.z), 
				new Vector2(agent.steeringTarget.x, agent.steeringTarget.y));

			//Debug output to Inspector
			debugDistToTarget = targDist;
            debugDistToSteeringTarget = distToSteeringTarget;
			debugForward = transform.forward;
			debugNormSpeed = agentNormForward;
			debugDesiredHeadingCorrection = angle;
			debugNormAngle = mecanimInputTurn;	
            debugAgentPathPending = agent.pathPending;
            debugAgentHasPath = agent.hasPath; 

		}
        #endif

	}



	void OnAnimatorMove ()
	{

        Vector3 origPosition = this.transform.position;


		//regular mecanim root motion behavior BEGINS
		this.transform.position = anim.rootPosition;
		this.transform.rotation = anim.rootRotation;
		//regular mecanim root motion behavior ENDS



		//loose coupling of agent to mecanim can be used.
		//This is because the mecanim root position follows the hips of character. As a turn is initiated, the root position
		//immediately shifts in the perpendicular direction of turn (with typical turn animations that lean into curve). 
		//Then end result is potentially a viscious cycle of turning slightly left and right
		//as the navMeshAgent tries to aim at the desired trajectory and keeps getting knocked off course by the hip
		//lean. In the demo this code is accociated with it's not a problem due to good input filtering, so disabled

		switch (agentPosCorrection) {
			case AgentPosCorrectionTypes.NoCorrection:
				//do nothing
				break;

            case AgentPosCorrectionTypes.StrictCorrection:
				//agent is forced to match character strictly             
                agent.nextPosition = anim.rootPosition;             
                agent.velocity = anim.velocity;              
				break;

			case AgentPosCorrectionTypes.CorrectionWithSmoothing:
				
				//agent is forced to match character (loosely)
				float dt = Time.deltaTime;
				float dt2 = Time.deltaTime * Time.deltaTime;

				float agentAccelToNewPos = filterAccel ( Mathf.Max(0f, (anim.rootPosition - agent.nextPosition).magnitude - agentPosMaxDistanceBeforeCorrection), 
					agentPosMaxDistanceBeforeCorrection, 1f, agentPosMaxAccelToCorrection);
				
				Vector3 finalAgentPos = filter (agent.nextPosition, anim.rootPosition, dt, dt2, agentAccelToNewPos, ref agentPosVel);
	
				agent.nextPosition = finalAgentPos;
				agent.velocity = anim.velocity; //not bothering to filter velocity
				break;
		}
			
        positionCorrection(origPosition, prevPosition);

        prevPosition = origPosition;
			
	}



	// Some helper functions


    protected void positionCorrection(Vector3 origPos, Vector3 prevPos) {
        float t;

        if (!agent.pathPending)
        {
            //if agent isn't moving, pull the character to the agent
            t = Mathf.Min(1f, Time.deltaTime / positionCorrectionOverTime);
			//TODO: this may be susceptible to falling through quad colliders and terrain
			this.transform.position = Vector3.Lerp (this.transform.position, agent.nextPosition, t); //new Vector3(agent.nextPosition.x, transform.position.y, agent.nextPosition.z), t);

        }




         //Check to see if we are making progress
         //use of prevPos and currentPos because both have been corrected by physics engine. anim.rootPosition has not

         Vector3 steeringDirection = agent.steeringTarget - origPos;

         if(steeringDirection.sqrMagnitude < VectorLengthThresholdSqr)
         {
             steeringDirection = transform.forward;

         }

         float gainOnSteeringTarget = Vector3.Project((origPos - prevPos), 
             steeringDirection.normalized).magnitude;

         float velGainOnSteeringTarget = gainOnSteeringTarget / Time.deltaTime;
         t = Mathf.Min(1f, Time.deltaTime / expAvgTimeWindow);
         t = Mathf.Pow(t, expAvgPowerFactor);
         expAvgVelTowardsSteeringTarget = Mathf.Lerp(expAvgVelTowardsSteeringTarget, velGainOnSteeringTarget, t);

         if (expAvgVelTowardsSteeringTarget < minAvgVelTowardsSteeringTarget)
         {
             #if UNITY_EDITOR
             if (debugOutput)
             {
                 print("Agent appears to be irrecoverably separated from avatar. Moving avatar to agent.");
             }
             #endif

			//TODO: this may be susceptible to falling through a quad collider or terrain
			this.transform.position = agent.nextPosition;//new Vector3(agent.nextPosition.x, transform.position.y, agent.nextPosition.z);

             expAvgVelTowardsSteeringTarget = mecanimMaxSpeed; 
         }


         #if UNITY_EDITOR
         if (debugOutput)
         {
             debugGainOnSteeringTarget = gainOnSteeringTarget;
         }
         #endif



    }



	//dynamic accel to remove some ripple
	protected static float filterAccel(float currVal, float nextVal, float magnitude, float power, float maxAccel) {

		float normDiff = Mathf.Abs (currVal - nextVal); 
		return filterAccel (normDiff, magnitude, power, maxAccel);

	}


	protected static float filterAccel(float scalar, float magnitude, float power, float maxAccel) {

		float normScalar = Mathf.Abs (scalar)/magnitude; 
		normScalar = Mathf.Clamp (normScalar, 0f, 1f);
		return filterAccel (normScalar, power, maxAccel);

	}



	protected static float filterAccel(float normScalar, float power, float maxAccel) {

		float normSign = Mathf.Sign (normScalar); //imaginary workaround
		normScalar = Mathf.Pow (Mathf.Abs(normScalar), power); //power for non-linear filtering
		normScalar = normSign * normScalar;
		return Mathf.Lerp (0f, maxAccel, normScalar);

	}



	//input filtering using basic particle dynamics
	protected static float filter(float currPos, float targetPos, float dt, float dtsqr, float maxAbsAccel, ref float vel) {

		//accel needed to reach targetPos from currPos considering current vel and time elapsed
		float accel = 2f * (targetPos - currPos - vel * dt) / dtsqr;

		//now limit the accel to what we actually allow according to the maximum defined
		float clampedAccel = Mathf.Clamp (accel, -maxAbsAccel, maxAbsAccel);

		//update our velocity
		vel = clampedAccel * dt;

		//the new position we got to. possibly didn't get to the target, but that's the idea for filtering purposes!
		return (currPos + vel * dt);

	}



	//input filtering using basic particle dynamics - this version works with Vector3's
	protected static Vector3 filter(Vector3 currPos, Vector3 targetPos, float dt, float dtsqr, float maxAbsAccel, ref Vector3 vel) {

		//accel needed to reach targetPos from currPos considering current vel and time elapsed
		Vector3 accel = 2f * (targetPos - currPos - vel * dt) / dtsqr;

		//now limit the accel to what we actually allow according to the maximum defined
		Vector3 clampedAccel = Vector3.ClampMagnitude (accel, maxAbsAccel);

		//update our velocity
		vel = clampedAccel * dt;

		//the new position we got to. possibly didn't get to the target, but that's the idea for filtering purposes!
		return (currPos + vel * dt);

	}



	//Find angle between two vectors with a sign for CCW versus CW
	protected static float SignedAngle(Vector2 a, Vector2 b)
	{
		//Find relative angle we need to correct to
		float angle = Mathf.Rad2Deg * (  Mathf.Atan2 (a.y, a.x)  - Mathf.Atan2(b.y, b.x) ) ;

		angle %= 360f;
		angle = (angle + 360f) % 360f;
		if (angle > 180f)
			angle -= 360f;

		return angle;


		//Another method of getting relative angle
		//		float angle =  Vector2.Angle (desiredHeading, currHeading);
		//
		//
		//		//determine sign based on CW or CCW
		//		if (currHeading.y * desiredHeading.x < currHeading.x * desiredHeading.y)
		//			angle = -angle;
		//
	}


    //Used to check if a mecanim param is supported by the animator. If so, find the hash and store it
    void checkMecanimParam(ref string paramName, ref int paramHash, ref bool isValid) {

        if (anim != null) //can only check during live play
        {

            isValid = false;

            foreach (AnimatorControllerParameter param in anim.parameters)
            {
                if (param.name == paramName)
                {
                    isValid = true;

                    paramHash = param.nameHash;

                    break;
                }

            }

            if (!isValid)
            {
                print("Mecanim param: " + paramName + " doesn't seem to exist in the Animator.");                       

            }

        }

    }



    #if UNITY_EDITOR

    //debugging output + control
    public bool debugOutput = false;
    public Vector3 debugForward;
    public float debugDesiredHeadingCorrection;
    public float debugNormAngle;
    public float debugNormSpeed;
    public float debugDistToTarget;
    public float debugDistToSteeringTarget;
    public float debugGainOnSteeringTarget;
    public bool debugAgentPathPending;
    public bool debugAgentHasPath;

    protected LineRenderer line; //for drawing waypoint path
    const int MaxCorners = 1000;
    Vector3[] corners = new Vector3[MaxCorners];

    //Called in editor when inspector values changed. Allows for error detection/correction of values
    void OnValidate()
    {
        checkMecanimParam(ref mecanimInputParamForward, ref mecanimInputParamForwardHash, ref mecanimInputParamForwardIsValid);
        checkMecanimParam(ref mecanimInputParamTurn, ref mecanimInputParamTurnHash, ref mecanimInputParamTurnIsValid);

        forcePositive(ref mecanimMaxSpeed, 1f);
        forcePositive(ref mecanimMaxTurn, 10f);

        forcePositive(ref waypointCaptureRadius, 2f);

        forcePositive(ref agentMaxAccel, 1f);
        forcePositive(ref agentBrakingDistance);

        forcePositive(ref agentAngleNormalizationFactor, 1f);
        forcePositive(ref agentAngleNormalizationPower, 1f);

        forcePositive(ref inputMaxAccelForward, 300f);
        forcePositive(ref inputMaxAccelTurn, 300f);

        forcePositive(ref inputFilterPowerForward, 1f);
        forcePositive(ref inputFilterPowerTurn, 1f);

        forcePositive(ref agentPosMaxDistanceBeforeCorrection, 1f);
        forcePositive(ref agentPosMaxAccelToCorrection, 200f);

        if (!debugOutput)
        {
            if (line != null)
            {
                line.positionCount = 0;
            }
        }

        waypointCaptureRadiusSqr = waypointCaptureRadius * waypointCaptureRadius;

        agentPosMaxDistanceBeforeCorrectionSqr = agentPosMaxDistanceBeforeCorrection * agentPosMaxDistanceBeforeCorrection;

        if(agent != null)
            setNavMeshAgent ();

    }

    //check if a value is negative. If so, correct to a default value
    void forcePositive(ref float v) {
        forcePositive(ref v, 0f);
    }

    void forcePositive(ref float v, float defaultVal) {
        if (v < 0f)
            v = defaultVal;
    }


	//draw path to current waypoint on game view
	void DebugDrawPath()
	{

		if( agent == null || agent.path == null )
			return;

		if( line == null )
		{
			
			line = this.GetComponent<LineRenderer>();

			if (line == null) {

				line = this.gameObject.AddComponent<LineRenderer> ();
				line.material = new Material (Shader.Find ("Sprites/Default")) { color = Color.yellow };
				line.startWidth = line.endWidth = 0.25f;             
				line.startColor = line.endColor = Color.yellow;

			}
		}

		var path = agent.path;

        line.positionCount = path.GetCornersNonAlloc(corners);
		//line.numPositions = path.corners.Length;

        for( int i = 0; i < line.positionCount ; ++i) 
		//for( int i = 0; i < path.corners.Length; i++ )
		{
            Vector3 c = corners[i];
			//Vector3 c = path.corners [i];
			c.Set (c.x, c.y+0.1f, c.z);			
			line.SetPosition( i, c );
		}

	}

    #endif
}


