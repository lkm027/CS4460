using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestExpMovingAvg : MonoBehaviour {

    public bool runIt = false;
    public bool clearAvg = false;
    public float timeWindow = 10f;
    public float sampleTime = 1 / 60f;
    public float initialValueOfMovingAvg = 5f;
    public float exponentialMovingAvg = 5f;
    public float halfWayPointexpMovingAvg = 5f;
    public float newMeasure = 1f;
    public float power = 1f;
    //public float power2 = 1f;
    public float normErr = 0f;


    public float avgTotal = 0f;
    public int sampleTotal = 0;
    public float timeTotal = 0f;


    public AnimationCurve movingAvg = AnimationCurve.Linear (0f, 0f, 1f, 0f);



    protected void clearMovingAvg() {

        //not efficient, but just temporarily used for debugging
        movingAvg.keys = new Keyframe[0];

        //make a nice flat line for keyframes to be inserted into
        movingAvg.AddKey (new Keyframe (0f, 0f));
        movingAvg.AddKey (new Keyframe (timeWindow, 0f));

    }




 
	// Use this for initialization
	void Start () {


	}
	
	// Update is called once per frame
	void Update () {
		
        if (clearAvg)
        {
            clearAvg = false;

            clearMovingAvg();

        }

        if (runIt)
        {
            runIt = false;

            clearMovingAvg();

            exponentialMovingAvg = initialValueOfMovingAvg;

            int sampleCount = (int)Mathf.Ceil(timeWindow / sampleTime);
            int halfSampleCount = sampleCount / 2;

            for (int i = 0; i < sampleCount; ++i)
            {

                float t = Mathf.Min(1f, sampleTime / timeWindow);

                         
                t = Mathf.Pow(t, power);
                exponentialMovingAvg = exponentialMovingAvg * (1f - t) + newMeasure * t;

        

                float graphTime = i * sampleTime;

                movingAvg.AddKey (new Keyframe (graphTime, exponentialMovingAvg));

                                      
                if (i == halfSampleCount)
                    halfWayPointexpMovingAvg = exponentialMovingAvg;


            }

            normErr = Mathf.Abs((exponentialMovingAvg - newMeasure) / (initialValueOfMovingAvg - newMeasure));


                

        }


	}







    void OnValidate()
    {

        runIt = true;

    }
}
