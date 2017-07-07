using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SecurityCamera : MonoBehaviour {


	public GameObject lookAt;

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void LateUpdate () {

		if (lookAt != null)
			this.transform.LookAt (lookAt.transform);
	}
}
