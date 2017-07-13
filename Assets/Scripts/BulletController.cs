using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BulletController : MonoBehaviour {

	void OnTriggerEnter(Collider other) {
		if (other.tag.Equals("Prisoner")) {
			print (other.tag);
			Destroy (gameObject);
			other.GetComponent<AIPrisonerController> ().hasBeenTagged ();
		} else if (other.tag.Equals("Scenery")) {
			Destroy (gameObject);
		}

	}
}
