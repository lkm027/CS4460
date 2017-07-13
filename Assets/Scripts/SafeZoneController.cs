using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SafeZoneController : MonoBehaviour {

	private int prisoners;
	public int fullSize;

	void Start () {
		prisoners = 0;
		if (fullSize == 0)
			fullSize = 2;
	}
	
	void OnTriggerEnter(Collider other) {
		if (other.tag.Equals("Prisoner")) {
			prisoners++;
		}
	}

	void OnTriggerExit(Collider other) {
		if (other.tag.Equals("Prisoner")) {
			prisoners--;
		}
	}

	public int getPrisoners() {
		return prisoners;
	}

	//Returns if the safezone is full of prisoners
	public bool isFull() {
		if (prisoners > fullSize) {
			print ("full");
			return true;
		}
			
		return false;
	}
}
