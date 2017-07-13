using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;

public class LevelManager : MonoBehaviour {

	public int totalTilReset;

	private int taggedTotal;

	public void addTagged() {
		taggedTotal++;
		if (taggedTotal >= totalTilReset) {
			SceneManager.LoadScene ("m3v2");
		}
	}
}
