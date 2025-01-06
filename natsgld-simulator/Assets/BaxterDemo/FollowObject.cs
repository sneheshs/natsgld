using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowObject : MonoBehaviour {
    public List<Transform> transforms;

    public Transform controlled;
    
    int current;

    // Use this for initialization
    void Start ()
    {
        current = 0;
        transform.position = transforms[current].position;
	}
	
	// Update is called once per frame
	void Update ()
    {
        if (current < transforms.Count && Vector3.Distance(controlled.position, transforms[current].position) < 0.011f)
        {
            //Debug.Log(current);
            if (current == 0)
            {
                Transform pickedObj = transforms[current].GetChild(0);
                Vector3 offset = pickedObj.position - pickedObj.GetChild(0).position;

                pickedObj.parent = controlled;
                pickedObj.position = controlled.position + offset;
                pickedObj.GetComponent<Rigidbody>().useGravity = false;
            }

            if (current == 2)
            {
                Transform pickedObj = controlled.GetChild(0);

                pickedObj.parent = null;
                pickedObj.GetComponent<Rigidbody>().useGravity = true;
            }

            current++;
            if (current != transforms.Count)
                transform.position = transforms[current].position;
        }
	}
}
