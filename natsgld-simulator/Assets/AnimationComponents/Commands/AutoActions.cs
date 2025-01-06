/*
MIT License

Copyright (c) 2025 Snehesh Shrestha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AutoActions : MonoBehaviour
{
    // --- For Objects ---
    [System.Serializable]
    public class WatchObjectRef
    {
        public string ObjectTag;
        public float GraspDistanceThreshold = 0.02f; // Obselete - TODO: Remove
        public float PickupDistanceThreshold = 0.04f;
        public float DropDistanceThreshold = 0.02f;
        public float PositionRandomizationThreshold = 0.2f;

        public GameObject Object;
        public MeshCollider collder;
        public Rigidbody rigidbody;

        public SaveObjectTag sot;
    }
    public WatchObjectRef[] WatchObjectsList;
    public WatchObjectRef[] SurfaceList;
    

    // --- For Hands ---
    public enum HandStates
    {
        Empty,
        Grabbed,
        PickedUp,
        PlacedDown,
        Dropped
    }

    private class TheHand
    {
        public String Name;
        public GameObject actionHand;
        public HandStates handState = HandStates.Empty;
        public GameObject lastAction;
        public int lastActionIndex;

        private LinkedList<Vector3> locationHistory = new LinkedList<Vector3>();
        private int LOCATION_HISTORY_SIZE = 15;
        private float distanceTotal = 0;
        private const float MOVING_THRESH = 0.00100f;

        public TheHand(String name)
        {
            Name = name;
        }

        public bool IsMoving()
        {
            if (locationHistory.Count < LOCATION_HISTORY_SIZE) return true;

            // Average out the distance moved per frame
            float avgDist = distanceTotal / (LOCATION_HISTORY_SIZE);

            // If the average distance moved is greater than the threshold we are moving
            return (avgDist > MOVING_THRESH);
        }

        public void ResetMovingHistory()
        {
            locationHistory.Clear();
            distanceTotal = 0;
        }

        public void UpdateMovingData()
        {
            Vector3 newPosition = actionHand.transform.position;

            // Start by adding the new position to the running total / queue
            if (locationHistory.Count > 0) {
                distanceTotal += Vector3.Distance(newPosition, locationHistory.First.Value);
            }
            locationHistory.AddFirst(newPosition);

            // Wait until the given sample size of positions has been collected
            if (locationHistory.Count > LOCATION_HISTORY_SIZE) {
                // Remove last distance from total / queue
                Vector3 last = locationHistory.Last.Value;
                locationHistory.RemoveLast();
                Vector3 secondLast = locationHistory.Last.Value;

                distanceTotal -= Vector3.Distance(last, secondLast);
            }
        }
    }

    private TheHand leftHand = new TheHand("LGripPos");
    private TheHand rightHand = new TheHand("RGripPos");

    private GameObject goImageSynth;

    public void Start()
    {
        goImageSynth = GameObject.FindGameObjectWithTag("Baxter_Camera");

        leftHand.actionHand = GameObject.FindGameObjectWithTag(leftHand.Name);
        rightHand.actionHand = GameObject.FindGameObjectWithTag(rightHand.Name);
        leftHand.lastAction = new GameObject();
        rightHand.lastAction = new GameObject();

        for (int i = 0; i < WatchObjectsList.Length; i++)
        {
            // Find the objects to watch for
            WatchObjectsList[i].Object = GameObject.FindGameObjectWithTag(WatchObjectsList[i].ObjectTag);

            if (WatchObjectsList[i].Object == null)
            {
                Debug.LogError("Object not found: " + WatchObjectsList[i].ObjectTag);
            }
            else
            {
                WatchObjectsList[i].rigidbody = WatchObjectsList[i].Object.GetComponent<Rigidbody>();
                if (WatchObjectsList[i].rigidbody == null)
                {
                    WatchObjectsList[i].rigidbody = WatchObjectsList[i].Object.AddComponent<Rigidbody>();
                }
                WatchObjectsList[i].rigidbody.useGravity = false;


                WatchObjectsList[i].collder = WatchObjectsList[i].Object.GetComponent<MeshCollider>();
                if (WatchObjectsList[i].collder == null)
                {
                    WatchObjectsList[i].collder = WatchObjectsList[i].Object.AddComponent<MeshCollider>();
                }
                WatchObjectsList[i].collder.convex = true;
                WatchObjectsList[i].collder.isTrigger = true;


                WatchObjectsList[i].sot = WatchObjectsList[i].Object.GetComponent<SaveObjectTag>();
                if (WatchObjectsList[i].sot == null)
                {
                    WatchObjectsList[i].sot = WatchObjectsList[i].Object.AddComponent<SaveObjectTag>();
                    
                    WatchObjectsList[i].sot.Tag = LayerMask.LayerToName(WatchObjectsList[i].Object.layer);
                    WatchObjectsList[i].sot.SaveMe = false;
                }
            }
        }

        // randomize_objects_placement();

        for (int i = 0; i < SurfaceList.Length; i++)
        {
            SurfaceList[i].Object = GameObject.FindGameObjectWithTag(SurfaceList[i].ObjectTag);

            if (SurfaceList[i].Object == null)
            {
                Debug.LogError("Surface Object not found: " + SurfaceList[i].ObjectTag);
            }
            else
            {
                SurfaceList[i].rigidbody = SurfaceList[i].Object.GetComponent<Rigidbody>();
                if (SurfaceList[i].rigidbody == null)
                {
                    SurfaceList[i].rigidbody = SurfaceList[i].Object.AddComponent<Rigidbody>();
                }
                SurfaceList[i].rigidbody.useGravity = false;

                SurfaceList[i].collder = SurfaceList[i].Object.GetComponent<MeshCollider>();
                if (SurfaceList[i].collder == null)
                {
                    SurfaceList[i].collder = SurfaceList[i].Object.AddComponent<MeshCollider>();
                }
                SurfaceList[i].collder.convex = true;
                SurfaceList[i].collder.isTrigger = true;
            }
        }
    }

    public void Update()
    {
        float distance = 0.0f;

        // Maintain moving history for AutoActions better logic
        leftHand.UpdateMovingData();
        rightHand.UpdateMovingData();

        for (int i=0; i< WatchObjectsList.Length ; i++)
        {
            /*
                Perform Pick Up Succes when Distance and Orientation are with a tolerance
                and the robot has stopped moving
                pickup here is the object to pick up and leftHand.actionHand or actionHandRight is the hand to pick it up with
                *** Right now Pos Only but should be Pos and Rot
            */
            // If hand is empty, check if object is close enough to pick up
            if (leftHand.handState == HandStates.Empty && !leftHand.IsMoving())
            {
                if (WatchObjectsList[i].sot.collidedWithTag == "LGripper")
                {
                    distance = Vector3.Distance(leftHand.actionHand.transform.position, WatchObjectsList[i].Object.transform.position);

                    goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;
                    
                    Debug.Log("Left Hand Object GRABBED: " + WatchObjectsList[i].ObjectTag + " at distance: " + distance);
                    
                    leftHand.lastAction.transform.position = WatchObjectsList[i].Object.transform.position;
                    leftHand.lastAction.transform.rotation = WatchObjectsList[i].Object.transform.rotation;
                    leftHand.lastActionIndex = i;

                    if (WatchObjectsList[i].Object.transform.parent.name == rightHand.Name)
                    {
                        // Debug.Log("Grabbed from the right hand");
                        rightHand.handState = HandStates.Empty;
                        rightHand.lastActionIndex = -1;
                    }
                    WatchObjectsList[i].Object.transform.SetParent(leftHand.actionHand.transform);

                    leftHand.handState = HandStates.Grabbed;
                    leftHand.ResetMovingHistory();
                    break;
                }
            }
            // Check if the object far enough from the CT or CB to be considered picked up
            else if (leftHand.handState == HandStates.Grabbed)
            {
                distance = Vector3.Distance(leftHand.lastAction.transform.position, WatchObjectsList[leftHand.lastActionIndex].Object.transform.position);
                // Debug.Log(distance.ToString() + "=" + leftHand.lastAction.transform.position.ToString() + "<-->" + WatchObjectsList[leftHand.lastActionIndex].Object.transform.position.ToString());

                goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                if (distance > WatchObjectsList[leftHand.lastActionIndex].PickupDistanceThreshold)
                {
                    Debug.Log("Left Hand Object PICKED UP: " + WatchObjectsList[leftHand.lastActionIndex].ObjectTag);
                    leftHand.handState = HandStates.PickedUp;
                }

                // Just check once as we already know the object index
                break;
            }

            // Perform drop if the object is close and orientation is within tolerance and the robot has stopped moving
            else if (leftHand.handState == HandStates.PickedUp && !leftHand.IsMoving())
            {
                if (WatchObjectsList[leftHand.lastActionIndex].sot.collidedWithTag != "")
                {
                    // Debug.Log("PickedUp State " + WatchObjectsList[leftHand.lastActionIndex].ObjectTag + " --> Now Collided with --> " + WatchObjectsList[leftHand.lastActionIndex].sot.collidedWithTag);

                    for (int j=0; j < SurfaceList.Length; j++)
                    {
                        if (WatchObjectsList[leftHand.lastActionIndex].sot.collidedWithTag == SurfaceList[j].ObjectTag)
                        {
                            Debug.Log("Left Hand Object PLACED DOWN: " + WatchObjectsList[leftHand.lastActionIndex].ObjectTag + " --> " + SurfaceList[j].ObjectTag);

                            goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                            leftHand.lastAction.transform.position = WatchObjectsList[leftHand.lastActionIndex].Object.transform.position;
                            leftHand.lastAction.transform.rotation = WatchObjectsList[leftHand.lastActionIndex].Object.transform.rotation;

                            WatchObjectsList[leftHand.lastActionIndex].Object.transform.SetParent(SurfaceList[j].Object.transform);

                            leftHand.lastActionIndex = j; // Index of Surface

                            leftHand.handState = HandStates.PlacedDown;
                            leftHand.ResetMovingHistory();
                            break;
                        }
                    }
                }

                break;
            }
            else if (leftHand.handState == HandStates.PlacedDown)
            {
                distance = Vector3.Distance(leftHand.actionHand.transform.position, leftHand.lastAction.transform.position);
                if (distance > SurfaceList[leftHand.lastActionIndex].DropDistanceThreshold)
                {
                    Debug.Log("Left Hand Object DROPPED on : " + SurfaceList[leftHand.lastActionIndex].ObjectTag);

                    goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                    leftHand.handState = HandStates.Empty;
                }
            }
        }

        for (int i=0; i< WatchObjectsList.Length ; i++)
        {
            // Check with right hand
            // Check if object is close enough to pick up
            if (rightHand.handState == HandStates.Empty && !rightHand.IsMoving())
            {
                if (WatchObjectsList[i].sot.collidedWithTag == "RGripper")
                {
                    distance = Vector3.Distance(rightHand.actionHand.transform.position, WatchObjectsList[i].Object.transform.position);
                    
                    Debug.Log("Right Hand Object GRABBED: " + WatchObjectsList[i].ObjectTag + " at distance: " + distance);

                    goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;
                    
                    rightHand.lastAction.transform.position = WatchObjectsList[i].Object.transform.position;
                    rightHand.lastAction.transform.rotation = WatchObjectsList[i].Object.transform.rotation;
                    rightHand.lastActionIndex = i;

                    if (WatchObjectsList[i].Object.transform.parent.name == leftHand.Name)
                    {
                        // Debug.Log("Grabbed from left hand");
                        leftHand.handState = HandStates.Empty;
                        leftHand.lastActionIndex = -1;
                    }
                    WatchObjectsList[i].Object.transform.SetParent(rightHand.actionHand.transform);
                    rightHand.handState = HandStates.Grabbed;
                    rightHand.ResetMovingHistory();
                    break;
                }
            }
            // Check if the object far enough from the CT or CB to be considered picked up
            else if (rightHand.handState == HandStates.Grabbed)
            {
                distance = Vector3.Distance(rightHand.lastAction.transform.position, WatchObjectsList[rightHand.lastActionIndex].Object.transform.position);
                // Debug.Log(distance.ToString() + "=" + rightHand.lastAction.transform.position.ToString() + "<-->" + WatchObjectsList[rightHand.lastActionIndex].Object.transform.position.ToString());

                if (distance > WatchObjectsList[i].PickupDistanceThreshold)
                {
                    Debug.Log("Right Hand Object PICKED UP: " + WatchObjectsList[rightHand.lastActionIndex].ObjectTag);

                    goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                    rightHand.handState = HandStates.PickedUp;
                }

                // Just check once as we already know the object index
                break;
            }
            else if (rightHand.handState == HandStates.PickedUp && !rightHand.IsMoving())
            {
                if (WatchObjectsList[rightHand.lastActionIndex].sot.collidedWithTag != "")
                {
                    // Debug.Log("PickedUp State " + WatchObjectsList[rightHand.lastActionIndex].ObjectTag + " --> Now Collided with --> " + WatchObjectsList[rightHand.lastActionIndex].sot.collidedWithTag);

                    for (int j=0; j < SurfaceList.Length; j++)
                    {
                        if (WatchObjectsList[rightHand.lastActionIndex].sot.collidedWithTag == SurfaceList[j].ObjectTag)
                        {
                            Debug.Log("Right Hand Object PLACED DOWN: " + WatchObjectsList[rightHand.lastActionIndex].ObjectTag + " --> " + SurfaceList[j].ObjectTag);

                            goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                            rightHand.lastAction.transform.position = WatchObjectsList[rightHand.lastActionIndex].Object.transform.position;
                            rightHand.lastAction.transform.rotation = WatchObjectsList[rightHand.lastActionIndex].Object.transform.rotation;

                            WatchObjectsList[rightHand.lastActionIndex].Object.transform.SetParent(SurfaceList[j].Object.transform);

                            rightHand.lastActionIndex = j; // Index of Surface

                            rightHand.handState = HandStates.PlacedDown;
                            rightHand.ResetMovingHistory();
                            break;
                        }
                    }
                }

                break;
            }
            else if (rightHand.handState == HandStates.PlacedDown)
            {
                distance = Vector3.Distance(rightHand.actionHand.transform.transform.position, rightHand.lastAction.transform.position);
                if (distance > SurfaceList[rightHand.lastActionIndex].DropDistanceThreshold)
                {
                    Debug.Log("Right Hand Object DROPPED on : " + SurfaceList[rightHand.lastActionIndex].ObjectTag);
                    
                    goImageSynth.GetComponent<ImageSynthesis>().objectManipulated[WatchObjectsList[i].Object.name] = true;

                    rightHand.handState = HandStates.Empty;
                }
            }
        }

        // ** TODO **
        // 3. Perform cutting show and hide after certain number close and far distance has been achieve by the knife
        // 4. Perform pouring if the objects are close and on top and rotation has occured
    } 

    // Randomize the objects placement with a gaussian distribution with the current location as the mean and a standard deviation of tolerance value 
    // Make sure the object does not change vertical up and down position
    // Adding default don't swap object positions because of their rotations, size, and scale can mess up where the objjects end up
    // TODO: Fix the object swapping positions such that objects are always colliding with the surface -- lower until collision with surface is detected
    // TODO: Fix so that the object when moves is not colliding with each other
    // TODO: Fix so that the object when moves is not off of the surface that it will fall on the floor
    public void randomize_objects_placement(bool bSwapObjectPositions = false)
    {
        // Save initial positions
        Vector3[] initPositions = new Vector3[WatchObjectsList.Length];
        if (bSwapObjectPositions)
        {
            for (int i=0; i< WatchObjectsList.Length ; i++)
            {
                initPositions[i] = WatchObjectsList[i].Object.transform.position;
            }

            // Randomize initPosition Array
            for (int i=0; i< WatchObjectsList.Length ; i++)
            {
                int randomIndex = UnityEngine.Random.Range(0, WatchObjectsList.Length);
                Vector3 temp = initPositions[i];
                initPositions[i] = initPositions[randomIndex];
                initPositions[randomIndex] = temp;
            }
        }

        // Assign shuffled positions with random tolerance
        for (int i=0; i< WatchObjectsList.Length ; i++)
        {
            Vector3 randomPosition = new Vector3(
                                                    UnityEngine.Random.Range(-WatchObjectsList[i].PositionRandomizationThreshold, WatchObjectsList[i].PositionRandomizationThreshold), 
                                                    0, 
                                                    UnityEngine.Random.Range(-WatchObjectsList[i].PositionRandomizationThreshold, WatchObjectsList[i].PositionRandomizationThreshold)
                                                );
            if (bSwapObjectPositions)
            {
                WatchObjectsList[i].Object.transform.position = initPositions[i] + randomPosition;
            }
            else
            {
                WatchObjectsList[i].Object.transform.position = WatchObjectsList[i].Object.transform.position + randomPosition;
                // // Swap X and Z but not Y to keep the object on the surface
                // WatchObjectsList[i].Object.transform.position.X = WatchObjectsList[i].Object.transform.position.X + randomPosition.X;
                // // WatchObjectsList[i].Object.transform.position.Y = WatchObjectsList[i].Object.transform.position.Y + randomPosition.Y;
                // WatchObjectsList[i].Object.transform.position.Z = WatchObjectsList[i].Object.transform.position.Z + randomPosition.Z;
            }
        }
        
    } 
}