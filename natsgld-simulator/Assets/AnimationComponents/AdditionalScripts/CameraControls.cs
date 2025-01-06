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
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraControls : MonoBehaviour
{
    // Transforms to act as start and end markers for the journey.
    private Transform nextPose;

    // Total distance between the markers.
    private float journeyLength;

    // Time when the movement started.
    private float startTime;

    //Left and right Banners for different camera angles
    private GameObject banner1;
    private GameObject banner2;

    private Transform cameraPose = null;
    private Transform[] allcams;

    private UnityEngine.UI.Text CameraAngle;

    private string[] sCamAngles;

    private GameObject MainCamera;
    private GameObject campostlist;

    // Start is called before the first frame update
    void Start()
    {
        MainCamera = GameObject.FindGameObjectWithTag("MainCamera");
        banner1 = GameObject.FindGameObjectWithTag("NG_Banner_1");
        banner2 = GameObject.FindGameObjectWithTag("NG_Banner_2");
        CameraAngle = GameObject.FindGameObjectWithTag("Camera_Angle").GetComponent<UnityEngine.UI.Text>();

        campostlist = GameObject.FindGameObjectWithTag("CameraPoseList");
        if (campostlist != null)
        {
            allcams = campostlist.GetComponentsInChildren<Transform>();
        }
        else
        {
            //Debug.LogError("CameraPoseList Not found. Make sure you have a CameraPostList in your scene.");
        }

        if (banner1)
        {
            banner1.SetActive(true);
        }
        else
        {
            //Debug.LogError("Banner 1 not found.");
        }

        //if (banner2)
        //{
        //    banner2.SetActive(false);
        //}
        //else
        //{
        //    //Debug.LogError("Banner 2 not found.");
        //}

        sCamAngles = new string[]
        {
            "front_scene_view",         //0
            "front_right_counter_view", //1
            "left_stove_view",          //2
            "left_stove_view_zoom",     //3
            "right_stove_view",         //4
            "right_stove_view_zoom",    //5
            "top_left_scene_view",      //6
            "back_right_counter_view",  //7
            "top_left_scene_view_zoom", //8
            "front_scene_view_zoom"     //9
        };

    }
    
    public void updateBanner(bool bBanner1) //, bool bBanner2)
    {
        if (banner1)
        {
            if (banner1.activeSelf != bBanner1)
            {
                banner1.SetActive(bBanner1);
            }
        }
        else
        {
            //Debug.LogError("Banner 1 not found.");
        }

        //if (banner2)
        //{
        //    if (banner2.activeSelf != bBanner2)
        //    {
        //        banner2.SetActive(bBanner2);
        //    }

        //}
        //else
        //{
        //    //Debug.LogError("Banner 2 not found.");
        //}
    }

    Transform GetCamera(string sCameraName)
    {
        foreach (Transform cam in allcams)
        {
            if (cam.name.ToLower() == sCameraName.Trim().ToLower())
            {
                return cam;
            }
        }

        return null;
    }

    // Update is called once per frame
    void Update()
    {
        string sCamera = "";

        if (CameraAngle == null)
        {
            //Debug.Log("NG ERROR: CameraAngle object is null. Make sure that CameraAngle in ROS Messages exists and has the correct tag.");
        }
        else
        {
            //ROS
            if (CameraAngle.text.Trim() != "")
            {
                sCamera = CameraAngle.text.Trim();
                CameraAngle.text = "";

                //Debug.Log("ROS recevived - changing camera angle to " + sCamera);

                //cameraPose = GameObject.Find("/CameraPoseList/" + sCamera);
                cameraPose = GetCamera(sCamera);
                if (cameraPose)
                {
                    nextPose = cameraPose;
                    InitalizeJourneyVariables();
                }
                else
                {
                    //Debug.Log("Camera Angle not found - " + sCamera);
                }
            }
        }

        //Key Press
        for (int i = 0; i < sCamAngles.Length; i++)
        {
            if (Input.GetKeyDown(i.ToString()))
            {
                //cameraPose = GameObject.Find("/CameraPoseList/" + sCamAngles[i]);
                sCamera = sCamAngles[i];
                cameraPose = GetCamera(sCamera);
                
                if (cameraPose)
                {
                    nextPose = cameraPose;
                    InitalizeJourneyVariables();
                }
                else
                {
                    //Debug.Log("Unity + MS Sucks! Camera Angle not found - " + sCamera);
                }

                break;
            }
        }

        /*
        if (sCamera != "")
        {
            //Show the correct banner for the corresponding cameraPose
            if (sCamera == "front_scene_view") //0
            {
                updateBanner(true, false);
            }
            else if (sCamera == "front_right_counter_view") //1
            {
                updateBanner(true, false);
            }
            else if (sCamera == "left_stove_view") //2
            {
                updateBanner(false, true);
            }
            else if (sCamera == "left_stove_view_zoom") //3
            {
                updateBanner(true, false);
            }
            else if (sCamera == "right_stove_view") //4
            {
                updateBanner(false, true);
            }
            else if (sCamera == "right_stove_view_zoom") //5
            {
                updateBanner(false, true);
            }
            else if (sCamera == "top_left_scene_view") //6
            {
                updateBanner(false, true);
            }
            else if (sCamera == "back_right_counter_view") //7
            {
                updateBanner(true, false);
            }
            else if (sCamera == "top_left_scene_view_zoom") //8
            {
                updateBanner(true, false);
            }
        }
        */

        if (cameraPose)
        {
            // Distance moved = time * speed.
            float speed = 0.2f;
            float distCovered = (Time.time - startTime) * speed;

            // Fraction of journey completed = current distance divided by total distance.
            if (System.Math.Abs(journeyLength) < System.Single.Epsilon)
            {
                ////Debug.LogError("Possible divide by zero -- fixed.");
                journeyLength = System.Single.Epsilon;
            }

            float fracJourney = distCovered / journeyLength;

            // Set our position as a fraction of the distance between the markers.
            MainCamera.transform.position = Vector3.Lerp(MainCamera.transform.position, nextPose.position, fracJourney);
            MainCamera.transform.rotation = Quaternion.Slerp(MainCamera.transform.rotation, nextPose.rotation, fracJourney);
        }
    }

    private void InitalizeJourneyVariables()
    {
        // Keep a note of the time the movement started.
        this.startTime = Time.time;

        // Calculate the journey length.
        this.journeyLength = Vector3.Distance(MainCamera.transform.position, nextPose.position);
    }
}
