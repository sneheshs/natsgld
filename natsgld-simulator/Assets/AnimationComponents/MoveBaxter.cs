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


public class MoveBaxter : MonoBehaviour
{

    private Animator anim;

    private UnityEngine.UI.Text Status;
    private UnityEngine.UI.Text AnimationControl;
    private string curAnimationControl = "";
    private bool bStarted = false;

    private bool baxMissing = false;

    private GameObject bax;
    private GameObject baxOG;
   
    private float journeyLength;
    private float rotationLength;
    
    
    private List<BaxterPosition> allPositions;
    //private string key;
    private BaxterPosition currentPosition;

    //private string[] keys;
    int count = 0;

    // Time when the movement started.
    private float startTime;
    private bool bStartMove = false;


    // Start is called before the first frame update
    void Start()
    {
        bool bax_fail = true;
        bool baxOG_fail = true;

        bax = GameObject.FindGameObjectWithTag("Baxter");

        if (bax != null)
        {
            anim = bax.GetComponent<Animator>();
            if (anim != null)
            {
                bax_fail = false;
            }
        }

        if (bax_fail)
        {
            baxOG = GameObject.FindGameObjectWithTag("BaxterOG");
            
            if (baxOG != null)
            {
                anim = baxOG.GetComponent<Animator>();
                if (anim != null)
                {
                    baxOG_fail = false;
                }
            }
        }
        


        if (bax_fail && baxOG_fail)
        {
            baxMissing = true;
            ////Debug.LogError("ERROR: Check the following possible issues: Missing Tag or Animator...\n\n- Baxter Tag is missing \n- Baxter is missing an Animator\n- BaxterOG Tag is missing \n- BaxterOG is missing an Animator.");
        }
        
        Status = GameObject.FindGameObjectWithTag("Animation_Status_Move").GetComponent<UnityEngine.UI.Text>();
        AnimationControl = GameObject.FindGameObjectWithTag("Animation_Control_Move").GetComponent<UnityEngine.UI.Text>();

        //keys = new string[]
        //{
        // InFrontofPan --> OilToPan, TwistPepper, PotTurn(On/Off)Gas, 
        // InFrontofPot --> ShallowPour, PickUpPotPan, PourUsingLadle, PourFromBoardToPan, Stirring, PanTurn(On/Off)Gas
        // ForFetch -->
        // DefaultPosition --> Cutting, DropBowl, DrobBoard, 
        // InFrontofLid --> Shallow Pour, PourFromBoardToPot
        // BetweenPanandPot --> OiltoPan
        // InFrontofSink --> DropPotPan
        // InFrontofBowl --> PickUpBowl
        //};

        allPositions = new List<BaxterPosition>();
        // allPositions.Add(new BaxterPosition("r", "InFrontOfPan",     new Vector3(1.7f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("t", "InFrontOfPot",     new Vector3(1.0f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("y", "InFrontOfLid",     new Vector3(0.7f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("u", "InFrontOfVeggies", new Vector3(-1.0f, 0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("i", "InFrontOfBoard",   new Vector3(0.132f,0.7989502f, 0.4149154f),    new Vector3(0f, 90f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("o", "InFrontOfSink",    new Vector3(1.4f,  0.7989502f, 0.4149154f),    new Vector3(0f, 90f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("p", "InFrontOfBowl",    new Vector3(-0.2f, 0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        // allPositions.Add(new BaxterPosition("l", "BetweenPanAndPot", new Vector3(1.4f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));

        string[] tmp;

        // DUPLLCATES FROM LEGACY
        tmp = new string[] {"InFrontOfPan", "TurnToBackStoveAnimation", "TurnToBackAddSaltAnimation", "TurnFromBoardToPanAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("q", itm,   new Vector3(1.7f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }

        tmp = new string[] {"BetweenPanAndPot", "TurnToBackStirringAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("w", itm,   new Vector3(1.4f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }

        tmp = new string[] {"InFrontOfPot", "TurnToBackBasilAnimation", "TurnFromBoardToPotAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("e", itm,   new Vector3(1.0f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }

        tmp = new string[] {"InFrontOfLid"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("r", itm,   new Vector3(0.7f,  0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }

        tmp = new string[] {"InFrontOfBowl", "TurnToPourAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("t", itm,   new Vector3(-0.2f, 0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }

        tmp = new string[] {"InFrontOfVeggies", "TurnToBackFetchAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("u", itm,   new Vector3(-1.0f, 0.7989502f, 0.4149154f),    new Vector3(0f, 270f, 0f), 0.3f));
        }


        tmp = new string[] {"InFrontOfSink", "TurnToBackCleanAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("[", itm,   new Vector3(1.4f,  0.7989502f, 0.4149154f),    new Vector3(0f, 90f, 0f), 0.3f));
        }

        tmp = new string[] {"InFrontOfBoard", "TurnToFrontStoveAnimation", "TurnToFrontFetchAnimation", "MoveBackAfterCleanUpAnimation"};
        foreach (var itm in tmp) {
            allPositions.Add(new BaxterPosition("]", itm,   new Vector3(0.132f,0.7989502f, 0.4149154f),    new Vector3(0f, 90f, 0f), 0.3f));
        }
    }

    void startMovement(string movementName)
    {
        //Debug.Log("Starting " + movementName);
        InitalizeJourneyVariables();
    }

    // Update is called once per frame
    void Update()
    {
        if (baxMissing)
            return;

        //if (Input.GetKeyDown(KeyCode.Tab))
        //{
        //    bStartMove = true;
        //    pos = positions[count];
        //    //Debug.Log(" NUMBER " + positions[count]);
        //    if (count < 2)
        //        count++;
        //    else
        //        count = 2;
        //    InitalizeJourneyVariables();
        //}


        // INPUT KEYS
        for(int i = 0; i<allPositions.Count; i++)
        {
            if (Input.GetKeyDown(allPositions[i].key)){

                bStartMove = true;
                currentPosition = allPositions[i];
                curAnimationControl = currentPosition.positionName;
                startMovement(currentPosition.positionName);
            }
        }

        // ROS
        if (AnimationControl.text != "")
        {
            curAnimationControl = AnimationControl.text;
            //Debug.Log("Received Animation Control MoveBaxter - " + curAnimationControl);
            AnimationControl.text = "";

            foreach (BaxterPosition _p in allPositions)
            {
                if (_p.positionName == curAnimationControl.Trim())
                {
                    bStartMove = true;
                    currentPosition = _p;
                    startMovement(currentPosition.positionName);
                }
            }
        }

        //Move baxter around using Vector3.lerp and Quaternion.Slerp. TAB is the key associated with it as seen in the if statement above.
        if (bStartMove)
        {
            float distCovered = (Time.time - startTime) * currentPosition.speed;

            // Fraction of journey completed = current distance divided by total distance.
            if (System.Math.Abs(journeyLength) < System.Single.Epsilon)
            {
                journeyLength = System.Single.Epsilon; ////Debug.LogError("Possible divide by zero -- fixed.");
            }

            float fracJourney = distCovered / journeyLength;
            float rotJourney = distCovered / rotationLength;

            // Set our position as a fraction of the distance between the markers.
            baxOG.transform.position = Vector3.Lerp(baxOG.transform.position, currentPosition.targetMove, fracJourney);
            baxOG.transform.rotation = Quaternion.Slerp(baxOG.transform.rotation, Quaternion.Euler(currentPosition.targetRotate), fracJourney);

            //bax.transform.position = Vector3.MoveTowards(transform.position, target, Time.deltaTime * speed);
            //DONE
            if (Vector3.Distance(baxOG.transform.position, currentPosition.targetMove) < .01)
            {
                bStartMove = false;
            }
        }

        //Reset Status
        if (Status.text != "")
        {
            Status.text = "";
        }

        //Done
        if (curAnimationControl == currentPosition.positionName && !bStartMove)
        {
            Status.text = "0, COMPLETED - " + curAnimationControl;
            //Debug.Log(Status.text);
            curAnimationControl = "";
        }


        /*
        //ROS
        if (AnimationControl.text != "")
        {
            curAnimationControl = AnimationControl.text;
            //Debug.Log("Received Animation Control MoveBaxter - " + curAnimationControl);
            AnimationControl.text = "";
            anim.Play(curAnimationControl);
        }

        //Started
        if (anim.GetCurrentAnimatorStateInfo(0).IsName(curAnimationControl))
        {
            bStarted = true;
        }

        //Reset Status
        if (Status.text != "")
        {
            Status.text = "";
        }

        //Done
        if (!anim.GetCurrentAnimatorStateInfo(0).IsName(curAnimationControl) && bStarted)
        {
            Status.text = "0, COMPLETED - "+ curAnimationControl;
            //Debug.Log(Status.text);
            bStarted = false;
        }
        */


    }
    //Main use of this function is the initialization of time when we start.
    //We also initilize the length between the start and end position.
    private void InitalizeJourneyVariables()
    {
        // Keep a note of the time the movement started.
        this.startTime = Time.time;

        // Calculate the journey length.
        this.journeyLength = Vector3.Distance(baxOG.transform.position, currentPosition.targetMove);
        this.rotationLength = Quaternion.Angle(baxOG.transform.rotation, Quaternion.Euler(currentPosition.targetRotate));

    }

    [Serializable]
    public struct BaxterPosition
    {
        public string key;
        public string positionName;
        public Vector3 targetMove;
        public Vector3 targetRotate;
        public float speed;

        public BaxterPosition(string _k, string _pn, Vector3 _tm, Vector3 _tr, float _s)
        {
            key = _k;
            positionName = _pn;
            targetMove = _tm;
            targetRotate = _tr;
            speed = _s;
        }
    }
}
