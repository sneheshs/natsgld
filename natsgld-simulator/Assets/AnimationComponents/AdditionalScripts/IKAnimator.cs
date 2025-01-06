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
using IKTarget = IKCommand.IKTarget;
using BioIK;

public class IKAnimator : MonoBehaviour
{
    private const float MOVING_THRESH = 0.00500f;
    public int FRAME_SAMPLE_SIZE = 15;

    // Make sure you put the hands in the right order in the inspector according to the enum
    // E.g. if Left = 0 and Right = 1, then put left first and right second

    private List<Transform> bodyTransforms = new List<Transform>();
    private List<GameObject> targets = new List<GameObject>();

    public IKAnimation currentAnimation;
    //private Transform currentCamera;

    // Testing Mode to run pretrained model prediction tests
    public bool TestingMode = false;
    public String TestingModeCommand = "";

    public String AnimStartStopStatus = "STOPPED";

    // Moving variables
    private LinkedList<Vector3> locationHistory;
    private float distanceTotal;

    // Backing-up variables
    private bool backingUp;
    private BackupAssignment[] backupTransforms;

    public string[] animationInputMappings;
    public IKAnimation[] animationList;
    //public List<Transform> poseList;
    //public CameraControls camera;

    private UnityEngine.UI.Text Status;
    private UnityEngine.UI.Text AnimationControl;
    private UnityEngine.UI.Text BaxterFaceAnimation;

    private GameObject Baxter_face_normal_blinking;
    private GameObject Baxter_face_confused;
    private string currentBaxterFace = "";

    private UnityEngine.UI.Text PoseControlLeft;
    private string[] valuesPoseControlLeft;
    private UnityEngine.UI.Text PoseControlRight;
    private string[] valuesPoseControlRight;
    private UnityEngine.UI.Text PoseControlStartStop;
    private string[] valuesPoseControlStartStop;
    public float motion_scaling_factor = 1.0f;
    private TMPro.TextMeshProUGUI OZ_Banner;
    System.Random random = new System.Random();
    private string[] objectslist = new string[] { "Onion", "Tomato", "Celery", "Potato", "Carrot"}; //"Knife",

    // private GameObject MagicPoof;

    private GameObject parentObj;
    private GameObject childObj;
    private bool ObjStatus;

    private GameObject parentSoup;
    private GameObject childSoup;

    Dictionary<string, GameObject> shower;
    Dictionary<string, GameObject> hider;
    string[] arr;
    int counter = 0;


    private bool isTotalPaused;
    private Dictionary<string, IKAnimation> animationDictionary = new Dictionary<string, IKAnimation>();

    private bool loading = false;

    public UdpSocket udpSocket;

    // private IKAnimation ika;
    public List<GameObject> bodyparts = new List<GameObject>();
    public List<BioJoint.Motion> bioJointMotions = new List<BioJoint.Motion>();

    void Start()
    {
        if (animationInputMappings.Length != animationList.Length)
        {
            throw new Exception("Animation Mapping and List are not the same length");
        }

        locationHistory = new LinkedList<Vector3>();
        backupTransforms = new BackupAssignment[Enum.GetValues(typeof(IKTarget)).Length];
        backingUp = false;

        // //Debug.Log("ALL ANIMATIONS UNPAUSED");
        isTotalPaused = false;
        //currentAnimation = null;
        //currentCamera = CameraControls.gameObject.transform;

        if (currentAnimation)
        {
            if (currentAnimation != null)
            {
                currentAnimation.Initialize(this);
            }
        }
        //else
        //{
        //    //Debug.Log("Cannot Start. No animation selected.");
        //}

        //indCurrentImage = 0;
        //FacialExpressionInProgress = false;
         // List<Transform> bodyTransforms = new List<Transform>();

        bodyTransforms.Add(GameObject.FindGameObjectWithTag("LGripPos").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("RGripPos").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("Baxter_leftArm_joint4").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("Baxter_rightArm_joint4").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("Baxter_monitor").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("Baxter_base_visible").transform);
        bodyTransforms.Add(GameObject.FindGameObjectWithTag("Baxter_upperBody").transform);

        targets.Add(GameObject.FindGameObjectWithTag("LHTarget"));
        targets.Add(GameObject.FindGameObjectWithTag("RHTarget"));
        targets.Add(GameObject.FindGameObjectWithTag("LShoulderTarget"));
        targets.Add(GameObject.FindGameObjectWithTag("RShoulderTarget"));
        targets.Add(GameObject.FindGameObjectWithTag("LookTarget"));
        targets.Add(GameObject.FindGameObjectWithTag("BaseTarget"));
        // targets.Add(GameObject.FindGameObjectWithTag("Baxter Hip Target Should Go Here If Needed"));


        Baxter_face_confused = GameObject.FindGameObjectWithTag("Baxter_face_confused");
        Baxter_face_normal_blinking = GameObject.FindGameObjectWithTag("Baxter_face_normal_blinking");
        ChangeBaxterFace(true, false);

        // MagicPoof = GameObject.FindGameObjectWithTag("MagicPoof");
        // if (MagicPoof != null)
        // {
        //     MagicPoof.SetActive(false);
        // }
        
        Status = GameObject.FindGameObjectWithTag("Animation_Status").GetComponent<UnityEngine.UI.Text>();
        AnimationControl =  GameObject.FindGameObjectWithTag("Animation_Control").GetComponent<UnityEngine.UI.Text>();
        BaxterFaceAnimation = GameObject.FindGameObjectWithTag("Baxter_Face_Animation").GetComponent<UnityEngine.UI.Text>();
        PoseControlLeft = GameObject.FindGameObjectWithTag("PoseControlLeft").GetComponent<UnityEngine.UI.Text>();
        PoseControlRight = GameObject.FindGameObjectWithTag("PoseControlRight").GetComponent<UnityEngine.UI.Text>();
        PoseControlStartStop = GameObject.FindGameObjectWithTag("PoseControlStartStop").GetComponent<UnityEngine.UI.Text>();
        OZ_Banner = GameObject.FindGameObjectWithTag("OZ_Banner").GetComponent<TMPro.TextMeshProUGUI>();

        parentObj = GameObject.FindGameObjectWithTag("InPot");
        if (parentObj != null)
        {
            // //Debug.Log("Hiding ..." + parentObj.name);

            childObj = parentObj;
            childObj.SetActive(false);
            ObjStatus = false;
        }

        parentSoup = GameObject.FindGameObjectWithTag("InBowl");
        if (parentSoup != null)
        {
            // //Debug.Log("Hiding ..." + parentSoup.name);

            childSoup = parentSoup;
            childSoup.SetActive(false);
            ObjStatus = false;
        }
        shower = new Dictionary<string, GameObject>();
        hider = new Dictionary<string, GameObject>();
        //arr = new ArrayList() { "Tomato", "Knife", "Board" };
        arr = UnityEditorInternal.InternalEditorUtility.tags;
        /*
        foreach (string s in arr)
        {
            //Debug.Log(s);
        }
        */
        foreach (string s in arr)
        {
            shower.Add(s, GameObject.FindGameObjectWithTag(s));
        }

        foreach (string s in arr)
        {
            hider.Add(s, shower[s]);
        }

        string[] headers = new string[] {
            "Baxter_rotationJoint",
            "Baxter_leftArm_joint1","Baxter_leftArm_joint2",
            "Baxter_leftArm_joint3", "Baxter_leftArm_joint4", 
            "Baxter_leftArm_joint5", "Baxter_leftArm_joint6", "Baxter_leftArm_joint7",
            "Baxter_monitorJoint",
            "Baxter_rightArm_joint1", "Baxter_rightArm_joint2", 
            "Baxter_rightArm_joint3", "Baxter_rightArm_joint4", 
            "Baxter_rightArm_joint5", "Baxter_rightArm_joint6", "Baxter_rightArm_joint7"
        };


        GameObject go;
        foreach (string header in headers)
        {
            go = GameObject.Find(header);
            bodyparts.Add(go);

            if (go.GetComponent<BioJoint>().X.Enabled)
            {
                bioJointMotions.Add(go.GetComponent<BioJoint>().X);
            }
            else if (go.GetComponent<BioJoint>().Y.Enabled)
            {
                bioJointMotions.Add(go.GetComponent<BioJoint>().Y);
            }
            else if (go.GetComponent<BioJoint>().Z.Enabled)
            {
                bioJointMotions.Add(go.GetComponent<BioJoint>().Z);
            }
        }

        // udpSocket = GameObject.FindGameObjectWithTag("AnimationContainer").GetComponent<UdpSocket>();
        udpSocket = GameObject.FindGameObjectWithTag("AnimationContainer").AddComponent<UdpSocket>();
    }

    void ChangeBaxterFace(bool blink, bool confused) //, bool wait, bool stare)
    {
        Baxter_face_normal_blinking.SetActive(blink);
        Baxter_face_confused.SetActive(confused);
    }

    public void showhide(string show, string hide)
    {
        // //Debug.Log("Show: "+ show);
        // //Debug.Log("hide: " + hide);
        if(hider[hide])
            hider[hide].SetActive(false);
        if (shower[show])
            shower[show].SetActive(true);
        
    }
    
    string tmpMessage = "";
    string[] tmpMsgParsed;
    public string curAnimationControl = "";
    string[] jointTargetValues;
    void Update()
    {
        curAnimationControl = "";
        ////Debug.Log("Loading = " + loading.ToString());
        

        //if (Input.GetKeyDown("escape"))
        //{
        //    if (isTotalPaused)
        //    {
        //        //Debug.Log("ALL ANIMATIONS PAUSED");
        //        isTotalPaused = true;
        //    }
        //    else
        //    {
        //        //Debug.Log("ALL ANIMATIONS UNPAUSED");
        //        isTotalPaused = false;
        //        if (currentAnimation != null)
        //        {
        //            currentAnimation.GetCurrentCommand().Initialize(this);
        //        }
        //    }
        //    currentAnimation = null;
        //}
        //else
        //{}
        if (Input.GetKeyDown(KeyCode.Backspace))
        {
            showhide((string)arr[counter],(string)arr[(counter+1)]);
            counter++;
        }


        if (TestingMode)
        {
            if (udpSocket.ReceivedMessage != "")
            {
                if (udpSocket.ReceivedMessage == "SYNC")
                {
                    print("[IK ANIM SENT] -->> SYNC");
                    udpSocket.SendData("SYNC");
                    udpSocket.ReceivedMessage = "";
                }
                else
                {
                    tmpMessage = udpSocket.ReceivedMessage;
                    tmpMsgParsed = tmpMessage.Split(',');
                    TestingModeCommand = tmpMsgParsed[0];
                    udpSocket.ReceivedMessage = "";
                }
            }

            // Return Format [ResponseTo,BaxterStates,ImageName,StateUpdate]
            if (TestingModeCommand == "")
            {}

            // From run_test
            else if (TestingModeCommand == "START")
            {
                Debug.Log("[STARTED]: " + tmpMsgParsed[1]);
                // udpSocket.SendData("STARTED");
                AnimStartStopStatus = "STARTED";
                TestingModeCommand = "";
            }

            // From run_test
            else if (TestingModeCommand == "STOP")
            {
                Debug.Log("[STOPPED]: " + tmpMsgParsed[1]);
                udpSocket.SendData("STOPPED");
                AnimStartStopStatus = "STOPPED";
                TestingModeCommand = "";
            }

            // From run_test
            else if (TestingModeCommand == "UPDATE")
            {
                Debug.Log("[UPDATE]: " + tmpMsgParsed[1]);
                AnimStartStopStatus = "UPDATENOW";

                // DO THE BAXTER STATE UPDATE THEN SAVE STATE
                jointTargetValues = tmpMsgParsed[1].Substring(1, tmpMsgParsed[1].Trim().Length - 2).Split(' ');
                for(int i =0; i < bodyparts.Count; i++)
                {
                    bioJointMotions[i].TargetValue = float.Parse(jointTargetValues[i].Trim());
                }

                TestingModeCommand = "";
            }

            // From ImageSynth
            else if (TestingModeCommand == "UPDATED")
            {
                AnimStartStopStatus = "UPDATED";

                // Reply to the Run_Test
                // Debug.Log("UPDATED COMMAND SENT TO RUN_TEST");

                TestingModeCommand = "";
            }
            else
            {
                Debug.Log("GOT A WEIRD COMMAND: " + TestingModeCommand);
                TestingModeCommand = "";
            }
        }

        
        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (AnimStartStopStatus == "STOPPED")
            {
                AnimStartStopStatus = "STARTED";
            }
            else
            {
                AnimStartStopStatus = "STOPPED";
            }

            Debug.Log("Animation Status: " + AnimStartStopStatus);
        }

        for (int i = 0; i < animationInputMappings.Length; i++)
        {
            if (Input.GetKeyDown(animationInputMappings[i]))
            {
                currentAnimation = animationList[i];
                currentAnimation.index = 0;
                currentAnimation.bFirstTime = true ;
                // //Debug.Log("Animation: " + currentAnimation + " started!");
                //if (currentAnimation.ToString()== "NodAnimation (IKAnimation)")
                //{
                //   try {
                //        GameObject.FindGameObjectWithTag("NG_Banner_1").SetActive(false);
                //    }
                //    catch 
                //    {
                //    }
                //}

                //currentCamera = poseList[i];
                //currentCamera.Update(animationInputMappings[i]);
                currentAnimation.Initialize(this);
                //b_StatusChange = false;
                break;
            }
        }

        if (AnimationControl == null)
        {
            //Debug.LogError("SOMETHING WRONG HERE...");
        }

        //ROS Control for start of animation
        if (AnimationControl.text != "")
        {
            curAnimationControl = AnimationControl.text;
            // //Debug.Log("Received ROS Anim Control - " + curAnimationControl);
            AnimationControl.text = "";

            //If NodCommand is received, that means robot understood, so hide instruction banner
            //if(curAnimationControl.Trim() == "NodAnimation")
            //{
            //    GameObject.FindGameObjectWithTag("NG_Banner_1").SetActive(false);
            //}
            //GameObject.Find(curAnimationControl).

            //ika = new IKAnimation(); // TODO: Memory leak after each animation the IK animation needs to be destroyed or need a way to track already created IK Animation
            //ika.commandObj = GameObject.Find(curAnimationControl);
            //currentAnimation = ika;
            //currentAnimation.index = 0;
            //currentAnimation.bFirstTime = true;
            //currentAnimation.status = "";
            //currentAnimation.Initialize(this);



            for (int i = 0; i < animationInputMappings.Length; i++)
            {
                // //Debug.Log(animationList[i].ToString() + " - "  + curAnimationControl);
                if (animationList[i].ToString().StartsWith(curAnimationControl))
                {
                    currentAnimation = animationList[i];
                    currentAnimation.index = 0;
                    currentAnimation.bFirstTime = true;
                    currentAnimation.status = "";
                    currentAnimation.Initialize(this);
                    break;
                }
            }
        }

        // if (Input.GetKeyDown(KeyCode.Return))
        // {
        //     MagicPoof.SetActive(!MagicPoof.activeSelf);
        // }

        // ROS Pose Control for left and right hands
        if (PoseControlLeft.text != "")
        {
            valuesPoseControlLeft = PoseControlLeft.text.Split(' ');
            PoseControlLeft.text = "";
            targets[0].transform.position += new Vector3(
                                            motion_scaling_factor * float.Parse(valuesPoseControlLeft[1]), 
                                            motion_scaling_factor * float.Parse(valuesPoseControlLeft[2]), 
                                            motion_scaling_factor * float.Parse(valuesPoseControlLeft[3]));
        }
        
        if (PoseControlRight.text != "")
        {
            valuesPoseControlRight = PoseControlRight.text.Split(' ');
            PoseControlRight.text = "";
            targets[1].transform.position += new Vector3(
                                            motion_scaling_factor * float.Parse(valuesPoseControlRight[1]), 
                                            motion_scaling_factor * float.Parse(valuesPoseControlRight[2]), 
                                            motion_scaling_factor * float.Parse(valuesPoseControlRight[3]));
        }

        if (PoseControlStartStop.text != "")
        {
            valuesPoseControlStartStop = PoseControlStartStop.text.Split(' ');
            PoseControlStartStop.text = "";
            
            // SAVE START AND STOP STATE IN THE CSV
            if (AnimStartStopStatus == "STOPPED")
            {
                AnimStartStopStatus = "STARTED";
                OZ_Banner.text = objectslist[random.Next(0, objectslist.Length)];
            }
            else
            {
                AnimStartStopStatus = "STOPPED";
                OZ_Banner.text += " Done";
            }
        }

        if (BaxterFaceAnimation.text != "")
        {
            currentBaxterFace = BaxterFaceAnimation.text;
            // //Debug.Log("Received ROS BaxterFaceAnimation - " + currentBaxterFace);
            BaxterFaceAnimation.text = "";

            switch (currentBaxterFace)
            {
                // case "Magic_poof_show":
                //     MagicPoof.SetActive(true);
                //     break;
                // case "Magic_poof_hide":
                //     MagicPoof.SetActive(false);
                //     break;

                case "Baxter_face_confused":
                    ChangeBaxterFace(false, true);
                    break;
                case "Baxter_face_normal_blinking":
                default:
                    ChangeBaxterFace(true, false);
                    break;
            }
        }
        if(currentAnimation != null && ObjStatus == false)
        { 
            if (currentAnimation.completedAnimation == "PourToPanCommand2 (MoveCommand)" ||
                currentAnimation.completedAnimation == "PourFromBoardToPotCommand2 (MoveCommand)" ||
                currentAnimation.completedAnimation == "ShallowPourCommand (MoveCommand)")
            {
                GameObject.FindGameObjectWithTag("OnBoard").SetActive(false);
                if (parentObj != null)
                {
                    // //Debug.Log("Showing ..." + parentObj.name);

                    parentObj.SetActive(true);
                    ObjStatus = true;
                }
            }

            if (currentAnimation.completedAnimation == "PourUsingLadleCommand (MoveCommand)" || currentAnimation.completedAnimation == "OiltoPanCommand (MoveCommand) 5")
            {
                if (parentSoup != null)
                {
                    // //Debug.Log("Showing ..." + parentSoup.name);

                    parentSoup.SetActive(true);
                    ObjStatus = true;
                }
            }
        }
        ////ROS Control for baxter face animation
        //if (BaxterFaceAnimation.text != "")
        //{
        //    curBaxterFaceAnimation = BaxterFaceAnimation.text;
        //    //Debug.Log("Received ROS Baxter Face Animation - " + curBaxterFaceAnimation);
        //    BaxterFaceAnimation.text = "";

        //    switch(curBaxterFaceAnimation)
        //    {
        //        case "Baxter_face_confused":
        //            Baxter_face_normal_blinking.SetActive(false);
        //            Baxter_face_confused.SetActive(true);
        //            break;
        //        case "Baxter_face_normal_blinking":
        //        default:
        //            Baxter_face_normal_blinking.SetActive(true);
        //            Baxter_face_confused.SetActive(false);
        //            break;
        //    }
        //}

        // ROS Notification of animation completed.
        if (currentAnimation != null)
        {
            if (currentAnimation.status != "")
            {
                // //Debug.Log(currentAnimation.status);
                Status.text = currentAnimation.status;
                currentAnimation.status = "";
            }
        }

        if (!loading)
            UpdateAnimation();
    }

    public void updateLoading(bool bStatus)
    {
        loading = bStatus;
    }

    private void UpdateAnimation()
    {
        if (currentAnimation == null || isTotalPaused) { return; }

       // if (!InitializeLocation()) {
        //    return;
        //}

        // For movables, call UpdateMovingData() so that calls to IsMoving() will be valid
        Movable movable = currentAnimation.GetCurrentCommand() as Movable;
        if (movable != null) {
            UpdateMovingData(movable);
        }

        // Animation will sometime pause to back up
        if (backingUp)
        {
            // As of now, only movable commands can back up
            if (!IsMoving(movable))
            {
                //Debug.Log("Backup Stop");
                // Re-initialize command on unpause
                currentAnimation.GetCurrentCommand().Initialize(this);
                backingUp = false;
                ResetMovingData();
            }
        }
        else if (!loading)
        {
            currentAnimation.Animate(this);
        }
        else { }
    }

   
    
    public bool InitializeLocation()
    {
        Vector3 currentAnimPos = currentAnimation.getBaseTransform().position;
        float currentAnimRot = currentAnimation.getBaseTransform().eulerAngles.y;
        bool atPos = true;

        if (Vector3.Distance(currentAnimPos, transform.position) > 0.1)
        {
            transform.position = Vector3.Lerp(transform.position, currentAnimPos, Time.deltaTime);
            atPos = false;
        }

        if (Mathf.Abs(Mathf.DeltaAngle(currentAnimRot, transform.eulerAngles.y)) > 5)
        {
            transform.eulerAngles = new Vector3(transform.eulerAngles.x, Mathf.LerpAngle(transform.eulerAngles.y, currentAnimRot, Time.deltaTime / 3), transform.eulerAngles.z);
            atPos = false;
        }

        return atPos;
    }

    // Callback when command finishes
    public void CommandStarting()
    {
        ResetMovingData();

        // Before starting a command, store the initial transforms of all targets so we can back up to them
        IKTarget[] targets = Enum.GetValues(typeof(IKTarget)) as IKTarget[];

        for (int i = 0; i < targets.Length; i++) {
            IKTarget target = targets[i];

            BackupAssignment ba = new BackupAssignment();
            ba.ikTarget = target;
            ba.targetTransform = GetBodyTransform(target);
            backupTransforms[i] = ba;
        }
    }

    public int iBackupCounter = 0;
    public void Backup()
    {
        Vector3 random;
        iBackupCounter++;

        foreach (BackupAssignment ba in backupTransforms)
        {
            if (iBackupCounter < 3)
            {
                // Add randomized offset to increase chances of backing up working
                random = UnityEngine.Random.insideUnitSphere * 0.1f;
            }
            else if (iBackupCounter < 6)
            {
                random = UnityEngine.Random.insideUnitSphere * 0.2f;
            }
            else if (iBackupCounter < 9)
            {
                random = UnityEngine.Random.insideUnitSphere * 0.3f;
            }
            else
            {
                random = UnityEngine.Random.insideUnitSphere * 0.5f;
            }

            if (iBackupCounter > 25)
            {
                iBackupCounter = 0;
            }

            // Make sure the backup position is upwards only
            random.y = Mathf.Abs(random.y);

            GetTarget(ba.ikTarget).position = ba.targetTransform.position + random;
            GetTarget(ba.ikTarget).rotation = ba.targetTransform.rotation;
        }

        ResetMovingData();
        backingUp = true;
    }

    public Transform GetBodyTransform(IKTarget target)
    {
        return bodyTransforms[(int)target];
    }

    public Transform GetTarget(IKTarget target)
    {
        return targets[(int)target].transform;
    }

    public GameObject GetBaxter()
    {
        // No longer used. Old method to get baxter_base

        GameObject bax_base = GameObject.FindGameObjectWithTag("Baxter_base");
        if (bax_base != null)
            return bax_base;
        else
        {
            //Debug.LogError("Baxter_base TAG not found.");
            return null;
        }
    }

    // Movable parameter currently unused - keeping it for now to indicate that you must have a movable to call isMoving
    public bool IsMoving(Movable movable)
    {
        if (locationHistory.Count < FRAME_SAMPLE_SIZE) {
            return true;
        }

        // Average out the distance moved per frame
        float avgDist = distanceTotal / (FRAME_SAMPLE_SIZE - 1);

        // If the average distance moved is greater than the threshold we are moving
        return (avgDist > MOVING_THRESH);
    }

    public void ResetMovingData()
    {
        locationHistory.Clear();
        distanceTotal = 0;
    }

    private void UpdateMovingData(Movable movable)
    {
        Vector3 newPosition = movable.GetAnimationPosition(this);

        // Start by adding the new position to the running total / queue
        if (locationHistory.Count > 0) {
            distanceTotal += Vector3.Distance(newPosition, locationHistory.First.Value);
        }
        locationHistory.AddFirst(newPosition);

        // Wait until the given sample size of positions has been collected
        if (locationHistory.Count > FRAME_SAMPLE_SIZE) {
            // Remove last distance from total / queue
            Vector3 last = locationHistory.Last.Value;
            locationHistory.RemoveLast();
            Vector3 secondLast = locationHistory.Last.Value;

            distanceTotal -= Vector3.Distance(last, secondLast);
        }
    }

    private void ToggleTarget(string joint_name, bool enable)
    {
        BioIK.BioSegment segment = GetBaxter().GetComponent<BioIK.BioIK>().FindSegment(joint_name);
        if (segment == null)
        {
            //Debug.Log("Joint name " + joint_name + " - Segment is null");
        }
        else
        {
            if (segment.Objectives != null && segment.Objectives.Length > 0)
            {
                for (int i=0; i< segment.Objectives.Length; i++) {
                    segment.Objectives[i].enabled = enable;
                }
            }
            else
            {
                //Debug.LogError("Joint " + joint_name + " not found.");
            }
        }
    }

    public void ToggleTarget(IKTarget target, bool enable)
    {
        switch(target)
        {
            case IKTarget.LeftHand:
                ToggleTarget("LGripPos", enable);
                break;
            case IKTarget.RightHand:
                ToggleTarget("RGripPos", enable);
                break;
            case IKTarget.LeftShoulder:
                ToggleTarget("Baxter_leftArm_joint4", enable);
                break;
            case IKTarget.RightShoulder:
                ToggleTarget("Baxter_rightArm_joint4", enable);
                break;
           case IKTarget.Head:
                ToggleTarget("Baxter_monitor", enable);
                break;
           case IKTarget.Base:
                ToggleTarget("Baxter_base_visible", enable);
                break;
          /* case IKTarget.Hip:
                //ToggleTarget("Baxter_rotationJoint", enable);
                ToggleTarget("Baxter_upperBody", enable);
                break;
                */
        }
    }

    private void ToggleJoint(string joint_name, BioIK.BioIK character, bool isEnable)
    {
        BioIK.BioSegment segment = GetBaxter().GetComponent<BioIK.BioIK>().FindSegment(joint_name);

        if (segment.Joint != null)
        {
            segment.Joint.enabled = isEnable;
        }
    }

    private void ToggleChildJoints(string joint_name, BioIK.BioIK character, bool isEnable)
    {
        BioIK.BioSegment segment = character.FindSegment(joint_name);

        if (segment.Joint != null)
        {
            /*
            if (isEnable) {
                //Debug.Log("Enabling Segment: " + segment.transform.name);
            } else {
                //Debug.Log("Disableing Segment: " + segment.transform.name);
            }
            segment.Joint.enabled = isEnable;
            */
        }

        for (int i = 0; i < segment.Childs.Length; i++)
        {
            ToggleChildJoints(segment.Childs[i].transform.name, character, isEnable);
        }
    }

    [Serializable]
    public struct BackupAssignment
    {
        public Transform targetTransform;
        public IKTarget ikTarget;
    }
}
