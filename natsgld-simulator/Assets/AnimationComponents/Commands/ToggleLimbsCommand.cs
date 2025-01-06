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

public class ToggleLimbsCommand : IKCommand
{
    public bool enableRightArm;
    public bool enableLeftArm;
    public bool enableHead;
    public bool enableBase;
    //public bool enableHip;

    public List<string> disableJointList;
    public List<string> enableJoinList;


    public ToggleLimbsCommand()
    {
        enableRightArm = true;
        enableLeftArm = true;
        enableHead = true;
        enableBase = true;
        //enableHip = true;

        disableJointList = null;
        enableJoinList = null;

    }

    public override void Initialize(IKAnimator anim)
    {
        // nothing to initalize
    }

    public override bool Execute(IKAnimator anim)
    {
        EnableBase(anim, enableBase);
        //EnableHip(anim, enableHip);
        EnableRightArm(anim, enableRightArm);
        EnableLeftArm(anim, enableLeftArm);
        EnableHead(anim, enableHead);

        if (disableJointList.Count > 0) { 
            //Debug.Log("Disabling List");
        }
        for (int i = 0; i < disableJointList.Count; i++)
        {
            EnableJoint(anim, disableJointList[i], false);
        }

        if (enableJoinList.Count > 0) { 
            //Debug.Log("Enabling List");
        }
        for (int i = 0; i < enableJoinList.Count; i++)
        {
            EnableJoint(anim, enableJoinList[i], true);
        }

        return true;
    }

    public void EnableRightArm(IKAnimator anim, bool Enable)
    {
        enableRightArm = Enable;
        BioIK.BioIK character = anim.GetBaxter().GetComponent<BioIK.BioIK>();
        ToggleChildJoints("Baxter_rightArm_joint1", character, Enable);
    }

    public void EnableLeftArm(IKAnimator anim, bool Enable)
    {
        enableLeftArm = Enable;
        BioIK.BioIK character = anim.GetBaxter().GetComponent<BioIK.BioIK>();
        ToggleChildJoints("Baxter_leftArm_joint1", character, Enable);
    }

    public void EnableHead(IKAnimator anim, bool Enable)
    {
        enableHead = Enable;
        BioIK.BioSegment segment = anim.GetBaxter().GetComponent<BioIK.BioIK>().FindSegment("Baxter_monitor");
        if (segment == null)
        {
            //Debug.Log("Joint name Baxter_monitor - Segment is null");
        }
        else
        {
            if (segment.Objectives != null && segment.Objectives.Length > 0)
            {
                segment.Objectives[0].enabled = Enable;
            }
            else
            {
                //Debug.Log("Joint name Baxter_monitor not found.");
            }
        }
    }

    public void EnableBase(IKAnimator anim, bool Enable)
    {
        enableBase = Enable;
        // EnableJoint(anim, "Baxter_base_visible", Enable);
        BioIK.BioSegment segment = anim.GetBaxter().GetComponent<BioIK.BioIK>().FindSegment("Baxter_base_visible");
        if (segment == null)
        {
            Debug.Log("Joint name Baxter_base_visible - Segment is null");
        }
        else
        {
            segment.enabled = Enable;
        }
    }
    /*
    public void EnableHip(IKAnimator anim, bool Enable)
    {
        //enableHip = Enable;
        //EnableJoint(anim, "Baxter_rotationJoint", Enable);

        enableHip = Enable;
        BioIK.BioSegment segment = anim.GetBaxter().GetComponent<BioIK.BioIK>().FindSegment("Baxter_upperBody");
        if (segment == null)
        {
            //Debug.Log("Joint name Baxter_upperBody - Segment is null");
        }
        else
        {
            if (segment.Objectives != null && segment.Objectives.Length > 0)
            {
                segment.Objectives[0].enabled = Enable;
            }
            else
            {
                //Debug.Log("Joint name Baxter_upperBody not found.");
            }
        }

        BioIK.BioSegment segment2 = anim.GetBaxter().GetComponent<BioIK.BioIK>().FindSegment("Baxter_rotationJoint");
        if (segment2.Joint != null)
        {
            segment2.Joint.enabled = Enable;
        }
        else
        {
            //Debug.Log("Joint name Baxter_rotationJoint not found.");
        }
    }
    */
    private void EnableJoint(IKAnimator anim, string joint_name, bool Enable)
    {
        // //Debug.Log("Enableing Joint " + joint_name);
        BioIK.BioIK character = anim.GetBaxter().GetComponent<BioIK.BioIK>();
        ToggleJoint(joint_name, character, Enable);
    }

    private void ToggleChildJoints(string joint_name, BioIK.BioIK character, bool isEnable)
    {
        ////Debug.Log("Joint Name:" + joint_name);
        BioIK.BioSegment segment = character.FindSegment(joint_name);

        if (segment.Joint != null)
        {
            /*
            if (isEnable) {
                //Debug.Log("Enabling Segment: " + segment.transform.name);
            } else {
                //Debug.Log("Disableing Segment: " + segment.transform.name);
            }
            */
            segment.Joint.enabled = isEnable;
        }

        for (int i = 0; i < segment.Childs.Length; i++)
        {
            ToggleChildJoints(segment.Childs[i].transform.name, character, isEnable);
        }
    }

    private void ToggleJoint(string joint_name, BioIK.BioIK character, bool isEnable)
    {
        BioIK.BioSegment segment = character.FindSegment(joint_name);

        if (segment != null)
        {
            if (segment.Joint != null)
            {
                segment.Joint.enabled = isEnable;
            }
            else
            {
                Debug.Log("Joint Segment doe not have any joints - " + joint_name);
            }
        }
        else
        {
            Debug.Log("Joint Segment not found - " + joint_name);
        }
    }
}
