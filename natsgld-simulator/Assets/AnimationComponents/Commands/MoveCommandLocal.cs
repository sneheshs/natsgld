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

public class MoveCommandLocal : IKCommand, Movable {
    [SerializeField]
    private MoveAssignment[] assignments;
    private Vector3[] startPositions;

    public override void Initialize(IKAnimator anim)
    {
        startPositions = new Vector3[assignments.Length];

        for (int i = 0; i < assignments.Length; i++)
        {
            MoveAssignment ma = assignments[i];

            // //Debug.Log(ma.ikTarget + " position = " + anim.GetTarget(ma.ikTarget).localPosition);
            // //Debug.Log(ma.ikTarget + " rotation = " + anim.GetTarget(ma.ikTarget).localRotation.eulerAngles.ToString());
            // //Debug.Log(ma.targetTransform + " position = " + ma.targetTransform.localPosition);
            // //Debug.Log(ma.targetTransform + " rotation = " + ma.targetTransform.localRotation.eulerAngles.ToString());

            anim.GetTarget(ma.ikTarget).localPosition = ma.targetTransform.localPosition;
            anim.GetTarget(ma.ikTarget).localRotation = ma.targetTransform.localRotation;

            anim.ToggleTarget(ma.ikTarget, true);

            startPositions[i] = anim.GetBodyTransform(ma.ikTarget).localPosition;


        }
    }

    public override bool Execute(IKAnimator anim)
    {
        float dist = 0F;

        ////Debug.Log("Waiting");
        if (!anim.IsMoving(this))
        {
            foreach (MoveAssignment ma in assignments)
            {
                dist = Vector3.Distance(anim.GetBodyTransform(ma.ikTarget).localPosition, ma.targetTransform.localPosition);
                //Debug.Log("DISTANCE = " + dist.ToString());

                if (dist > ma.threshold)
                {
                    throw new UnreachedTargetException("Move command failed to reach target.");
                }
            }

            /*foreach (MoveAssignment ma in assignments) {
                anim.ToggleTarget(ma.ikTarget, false);
            }*/

            return true;
        }

        for (int i = 0; i < assignments.Length; i++)
        {
            MoveAssignment ma = assignments[i];
            // //Debug.Log("> " + ma.targetTransform + " position = " + anim.GetTarget(ma.ikTarget).localPosition);
            // //Debug.Log("> " + ma.ikTarget + " rotation = " + anim.GetTarget(ma.ikTarget).localRotation.eulerAngles.ToString());
        }

        return false;
    }

    // Returns the average of all the moving targets
    public Vector3 GetAnimationPosition(IKAnimator anim) {
        Vector3 total = new Vector3();

        foreach (MoveAssignment ma in assignments) {
            total += anim.GetBodyTransform(ma.ikTarget).localPosition;
        }

        return total / assignments.Length;
    }

    [Serializable]
    public struct MoveAssignment
    {
        public Transform targetTransform;
        public IKTarget ikTarget;
        public float threshold;
    }
}
