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

public class CircleMoveCommand : IKCommand, Movable {
    private const float DISTANCE_TO_THRESHOLD = .05f;

    private Transform target;
    private int currentSplit;
    private int currentCircle;

    // default for now is y
    public char constantAxis;
    public float radius;
    public float startingAngle;
    public int numberOfSplits;
    public int numberOfCircles;

    [SerializeField]
    string circleCenterLocationName;
    [SerializeField]
    IKTarget moveHand;

    Transform circleCenter;

    private Vector3 startPos;

    public override void Initialize(IKAnimator anim) {
        currentSplit = 0;
        currentCircle = 1;

        circleCenter = GameObject.Find(circleCenterLocationName).transform;
        if (circleCenter == null) {
            //Debug.LogError("Move object not found");
        }

        float theta = calculateNewTheta();

        Vector3 newTargetLocation = calculateNewTargetLocation(theta, circleCenter.position.x, circleCenter.position.y, circleCenter.position.z);

        SetTargetLocation(anim, newTargetLocation, circleCenter.rotation);

        startPos = anim.GetBodyTransform(moveHand).position;
    }

    public override bool Execute(IKAnimator anim)
    {
        if (currentSplit < numberOfSplits) {
            if (Vector3.Distance(anim.GetBodyTransform(moveHand).position, anim.GetTarget(moveHand).position) < DISTANCE_TO_THRESHOLD || !anim.IsMoving(this)) {
                currentSplit++;

                float theta = calculateNewTheta();

                Vector3 newTargetLocation = calculateNewTargetLocation(theta, circleCenter.position.x, circleCenter.position.y, circleCenter.position.z);

                SetTargetLocation(anim, newTargetLocation, circleCenter.rotation);

                anim.ResetMovingData();
            }
        }

        if (currentSplit == numberOfSplits) {
            if (currentCircle == numberOfCircles)
            {
                return true;
            } else
            {
                currentSplit = 0;
                currentCircle += 1;
                return false;
            }
        } else {
            return false;
        }
    }

    private void SetTargetLocation(IKAnimator anim, Vector3 newTargetLocation, Quaternion newTargetRotation) {
        anim.GetTarget(moveHand).position = newTargetLocation;
        anim.GetTarget(moveHand).rotation = circleCenter.rotation;
    }

    private float calculateNewTheta() {
        float newTheta = startingAngle;

        float sizeOfSplit = 360f / numberOfSplits;

        newTheta = newTheta + currentSplit * sizeOfSplit;
        if (newTheta > 360f) {
            newTheta = newTheta - 360f;
        }

        float radian_conversion = newTheta * Mathf.PI / 180f; 

        return radian_conversion;
    }

    private Vector3 calculateNewTargetLocation(float theta, float orig_x, float orig_y, float orig_z) {
        // cicrcle at origin:
        // x = r * cos(t)  ;  y = r * sin(t)
        // circle not at origin:
        // x = h + r * cost(t)  ;  y = k + r * sin(t)

        Vector3 newTargetLocation = new Vector3(0f, 0f, 0f);

        if (constantAxis == 'x') {
            float y = radius * Mathf.Cos(theta);
            float z = radius * Mathf.Sin(theta);

            newTargetLocation = new Vector3(orig_x, orig_y + y, orig_z + z);
        }
        if (constantAxis == 'y') {
            float x = radius * Mathf.Cos(theta);
            float z = radius * Mathf.Sin(theta);

            newTargetLocation = new Vector3(orig_x + x, orig_y, orig_z + z);
        }
        if (constantAxis == 'z') {
            float x = radius * Mathf.Cos(theta);
            float y = radius * Mathf.Sin(theta);

            newTargetLocation = new Vector3(orig_x + x, orig_y + y, orig_z);
        }

        return newTargetLocation;
    }

    public Vector3 GetAnimationPosition(IKAnimator anim) {
        return anim.GetBodyTransform(moveHand).position;
    }

    public Vector3 GetBackupPosition() {
        return startPos;
    }

    public IKTarget GetBackupHand() {
        return moveHand;
    }
}
