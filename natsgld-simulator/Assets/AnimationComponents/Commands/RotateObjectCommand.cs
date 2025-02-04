﻿/*
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

public class RotateObjectCommand : IKCommand
{
    public char axis;
    public float rotation;
    public bool isClockwise;
    public IKTarget rotateHand;

    private Vector3 rotateDir;
    private int rotationCount;

    public override void Initialize(IKAnimator anim)
    {
        if (axis == 'x') {
            rotateDir = Vector3.right;
        }
        else if (axis == 'y') {
            rotateDir = Vector3.up;
        }
        else if (axis == 'z') {
            rotateDir = Vector3.forward;
        }
        else {
            //Debug.LogError("Axis must be x, y, or z.");
            rotateDir = Vector3.zero;
        }

        rotateDir = isClockwise ? rotateDir * -1 : rotateDir;
        rotationCount = 0;
    }

    public override bool Execute(IKAnimator anim) {
        Transform rotateObject = anim.GetBodyTransform(rotateHand).GetChild(0);

        if (rotateObject == null) {
            //Debug.LogError("No object in selected hand to rotate.");
            return false;
        }

        // Rotates 1 degree per second
        rotateObject.Rotate(rotateDir, Space.World);
        rotationCount++;

        return (rotationCount > rotation);
    }
}
