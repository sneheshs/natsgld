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

public class PickupCommand : IKCommand, Movable
{
    [SerializeField]
    string pickupName;
    [SerializeField]
    IKTarget pickupHand;

    Transform pickup;

    private Vector3 startPos;

    public override void Initialize(IKAnimator anim)
    {
        anim.ToggleTarget(IKTarget.LeftShoulder, false);
        anim.ToggleTarget(IKTarget.RightShoulder, false);

        pickup = GameObject.Find(pickupName).transform;
        if (pickup == null)
        {
            //Debug.LogError("All pickup prefabs must have a pickup component.");
            return;
        }

        anim.GetTarget(pickupHand).position = pickup.position;
        anim.GetTarget(pickupHand).rotation = pickup.rotation;

        startPos = anim.GetBodyTransform(pickupHand).position;
    }

    public override bool Execute(IKAnimator anim)
    {
        float distance_to_threshold = pickup.GetComponent<Pickup>().distance_to_threshold;

        if (!anim.IsMoving(this))
        {
            if (Vector3.Distance(anim.GetBodyTransform(pickupHand).position, pickup.position) < distance_to_threshold)
            {
                pickup.SetParent(anim.GetBodyTransform(pickupHand));
                return true;
            }
            else
            {
                throw new UnreachedTargetException("Pickup command failed to reach target.");
            }
        }

        return false;
    }

    public Vector3 GetAnimationPosition(IKAnimator anim)
    {
        return anim.GetBodyTransform(pickupHand).position;
    }
}