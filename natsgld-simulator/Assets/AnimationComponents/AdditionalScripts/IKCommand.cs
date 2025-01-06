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

public abstract class IKCommand : MonoBehaviour
{
    public enum IKTarget
    {
        LeftHand = 0,
        RightHand = 1,
        LeftShoulder = 2,
        RightShoulder = 3,
        Head = 4,
        Base = 5
        //Hip = 6
    };

    public int FRAME_SAMPLE_SIZE = 15;

    public virtual void Initialize(IKAnimator anim) { }
    /**
     * Runs every frame. Returns true if command is complete 
     */
    public abstract bool Execute(IKAnimator anim);
}
