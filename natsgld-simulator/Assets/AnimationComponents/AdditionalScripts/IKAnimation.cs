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
using UnityEditor;
using System;
using System.IO;

public class IKAnimation : ScriptableObject
{
    [SerializeField]
    public GameObject commandObj; // Gameobject prefab with commands stored

    public int index;

    public string status;
    public string completedAnimation;

    private GameObject bax;
    private BioIK.BioIK bik;

    public bool bFirstTime = true;

    public void Initialize(IKAnimator anim)
    {
        index = 0;
        // Initialize first command
        GetCurrentCommand().Initialize(anim);
        anim.CommandStarting();
        status = "";
        completedAnimation = "";

        //bax = GameObject.Find("/Baxter/Baxter_base");
        bax = GameObject.FindGameObjectWithTag("Baxter_base");

        if (bax)
        {
            bik = bax.GetComponents<BioIK.BioIK>()[0];
        }
        else
        {
            //Debug.Log("ERROR: Baxter or Baxter_base not found!");
        }

        bFirstTime = true;
    }

    public void Animate(IKAnimator anim)
    {
        String __s = "";

        IKCommand[] commands = commandObj.GetComponents<IKCommand>();

        if (index >= commands.Length) { anim.updateLoading(false); return; }

        anim.updateLoading(true);

        try
        {
            //if saved file, set solution flag true
            //else, set solution flag false

            string filename = "Solutions/" + index.ToString() + "_" + commands[index].ToString();

            if (File.Exists(filename) && bFirstTime)
            {
                bFirstTime = false;
                //Debug.Log("Saved Solution found -- " + filename);
                string sdouble = File.ReadAllText(filename).Trim();
                string[] sSol = sdouble.Split('|');

                bik.OptimalSolution = new double[sSol.Length];
                for (int i = 0; i < sSol.Length; i++)
                {
                    bik.OptimalSolution[i] = Convert.ToDouble(sSol[i]);
                }
                //Load solution from file
                bik.OptimalSolutionExists = true;
            }
            else
            {
                ////Debug.Log("No Solution found -- " + filename);
                bik.OptimalSolutionExists = false;
            }

            ////Debug.Log("IKAniation: Execting " + commands[index].name);
            anim.FRAME_SAMPLE_SIZE = commands[index].FRAME_SAMPLE_SIZE;
            if (commands[index].Execute(anim))
            {
                //Debug.Log("IKAniation: FINISHED Executing " + commands[index].ToString());

                __s = "";
                if (bik.Solution.Length > 0)
                {
                    foreach (Double d in bik.Solution)
                    {
                        ////Debug.Log("BioIK Solution found ***");

                        __s += d.ToString() + "|";
                    }

                    if (__s.Length > 0)
                    {
                        //Debug.Log("BioIK Solution = " + __s);
                        __s = __s.Substring(0, __s.Length - 1);
                    }
                    //TODO: Maybe the below if confidion sould be include in within this if condition

                    if (!bik.OptimalSolutionExists && bFirstTime)
                    {
                        //Debug.Log("NEW BioIK Solution Saved!!!!!!!!!!!!!!");
                        File.WriteAllText(filename, __s);
                    }
                }
                else
                {
                    //Debug.Log("IKAnimaton: There was no BioIK Solution for this step.");
                }
                
                anim.iBackupCounter = 0;
                status = "" + index + ", STEP DONE " + index + " - " + commands[index].ToString();

                completedAnimation = commands[index].ToString() +" "+ index.ToString();
                ////Debug.Log(status);
                index++;

                bFirstTime = true;

                // Initialize right before command is first run
                if (index < commands.Length)
                {
                    GetCurrentCommand().Initialize(anim);
                    anim.CommandStarting();
                }
            }

            if (index == commands.Length)
            {
                status = "" + commands.Length + ", COMPLETED - " + commands[commands.Length - 1].ToString();
                completedAnimation = commands[commands.Length - 1].ToString();
            }
        }
        catch (UnreachedTargetException e)
        {
            //Debug.LogWarning("Caught Error: " + e.Message);

            // Must be movable to back up
            if (GetCurrentCommand() as Movable != null) {
                anim.Backup();
                Debug.LogWarning("Backing up - " + commands[index].ToString());
            } else {
                //Debug.LogError("UnreachedTargetException thrown by non-Movable");
            }
        }

        anim.updateLoading(false);
    }

    public IKCommand GetCurrentCommand()
    {
        IKCommand[] commands = commandObj.GetComponents<IKCommand>();
        
        return (index >= commands.Length) ? null : commands[index];
    }

    public Transform getBaseTransform()
    {
        return commandObj.transform;
    }
    
    [MenuItem("Assets/Create/IKAnimation")]
    public static void CreateAnimation()
    {
        IKAnimation asset = CreateInstance<IKAnimation>();

        AssetDatabase.CreateAsset(asset, "Assets/NewIKAnimation.asset");
        AssetDatabase.SaveAssets();

        EditorUtility.FocusProjectWindow();

        Selection.activeObject = asset;
    }
}
