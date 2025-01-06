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
//sing System.Collections;
//using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class Main : MonoBehaviour
{
    //public string[] scenes;
    //public string[] keyMap;

    public Animator animator;

    private UnityEngine.UI.Text AnimationName;
    private string nextScene;

    void Start()
    {
        AnimationName = GameObject.FindGameObjectWithTag("Animation_Name").GetComponent<UnityEngine.UI.Text>();
    }

    void Update()
    {
        if (AnimationName == null)
        {
            //Debug.Log("NG ERROR: AnimationName object is null. Check MainCamera Object if NG ROS Messages obects are linked.");
        }
        else
        {
            if (AnimationName.text != "")
            {
                nextScene = AnimationName.text;
                FadeToNextScene();
                AnimationName.text = "";
            }
        }

        //Legacy code -- Delete it next time you see it still not being used
        //for (int i = 0; i < scenes.Length; i++)
        //{
        //    if (Input.GetKeyDown(keyMap[i]))
        //    {
        //        //Debug.Log("Scene2 loading: " + scenes[i]);
        //        var parameters = new LoadSceneParameters(LoadSceneMode.Single);

        //        SceneManager.LoadScene(scenes[i], parameters);
        //    }
        //}

        if (Input.GetKeyDown(KeyCode.Space))
        {
            //Debug.Log("Received SPACE");

            nextScene = "CuttingTomatoAnimation";
            FadeToNextScene();
        }
    }

    public void FadeToNextScene()
    {
        //Debug.Log("FadeOut Trigger Set.");
        animator.SetTrigger("trigger1");
    }

    public void OnFadeComplete()
    {
        //Debug.Log("FadeOut Completed. Loading Next Scene.");

        //var parameters = new LoadSceneParameters(LoadSceneMode.Single);
        //SceneManager.LoadScene(AnimationName.text, parameters);
        SceneManager.LoadScene(nextScene);
    }
}
