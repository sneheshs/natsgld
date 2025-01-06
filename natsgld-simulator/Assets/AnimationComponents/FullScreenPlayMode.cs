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
*/using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[InitializeOnLoad]
public class FullscreenPlayMode : MonoBehaviour
{


    //The size of the toolbar above the game view, excluding the OS border.
    private static int tabHeight = 40;

    static FullscreenPlayMode()
    {
        EditorApplication.playmodeStateChanged -= CheckPlayModeState;
        EditorApplication.playmodeStateChanged += CheckPlayModeState;

    }

    //// Start is called before the first frame update
    //void Start()
    //{

    //}

    //// Update is called once per frame
    //void Update()
    //{

    //}

    static void CheckPlayModeState()
    {
      /*  if (EditorApplication.isPlaying)
        {
           // FullScreenGameWindow();
        }
        else
        {
            CloseGameWindow();
        }*/
    }

    static EditorWindow GetMainGameView()
    {
        //EditorApplication.ExecuteMenuItem("Window/Game");

        System.Type T = System.Type.GetType("UnityEditor.GameView,UnityEditor");
        System.Reflection.MethodInfo GetMainGameView = T.GetMethod("GetMainGameView", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Static);
        System.Object Res = GetMainGameView.Invoke(null, null);
        return (EditorWindow)Res;
    }

    static void FullScreenGameWindow()
    {

        EditorWindow gameView = GetMainGameView();
        Rect newPos = new Rect(0, 0 - tabHeight, 1920, 1080 + 18);

        gameView.titleContent.text = "NG Kitchen";
        gameView.position = newPos;
        gameView.minSize = new Vector2(1920, 1080 + 18);
        gameView.maxSize = gameView.minSize;
        gameView.position = newPos;
    }

    static void CloseGameWindow()
    {
        EditorWindow gameView = GetMainGameView();
        gameView.Close();
    }
}
