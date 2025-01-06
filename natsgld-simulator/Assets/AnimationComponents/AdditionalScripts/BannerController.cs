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
using UnityEngine.Video;
using UnityEngine.UI;

public class BannerController : MonoBehaviour
{
    VideoPlayer videoPlayer;
    Dictionary<string, GameObject> shower;
    Dictionary<string, GameObject> hider;
    string[] objectsToHide;
    //Dictionary<string, string> videoDictionary;

    Text BannerVideoMessage;
    List<string> bannerVideoMessageList;

    string videoPath = "/Videos/New_Videos/";
    string videoExt = ".mp4";

    // Start is called before the first frame update
    void Start()
    {
        //Video players for 
        videoPlayer = GameObject.FindGameObjectWithTag("NG_VideoPlayer").GetComponent<VideoPlayer>();

        // Dictionary to store all shower and hider GameObject Handlers based on tags
        shower = new Dictionary<string, GameObject>();
        hider = new Dictionary<string, GameObject>();

        // Cycle through all tags and populate shower and hider handlers
        foreach (string s in UnityEditorInternal.InternalEditorUtility.tags)
        {
            ////Debug.Log("Tag: " + s);
            if (GameObject.FindGameObjectWithTag(s))
            {
                shower.Add(s, GameObject.FindGameObjectWithTag(s));
                hider.Add(s, shower[s]);
            }
            else
            {
                ////Debug.LogWarning("GameObject with tag " + s + " not found!");
            }
        }

        //// Initialize the video dictionary
        //videoDictionary = new Dictionary<string, string>();
        //buildVideoDictionary();

        BannerVideoMessage = GameObject.FindGameObjectWithTag("Banner_Video").GetComponent<Text>();
        bannerVideoMessageList = new List<string>();

        // Show left banner by default
        showhideBanner(true, false);

        // Hide all Precut things
        string[] hide_keywords = { "_Cut_", "_Default_Hidden_" }; // Keywords to use to hide stuff on load
        foreach (string goName in shower.Keys)
        {
            foreach (string hide_keyword in hide_keywords)
            {
                if (goName.Contains(hide_keyword))  
                {
                    gameobject_showhide(goName, false);
                }
            }
        }
    }

    /// <summary>
    /// Shows or hides the banners
    /// </summary>
    /// <param name="bShowBanner1">If set to <c>true</c> show banner1 else hide.</param>
    /// <param name="bShowBanner2">If set to <c>true</c> show banner2 else hide.</param>
    void showhideBanner(bool bShowBanner1, bool bShowBanner2)
    {
        gameobject_showhide("NG_Banner_1", bShowBanner1);
        gameobject_showhide("NG_Banner_2", bShowBanner2);
    }

    /// <summary>
    /// Shows or hides the specified GameObject.
    /// </summary>
    /// <param name="showhideGameObject">GameObject to Show/ Hide</param>
    /// <param name="bShow">If set to <c>true</c> show else hide.</param>
    void gameobject_showhide(string showhideGameObject, bool bShow)
    {
        if (bShow)
        {
            if (shower.ContainsKey(showhideGameObject))
            {
                shower[showhideGameObject].SetActive(true);
            }
            else
            {
                ////Debug.LogWarning ("GameObject Not found to Show: " + showhideGameObject);
            }
        }
        else
        {
            if (hider.ContainsKey(showhideGameObject))
            {
                hider[showhideGameObject].SetActive(false);
            }
            else
            {
                ////Debug.LogWarning ("GameObject Not found to Show: " + showhideGameObject);
            }
        }
    }

    /// <summary>
    /// Loads the vide into banners.
    /// </summary>
    /// <param name="actionKey">Action key describes the video name sent from ROS.</param>
    void loadVideIntoBanners(string actionKey) //string url)
    {
        //videoPlayer.url = url;
        //videoPlayer.url = videoDictionary[actionKey];

        videoPlayer.isLooping = true;
        videoPlayer.waitForFirstFrame = false;
        videoPlayer.skipOnDrop = true;
        videoPlayer.playOnAwake = true;
        videoPlayer.url = Application.dataPath + videoPath + actionKey + videoExt;
        videoPlayer.Play();
        //videoPlayer.isPlaying()
    }

    // Update is called once per frame
    public void Update()
    {
        string sBannerVideoMsg = "";

        // ROS Handler
        if (BannerVideoMessage.text != "")
        {
            sBannerVideoMsg = BannerVideoMessage.text.Trim();
            BannerVideoMessage.text = "";
            bannerVideoMessageList.Add(sBannerVideoMsg);
            //Debug.Log("Received Banner Video Message - " + sBannerVideoMsg);
        }

        // Process All ROS Banner Messages Received
        if (bannerVideoMessageList.Count > 0)
        {
            string[] cmd = bannerVideoMessageList[0].Trim().Split('|');
            bannerVideoMessageList.RemoveAt(0);

            // Command
            switch (cmd[0].ToLower().Trim())
            {
                case "showbannerwithvideo":
                    switch (cmd[1].ToLower().Trim())
                    {
                        case "left":
                            showhideBanner(true, false);
                            break;
                        case "right":
                            showhideBanner(false, true);
                            break;
                        case "both":
                            showhideBanner(true, true);
                            break;
                        case "none":
                            showhideBanner(false, false);
                            break;
                    }
                    loadVideIntoBanners(cmd[2].Trim());
                    break;
                    
                // Show Hide
                case "showbanner":
                    // cmd[1] left, right, both, none
                    switch (cmd[1].ToLower().Trim())
                    {
                        case "left":
                            showhideBanner(true, false);
                            break;
                        case "right":
                            showhideBanner(false, true);
                            break;
                        case "both":
                            showhideBanner(true, true);
                            break;
                        case "none":
                            showhideBanner(false, false);
                            break;
                    }
                    break;
                case "showandhide":
                    // cmd[1] show
                    // cmd[2] hide
                    gameobject_showhide(cmd[1].Trim(), true);
                    gameobject_showhide(cmd[2].Trim(), false);
                    break;
                case "show":
                    gameobject_showhide(cmd[1].Trim(), true);
                    break;
                case "hide":
                    gameobject_showhide(cmd[1].Trim(), false);
                    break;
                // Load Video
                case "loadvideo":
                    // cmd[1] video key or URL
                    loadVideIntoBanners(cmd[1].Trim());
                    break;
            }
        }

        // // This code loops the video -- NOT NEEDED, THE VIDEO AUTOMATICALLY LOOPS IT
        // if (videoPlayer.isPlaying)
        // {
        //     //Debug.Log("PLAYING -- " + videoPlayer.frame.ToString() + "/" + videoPlayer.frameCount.ToString());
        //     // if ((ulong)videoPlayer.frame >= (videoPlayer.frameCount - 5))
        //     // {
        //     //     videoPlayer.frame = 5;
        //     //     videoPlayer.Play();
        //     // }
        // }

        // Commented out - because this is not data collection
        // // WHICH BANNER
        // if (Input.GetKeyDown(KeyCode.LeftShift))
        // {
        //     showhideBanner(true, false);
        //     videoPlayer.Play();
        // }
        // else if (Input.GetKeyDown(KeyCode.RightShift))
        // {
        //     showhideBanner(false, true);
        //     videoPlayer.Play();
        // }
        // else if (Input.GetKeyDown(KeyCode.Space))
        // {
        //     showhideBanner(false, false);
        //     videoPlayer.Play();
        // }
        // else if (Input.GetKeyDown(KeyCode.Return))
        // {
        //     showhideBanner(true, true);
        //     videoPlayer.Play();
        // }
    }

}
