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
using UnityEngine;
using System;

public class SaveMLImages : MonoBehaviour
{
        public string Filename = "NatComm";
        public string OutputPath = "Recordings/MLImages/";
        private ImageSynthesis IS;
        private int imageCounter = 0;

        public bool SaveRGB = true;
        public bool SaveDepth = false;
        public bool SaveObjectSegmentation = false;
        public bool SaveCategorySegmentation = false;

        private DateTime DT;


        void Start()
        {
                CreateDirectoryIfDoesntExist(OutputPath.Substring(0, OutputPath.Length - 1));

                IS = GetComponent<ImageSynthesis>();
                string checkpath = "";
                if (SaveRGB)
                {
                        checkpath = OutputPath + "_img";
                        CreateDirectoryIfDoesntExist(checkpath);
                }
                if (SaveDepth)
                {
                        checkpath = OutputPath + "_depth";
                        CreateDirectoryIfDoesntExist(checkpath);
                }
                if (SaveObjectSegmentation)
                {
                        checkpath = OutputPath + "_id";
                        CreateDirectoryIfDoesntExist(checkpath);
                }
                if (SaveCategorySegmentation)
                {
                        checkpath = OutputPath + "_layer";
                        CreateDirectoryIfDoesntExist(checkpath);
                }
        }

        void CreateDirectoryIfDoesntExist(string checkpath)
        {
                if (!System.IO.Directory.Exists(checkpath))
                {
                        System.IO.Directory.CreateDirectory(checkpath);
                }
        }

        void Update()
        {
                DT = DateTime.Now;
                // IS.Save(Filename + "_" + imageCounter.ToString("D5") + "_" + DT.Ticks.ToString(), 640, 480, OutputPath, SaveRGB, SaveDepth, SaveObjectSegmentation, SaveCategorySegmentation);
                IS.Save("" + imageCounter.ToString("D5") + "_" + DT.Ticks.ToString(), 640, 480, OutputPath, SaveRGB, SaveDepth, SaveObjectSegmentation, SaveCategorySegmentation);
                imageCounter++;
        }
}
