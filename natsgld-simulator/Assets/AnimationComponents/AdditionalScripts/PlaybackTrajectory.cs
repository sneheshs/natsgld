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
using System.Data;
using System.IO;
using System.Collections.Generic;
using System.Text.RegularExpressions;

public class PlaybackTrajectory : MonoBehaviour
{
        public string Filename = "Playback.csv";
        public string InputPath = "Recordings/MLImages/";

        public DataTable dt;
        public float deltaTime = 1.0f/30.0f;   // 30 fps
        private float currentDelta = 0.0f;
        private int currentRow = 0;
        private int prevRow = 0;
        private List<GameObject> targets = new List<GameObject>();
        private List<GameObject> bodyparts = new List<GameObject>();
        
        // Parse CSV file into DataTable
        public static DataTable ParseCSVtoDataTable(string strFilePath)
        {
                DataTable dt = new DataTable();
                using (StreamReader sr = new StreamReader(strFilePath))
                {
                        string[] headers = sr.ReadLine().Trim().Split(',');
                        foreach (string header in headers)
                        {
                                dt.Columns.Add(header);
                        }

                        while (!sr.EndOfStream)
                        {
                                string sline = sr.ReadLine().Trim();

                                // regular expression to parse CSV line with double quotes field with comma
                                // sline.Split(',') <-- can't handle fields with double quotes and comma
                                string[] cols = Regex.Split(sline, ",(?=(?:[^\"]*\"[^\"]*\")*(?![^\"]*\"))");

                                DataRow dr = dt.NewRow();
                                for (int i = 0; i < headers.Length; i++)
                                {
                                        dr[i] = cols[i];
                                }
                                dt.Rows.Add(dr);
                        }
                }
                return dt;
        }

        void Start()
        {
                if (!System.IO.Directory.Exists(InputPath))
                {
                        Debug.Log("Path Not Found: " + InputPath);
                }

                string checkpath = InputPath + Filename;
                if (!System.IO.File.Exists(checkpath))
                {
                        Debug.Log("File Not Found: " + checkpath);
                }
        
                dt = ParseCSVtoDataTable(checkpath);
                // Debug.Log("Columns: " + dt.Columns.ToString());
                // Debug.Log("Rows: " + dt.Rows.Count.ToString());

                targets.Add(GameObject.FindGameObjectWithTag("LHTarget"));
                targets.Add(GameObject.FindGameObjectWithTag("RHTarget"));
                targets.Add(GameObject.FindGameObjectWithTag("LShoulderTarget"));
                targets.Add(GameObject.FindGameObjectWithTag("RShoulderTarget"));
                targets.Add(GameObject.FindGameObjectWithTag("LookTarget"));
                targets.Add(GameObject.FindGameObjectWithTag("BaseTarget"));

                bodyparts.Add(GameObject.FindGameObjectWithTag("LGripPos"));
                bodyparts.Add(GameObject.FindGameObjectWithTag("RGripPos"));
                bodyparts.Add(GameObject.FindGameObjectWithTag("Baxter_leftArm_joint4"));
                bodyparts.Add(GameObject.FindGameObjectWithTag("Baxter_rightArm_joint4"));
                bodyparts.Add(GameObject.FindGameObjectWithTag("Baxter_monitor"));
                bodyparts.Add(GameObject.FindGameObjectWithTag("BaxterOG"));
        }


        void setTargetTransform(int targetIndex, int row, int column, bool showDebug = false)
        {
                // string sPrint = "";
                // for (int i = 0; i < dt.Columns.Count; i++) sPrint += dt.Rows[row][i].ToString() + ", ";
                // Debug.Log(row.ToString() + ")" + sPrint);

                targets[targetIndex].transform.position = new Vector3(
                        float.Parse(dt.Rows[row][column + 0].ToString()),
                        float.Parse(dt.Rows[row][column + 1].ToString()),
                        float.Parse(dt.Rows[row][column + 2].ToString())
                );
                targets[targetIndex].transform.rotation = new Quaternion(
                        float.Parse(dt.Rows[row][column + 3].ToString()),
                        float.Parse(dt.Rows[row][column + 4].ToString()),
                        float.Parse(dt.Rows[row][column + 5].ToString()),
                        float.Parse(dt.Rows[row][column + 6].ToString())
                );
                if (showDebug)
                {
                        Debug.Log(row.ToString() + ")" + targets[targetIndex].name + " - " + targets[targetIndex].transform.position.ToString()  + " | " + targets[targetIndex].transform.rotation.ToString());
                }
        }


        void setDirectTransform(int bodypartIndex, int row, int column)
        {
                // string sPrint = "";
                // for (int i = 0; i < dt.Columns.Count; i++) sPrint += dt.Rows[row][i].ToString() + ", ";
                // Debug.Log(row.ToString() + ")" + sPrint);

                bodyparts[bodypartIndex].transform.position = new Vector3(
                        float.Parse(dt.Rows[row][column + 0].ToString()),
                        float.Parse(dt.Rows[row][column + 1].ToString()),
                        float.Parse(dt.Rows[row][column + 2].ToString())
                );
                bodyparts[bodypartIndex].transform.rotation = new Quaternion(
                        float.Parse(dt.Rows[row][column + 3].ToString()),
                        float.Parse(dt.Rows[row][column + 4].ToString()),
                        float.Parse(dt.Rows[row][column + 5].ToString()),
                        float.Parse(dt.Rows[row][column + 6].ToString())
                );
                // Debug.Log(row.ToString() + ")" + bodyparts[bodypartIndex].name + " - " + bodyparts[bodypartIndex].transform.position.ToString() + " | " + bodyparts[bodypartIndex].transform.rotation.ToString());
        }

        // *** BIG TODO: While movements work, pick up, drop, cut, and pour needs extra work to make it work
        void Update()
        {
                if(currentRow < dt.Rows.Count-1)
                {
                        if (currentDelta <= 0.0f)
                        {
                                // Set the target positions
                                // Set the target orientations
                                currentDelta = deltaTime;
                                prevRow = currentRow;
                                currentRow++;
                        }
                        else
                        {
                                currentDelta -= Time.deltaTime;
                        }

                        if (currentRow != prevRow)
                        {
                                // Do we need to interpolate between the previous and current rows?
                                // LHTarget 0, RHTarget 1, LShoulderTarget 2, RShoulderTarget 3, LookTarget 4, BaseTarget 5
                                // Column Base 8, LH 18, RH 28, Head 38
                                setTargetTransform(0, currentRow, 18);  // LH
                                setTargetTransform(1, currentRow, 28);  // RH
                                // 2 LShoulderTarget
                                // 3 RShoulderTarget
                                setTargetTransform(4, currentRow, 38); // Head
                                setDirectTransform(5, currentRow, 8);  // Base
                        }
                }
        }
}