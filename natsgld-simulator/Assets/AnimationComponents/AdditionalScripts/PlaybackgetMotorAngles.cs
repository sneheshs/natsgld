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
using BioIK;

public class PlaybackgetMotorAngles : MonoBehaviour
{
    public string Filename = "Playback.csv";
    public string InputPath = "Recordings/MLImages/";

    public DataTable dt;
    public float deltaTime = 1.0f/30.0f;   // 30 fps
    private float currentDelta = 0.0f;
    private int currentRow = 0;
    private int prevRow = 0;
    
    public string[] headers;
    private int numColsToSkip = 2;

    public List<GameObject> bodyparts = new List<GameObject>();
    public List<BioJoint.Motion> bioJointMotions = new List<BioJoint.Motion>();
    
    // Parse CSV file into DataTable
    public DataTable ParseCSVtoDataTable(string strFilePath)
    {
        GameObject go;

        DataTable dt = new DataTable();
        using (StreamReader sr = new StreamReader(strFilePath))
        {
            int count = 0;
            headers = sr.ReadLine().Trim().Split(',');
            foreach (string header in headers)
            {
                dt.Columns.Add(header);

                if (count >= numColsToSkip)
                {
                    go = GameObject.Find(header);
                    bodyparts.Add(go);

                    if (go.GetComponent<BioJoint>().X.Enabled)
                    {
                        bioJointMotions.Add(go.GetComponent<BioJoint>().X);
                    }
                    else if (go.GetComponent<BioJoint>().Y.Enabled)
                    {
                        bioJointMotions.Add(go.GetComponent<BioJoint>().Y);
                    }
                    else if (go.GetComponent<BioJoint>().Z.Enabled)
                    {
                        bioJointMotions.Add(go.GetComponent<BioJoint>().Z);
                    }
                }
                count++;
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
    }

    void Update()
    {
        if(currentRow < dt.Rows.Count-1)
        {
            if (currentDelta <= 0.0f)
            {
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
                Debug.Log("Row: " + currentRow);
                for(int i =0; i < bodyparts.Count; i++)
                {
                    bioJointMotions[i].TargetValue = float.Parse(dt.Rows[currentRow][i+numColsToSkip].ToString());
                }
            }
        }
    }
}