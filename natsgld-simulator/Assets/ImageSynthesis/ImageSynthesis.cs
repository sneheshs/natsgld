using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;
using System.Collections.Generic;
using BioIK;

// @TODO:
// . support custom color wheels in optical flow via lookup textures
// . support custom depth encoding
// . support multiple overlay cameras
// . tests
// . better example scene(s)

// @KNOWN ISSUES
// . Motion Vectors can produce incorrect results in Unity 5.5.f3 when
//      1) during the first rendering frame
//      2) rendering several cameras with different aspect ratios - vectors do stretch to the sides of the screen

[RequireComponent (typeof(Camera))]
public class ImageSynthesis : MonoBehaviour {

	// pass configuration
	private CapturePass[] capturePasses = new CapturePass[] {
		new CapturePass() { name = "_img" },
		new CapturePass() { name = "_id", supportsAntialiasing = false },
		new CapturePass() { name = "_layer", supportsAntialiasing = false },
		new CapturePass() { name = "_depth" }
		// new CapturePass() { name = "_normals" },
		// new CapturePass() { name = "_flow", supportsAntialiasing = false, needsRescale = true } // (see issue with Motion Vectors in @KNOWN ISSUES)
	};

	private Dictionary<int, Rect> objectBBox = new Dictionary<int, Rect>();
	private Dictionary<int, string> objectName = new Dictionary<int, string>();
	private Dictionary<int, Color> objectColors = new Dictionary<int, Color>();

	List<string> objectManipulatedKeys;
	public Dictionary<string, bool> objectManipulated = new Dictionary<string, bool>();
	public Dictionary<string, Transform> objectManiStartTransform = new Dictionary<string, Transform>();


	struct CapturePass {
		// configuration
		public string name;
		public bool supportsAntialiasing;
		public bool needsRescale;
		public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }

		// impl
		public Camera camera;
	};
	
	public Shader uberReplacementShader;
	public Shader opticalFlowShader;

	public float opticalFlowSensitivity = 1.0f;

	// cached materials
	private Material opticalFlowMaterial;

	//Robot Handle To Save States
	GameObject baxOG, baxLH, baxRH, baxLArm, baxRArm, baxHead, baxBase;
	StreamWriter swBaxPosOri, swObjectBBox;
	string sLine = "";
	string sBBox = "";
	IKAnimator anim;
	BioIK.BioIK charac;
	BioSegment headSegment, laSegment, raSegment, baseSegment;

	public string path = "Recordings/MLImages/";
	public bool saveStates = true;

	GameObject baxter_base;


	// public double LowerLimit;
	// public double UpperLimit;
	public BioSegment[] bioIKSegmentsWithJoints = new BioSegment[0];
	public double[] bioJointsTargetValues = new double[0];
	public double[] prev_bioJointsTargetValues = new double[0];

	// -1 = None, 0 = X, 1 = Y, 2 = Z // Assuming there is only 1 target value per joint
	public int[] bioJointTargetXYZ = new int[0];

	private TMPro.TextMeshProUGUI OZ_Banner;


	private void AddChild(BioSegment child)
	{
		System.Array.Resize(ref bioIKSegmentsWithJoints, bioIKSegmentsWithJoints.Length+1);
		bioIKSegmentsWithJoints[bioIKSegmentsWithJoints.Length-1] = child;

		System.Array.Resize(ref bioJointsTargetValues, bioJointsTargetValues.Length+1);
		System.Array.Resize(ref bioJointTargetXYZ, bioJointTargetXYZ.Length+1);
		bioJointsTargetValues[bioJointsTargetValues.Length-1] = 0.0;
		bioJointTargetXYZ[bioJointTargetXYZ.Length-1] = -1;
			
		if (child.Joint.X.Enabled)
		{
			bioJointsTargetValues[bioJointsTargetValues.Length-1] = child.Joint.X.TargetValue;
			bioJointTargetXYZ[bioJointTargetXYZ.Length-1] = 0;
		}
		else if(child.Joint.Y.Enabled)
		{
			bioJointsTargetValues[bioJointsTargetValues.Length-1] = child.Joint.Y.TargetValue;
			bioJointTargetXYZ[bioJointTargetXYZ.Length-1] = 1;
		}
		else if(child.Joint.Z.Enabled)
		{
			bioJointsTargetValues[bioJointsTargetValues.Length-1] = child.Joint.Z.TargetValue;
			bioJointTargetXYZ[bioJointTargetXYZ.Length-1] = 2;
		}
	}

	private string sHeader = "";

	void populateBioIKSegmentsWithJoints(BioSegment bikSeg)
	{
		if (bikSeg != null)
		{
			if (bikSeg.Joint != null)
			{
				Debug.Log("Adding BioIK Segment: " + bikSeg.name + " with joint: " + bikSeg.Joint.name);
				AddChild(bikSeg);
				sHeader += bikSeg.name + ",";
			}

			if(bikSeg.Childs.Length > 0)
			{
				foreach (BioSegment child in bikSeg.Childs)
				{
					populateBioIKSegmentsWithJoints(child);
				}
			}
		}
	}

	void populatedObjects()
	{
		var renderers = Object.FindObjectsOfType<Renderer>();
		foreach (var r in renderers)
		{
			if (saveStates)
			{
				SaveObjectTag SOT = r.gameObject.GetComponent<SaveObjectTag>();
				if (SOT && SOT.SaveMe)
				{
					Debug.Log("Adding Object: " + r.gameObject.name);

					objectManipulated[r.gameObject.name] = false;
					objectManiStartTransform[r.gameObject.name] = new GameObject().transform;

					sHeader += r.gameObject.name + ",";
				}
			}
		}

		objectManipulatedKeys = new List<string>(objectManipulated.Keys);
	}


	void Start()
	{
		if (!System.IO.Directory.Exists(path.Substring(0, path.Length - 1)))
		{
			System.IO.Directory.CreateDirectory(path.Substring(0, path.Length - 1));
		}

		// default fallbacks, if shaders are unspecified
		if (!uberReplacementShader)
			uberReplacementShader = Shader.Find("Hidden/UberReplacement");

		if (!opticalFlowShader)
			opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

		// use real camera to capture final image
		capturePasses[0].camera = GetComponent<Camera>();
		for (int q = 1; q < capturePasses.Length; q++)
			capturePasses[q].camera = CreateHiddenCamera (capturePasses[q].name);

		
		
		if (saveStates)
		{
			// Get robot handle
			baxOG = GameObject.FindGameObjectWithTag("BaxterOG");
			baxLH = GameObject.FindGameObjectWithTag("LGripPos");
			baxRH = GameObject.FindGameObjectWithTag("RGripPos");
			baxLArm = GameObject.FindGameObjectWithTag("Baxter_leftArm_joint1");
			baxRArm = GameObject.FindGameObjectWithTag("Baxter_rightArm_joint1");
			baxHead = GameObject.FindGameObjectWithTag("Baxter_monitor");
			baxBase = GameObject.FindGameObjectWithTag("Baxter_base_visible");

			anim = GameObject.FindGameObjectWithTag("AnimationContainer").GetComponent<IKAnimator>();


			try
			{
				OZ_Banner = GameObject.FindGameObjectWithTag("OZ_Banner").GetComponent<TMPro.TextMeshProUGUI>();
			}
			catch(System.Exception e)
			{
				OZ_Banner = null;
			}

		
			charac = anim.GetBaxter().GetComponent<BioIK.BioIK>();
			headSegment = charac.FindSegment("Baxter_monitor");

			laSegment = charac.FindSegment("Baxter_leftArm_joint1");
			raSegment = charac.FindSegment("Baxter_rightArm_joint1");
			baseSegment = charac.FindSegment("Baxter_base_visible");

			swBaxPosOri = new StreamWriter(path + "BaxterStates.csv");
			swObjectBBox = new StreamWriter(path + "ObjectIDColors.csv");
		
			// swBaxPosOri.WriteLine("SN,Command,BaseTX,BaseTY,BaseTZ,BaseRX,BaseRY,BaseRZ,BaseRW,LeftHandTX,LeftHandTY,LeftHandTZ,LeftHandRX,LeftHandRY,LeftHandRZ,LeftHandRW,RightHandTX,RightHandTY,RightHandTZ,RightHandRX,RightHandRY,RightHandRZ,RightHandRW,HeadTX,HeadTY,HeadTZ,HeadRX,HeadRY,HeadRZ,HeadRW");
			// string sHeader = "SN,Command,BaseTX,BaseTY,BaseTZ,BaseRZ,BaseRX,BaseRY,LeftHandTX,LeftHandTY,LeftHandTZ,LeftHandRZ,LeftHandRX,LeftHandRY,RightHandTX,RightHandTY,RightHandTZ,RightHandRZ,RightHandRX,RightHandRY,HeadTX,HeadTY,HeadTZ,HeadRZ,HeadRX,HeadRY";
			sHeader = "SN,Command,";

			// NOW WE USE JOINT TARGET VALUE INSTEAD
			baxter_base = GameObject.FindGameObjectWithTag("Baxter_base");
			// prev_bioJointsTargetValues = new double[bioJointsTargetValues.Length];
			// for (int i = 0; i < bioJointsTargetValues.Length; i++)
			// {
			// 	prev_bioJointsTargetValues[i] = 0.0 + bioJointsTargetValues[i];
			// }

			// NOT USING THIS ANYMORE
			string[] sJointsEnabled = new string[] {"Base", "LeftArm", "RightArm", "Head", "DisabledList", "EnabledList"};
			for (int i =0; i < sJointsEnabled.Length; i++)
			{
				sHeader += sJointsEnabled[i] + ",";
			}

			string[] sJoints = new string[] {"Base", "LeftHand", "RightHand", "Head"};
			string[] sPos = new string[] {"TX", "TY", "TZ"};
			string[] sOri = new string[] {"RX", "RY", "RZ", "RW", "_r", "_p", "_y"};
			for (int i = 0; i < sJoints.Length; i++)
			{
				for (int j = 0; j < sPos.Length; j++)
				{
					sHeader += sJoints[i] + sPos[j] + ",";
				}
				for (int j = 0; j < sOri.Length; j++)
				{
					sHeader += sJoints[i] + sOri[j] + ",";
				}
			}

			populateBioIKSegmentsWithJoints(baxter_base.GetComponent<BioSegment>());
			populatedObjects();

			// sHeader = sHeader.Substring(0, sHeader.Length - 1); // Remove last comma
			sHeader += "DBSN";

			swBaxPosOri.WriteLine(sHeader);
			swBaxPosOri.AutoFlush = true;
			swObjectBBox.AutoFlush = true;
		}

		OnCameraChange();
		OnSceneChange();
	}

	void LateUpdate()
	{
		#if UNITY_EDITOR
		if (DetectPotentialSceneChangeInEditor())
			OnSceneChange();
		#endif // UNITY_EDITOR

		// @TODO: detect if camera properties actually changed
		OnCameraChange();
	}
	
	private Camera CreateHiddenCamera(string name)
	{
		var go = new GameObject (name, typeof (Camera));
		go.hideFlags = HideFlags.HideAndDontSave;
		go.transform.parent = transform;

		var newCamera = go.GetComponent<Camera>();
		return newCamera;
	}


	static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode)
	{
		SetupCameraWithReplacementShader(cam, shader, mode, Color.black);
	}

	static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacelementModes mode, Color clearColor)
	{
		var cb = new CommandBuffer();
		cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
		cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
		cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
		cam.SetReplacementShader(shader, "");
		cam.backgroundColor = clearColor;
		cam.clearFlags = CameraClearFlags.SolidColor;
	}

	static private void SetupCameraWithPostShader(Camera cam, Material material, DepthTextureMode depthTextureMode = DepthTextureMode.None)
	{
		var cb = new CommandBuffer();
		cb.Blit(null, BuiltinRenderTextureType.CurrentActive, material);
		cam.AddCommandBuffer(CameraEvent.AfterEverything, cb);
		cam.depthTextureMode = depthTextureMode;
	}

	enum ReplacelementModes {
		ObjectId 			= 0,
		CatergoryId			= 1,
		DepthCompressed		= 2,
		DepthMultichannel	= 3,
		Normals				= 4
	};

	public void OnCameraChange()
	{
		int targetDisplay = 1;
		var mainCamera = GetComponent<Camera>();
		foreach (var pass in capturePasses)
		{
			if (pass.camera == mainCamera)
				continue;

			// cleanup capturing camera
			pass.camera.RemoveAllCommandBuffers();

			// copy all "main" camera parameters into capturing camera
			pass.camera.CopyFrom(mainCamera);

			// set targetDisplay here since it gets overriden by CopyFrom()
			pass.camera.targetDisplay = targetDisplay++;
		}

		// cache materials and setup material properties
		if (!opticalFlowMaterial || opticalFlowMaterial.shader != opticalFlowShader)
			opticalFlowMaterial = new Material(opticalFlowShader);
		opticalFlowMaterial.SetFloat("_Sensitivity", opticalFlowSensitivity);

		// setup command buffers and replacement shaders
		SetupCameraWithReplacementShader(capturePasses[1].camera, uberReplacementShader, ReplacelementModes.ObjectId);
		SetupCameraWithReplacementShader(capturePasses[2].camera, uberReplacementShader, ReplacelementModes.CatergoryId);
		SetupCameraWithReplacementShader(capturePasses[3].camera, uberReplacementShader, ReplacelementModes.DepthMultichannel, Color.white);
		// SetupCameraWithReplacementShader(capturePasses[4].camera, uberReplacementShader, ReplacelementModes.Normals);
		// SetupCameraWithPostShader(capturePasses[5].camera, opticalFlowMaterial, DepthTextureMode.Depth | DepthTextureMode.MotionVectors);
	}

	public Rect GUI3dRectWithObject(GameObject go)
	{
		Vector3 cen = go.GetComponent<Renderer>().bounds.center;
		Vector3 ext = go.GetComponent<Renderer>().bounds.extents;
		Vector2[] extentPoints = new Vector2[8]
		{
				WorldToGUIPoint(new Vector3(cen.x-ext.x, cen.y-ext.y, cen.z-ext.z)),
				WorldToGUIPoint(new Vector3(cen.x+ext.x, cen.y-ext.y, cen.z-ext.z)),
				WorldToGUIPoint(new Vector3(cen.x-ext.x, cen.y-ext.y, cen.z+ext.z)),
				WorldToGUIPoint(new Vector3(cen.x+ext.x, cen.y-ext.y, cen.z+ext.z)),
				WorldToGUIPoint(new Vector3(cen.x-ext.x, cen.y+ext.y, cen.z-ext.z)),
				WorldToGUIPoint(new Vector3(cen.x+ext.x, cen.y+ext.y, cen.z-ext.z)),
				WorldToGUIPoint(new Vector3(cen.x-ext.x, cen.y+ext.y, cen.z+ext.z)),
				WorldToGUIPoint(new Vector3(cen.x+ext.x, cen.y+ext.y, cen.z+ext.z))
		};
		Vector2 min = extentPoints[0];
		Vector2 max = extentPoints[0];
		foreach (Vector2 v in extentPoints)
		{
			min = Vector2.Min(min, v);
			max = Vector2.Max(max, v);
		}
		return new Rect(min.x, min.y, max.x - min.x, max.y - min.y);
	}

	public Rect GUI2dRectWithObject(GameObject go)
	{
		Vector3[] vertices = go.GetComponent<MeshFilter>().mesh.vertices;

		float x1 = float.MaxValue, y1 = float.MaxValue, x2 = 0.0f, y2 = 0.0f;

		foreach (Vector3 vert in vertices)
		{
			Vector2 tmp = WorldToGUIPoint(go.transform.TransformPoint(vert));

			if (tmp.x < x1) x1 = tmp.x;
			if (tmp.x > x2) x2 = tmp.x;
			if (tmp.y < y1) y1 = tmp.y;
			if (tmp.y > y2) y2 = tmp.y;
		}

		Rect bbox = new Rect(x1, y1, x2 - x1, y2 - y1);
		return bbox;
	}

	public Vector2 WorldToGUIPoint(Vector3 world)
	{
		Vector2 screenPoint = Camera.main.WorldToScreenPoint(world);
		screenPoint.y = (float)Screen.height - screenPoint.y;
		return screenPoint;
	}

	public void OnSceneChange()
	{
		if (saveStates)
		{
			swObjectBBox.WriteLine("Object Name, Object Type, Object Color");
		}

		var renderers = Object.FindObjectsOfType<Renderer>();
		var mpb = new MaterialPropertyBlock();
		foreach (var r in renderers)
		{
			var id = r.gameObject.GetInstanceID();
			var layer = r.gameObject.layer;
			var tag = r.gameObject.tag;
			
			if (tag == "LGripper" || tag == "RGripper")
			{
				mpb.SetColor("_ObjectColor", Color.clear);
				mpb.SetColor("_CategoryColor", Color.clear);
				r.SetPropertyBlock(mpb);
				r.enabled = false;
			}
			else
			{
				mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
				mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
				r.SetPropertyBlock(mpb);
			}

			if (saveStates)
			{
				SaveObjectTag SOT = r.gameObject.GetComponent<SaveObjectTag>();
				if (SOT && SOT.SaveMe)
				{
					swObjectBBox.WriteLine(r.gameObject.name + "," + SOT.Tag + "," + ColorUtility.ToHtmlStringRGB(mpb.GetColor("_ObjectColor")));
				}
			}
		}
	}

	public void Save(string filename, int width = -1, int height = -1, string path = "", bool SaveRGB = true, bool SaveDepth = true, bool SaveObjectSegmentation = true, bool SaveCategorySegmentation = true)
	{
		if (width <= 0 || height <= 0)
		{
			width = Screen.width;
			height = Screen.height;
		}

		var filenameExtension = System.IO.Path.GetExtension(filename);
		if (filenameExtension == "")
			filenameExtension = ".png";
		var filenameWithoutExtension = Path.GetFileNameWithoutExtension(filename);

		// var pathWithoutExtension = Path.Combine(path, filenameWithoutExtension);

		// execute as coroutine to wait for the EndOfFrame before starting capture
		StartCoroutine(
			WaitForEndOfFrameAndSave(path, filenameWithoutExtension, filenameExtension, width, height, SaveRGB, SaveDepth, SaveObjectSegmentation, SaveCategorySegmentation));
	}

	private IEnumerator WaitForEndOfFrameAndSave(string path, string filenameWithoutExtension, string filenameExtension, int width, int height, bool SaveRGB = true, bool SaveDepth = true, bool SaveObjectSegmentation = true, bool SaveCategorySegmentation = true)
	{
		yield return new WaitForEndOfFrame();
		Save(path, filenameWithoutExtension, filenameExtension, width, height, SaveRGB, SaveDepth, SaveObjectSegmentation, SaveCategorySegmentation);
	}

	Transform curBaxTransform = null;
	Transform curBaxLHTransform = null;
	Transform curBaxRHTransform = null;
	Transform curBaxHeadTransform = null;

	Transform prevBaxTransform = null;
	Transform prevBaxLHTransform = null;
	Transform prevBaxRHTransform = null;
	Transform prevBaxHeadTransform = null;

	Transform diffBaxTransform = null;
	Transform diffBaxLHTransform = null;
	Transform diffBaxRHTransform = null;
	Transform diffBaxHeadTransform = null;

	string curAnimationControl = "";
	string prevAnimationControl = "";
	bool prevFirstTime = true;

	private string session_sync_path = "Natcomm/Pxx/session_sync.txt";

	private string eulerDeg2Rad(Vector3 angles)
	{
		return (angles.x * Mathf.Deg2Rad).ToString() + "," + (angles.y * Mathf.Deg2Rad).ToString() + "," + (angles.z * Mathf.Deg2Rad).ToString();
	}

	// REFERENCE: https://answers.unity.com/questions/1609179/the-rotation-and-translation-between-2-similar-cub.html
	private Transform ComputeDifference(Transform T1, Transform T2)
	{
		Transform T = new GameObject().transform;
		T.position = T2.position - T1.position;
		T.rotation = T2.rotation * Quaternion.Inverse( T1.rotation );
		return T;
	}
	
	public Transform ApplyDifference(Transform T1, Vector3 positionDifference, Quaternion rotationDifference)
	{
		T1.position += positionDifference;
		T1.rotation = rotationDifference * T1.rotation;
		return T1;
	}

	private string getEnabledCSVText(BioSegment segment)
	{
		string sLine = "";

		if (segment != null && segment.Joint != null)
		{
			sLine += "," + segment.Joint.enabled.ToString();
		}
		else
		{
			sLine += "," + "";
		}

		return sLine;
	}

	private string getCurrentPosRot()
	{
		curBaxTransform = baxOG.transform;
		curBaxLHTransform = baxLH.transform;
		curBaxRHTransform = baxRH.transform;
		curBaxHeadTransform = baxHead.transform;

		string sLine = "";

		// Debug.Log("curBaxTransform: " + curBaxTransform.position.ToString() + " " + curBaxTransform.rotation.ToString());
		// Debug.Log("baxOG.transform: " + baxOG.transform.position.ToString() + " " + baxOG.transform.rotation.ToString());


		// sLine += getEnabledCSVText(baseSegment);						// Base
		if (baseSegment != null)
		{
			sLine += "," + baseSegment.enabled.ToString();
		}
		else
		{
			sLine += "," + "";
		}
		sLine += getEnabledCSVText(laSegment);							// Left Arm
		sLine += getEnabledCSVText(raSegment);							// Right Arm
		sLine += "," + headSegment.Objectives[0].enabled.ToString();	// Head
		sLine += ",";													// Diabled List
		sLine += ",";													// Enabled List

		// Baxter Body
		sLine += "," + curBaxTransform.position.ToString();
		sLine += "," + curBaxTransform.rotation.ToString();
		sLine += "," + curBaxTransform.rotation.eulerAngles.ToString();

		// Baxter Left Hand
		sLine += "," + curBaxLHTransform.position.ToString();
		sLine += "," + curBaxLHTransform.rotation.ToString();
		sLine += "," + curBaxLHTransform.rotation.eulerAngles.ToString();

		// Baxter Right Hand
		sLine += "," + curBaxRHTransform.position.ToString();
		sLine += "," + curBaxRHTransform.rotation.ToString();
		sLine += "," + curBaxRHTransform.rotation.eulerAngles.ToString();

		// Baxter Head
		sLine += "," + curBaxHeadTransform.position.ToString();
		sLine += "," + curBaxHeadTransform.rotation.ToString();
		sLine += "," + curBaxHeadTransform.rotation.eulerAngles.ToString();

		prevBaxTransform = curBaxTransform;
		prevBaxLHTransform = curBaxLHTransform;
		prevBaxRHTransform = curBaxRHTransform;
		prevBaxHeadTransform = curBaxHeadTransform;

		return sLine;
	}

	private string getCurrentMotorAngles()
	{
		string sLine = "";

		for (int i=0; i < bioIKSegmentsWithJoints.Length; i++)
		{
			if (bioJointTargetXYZ[i] == 0)
			{
				// Round TargetValue to 2 decimal places before assigning to bioJointsTargetValues
				bioJointsTargetValues[i] = System.Math.Round(bioIKSegmentsWithJoints[i].Joint.X.TargetValue, 2);
			}
			else if(bioJointTargetXYZ[i] == 1)
			{
				bioJointsTargetValues[i] = System.Math.Round(bioIKSegmentsWithJoints[i].Joint.Y.TargetValue, 2);
			}
			else if(bioJointTargetXYZ[i] == 2)
			{
				bioJointsTargetValues[i] = System.Math.Round(bioIKSegmentsWithJoints[i].Joint.Z.TargetValue, 2);
			}

			sLine += "," + bioJointsTargetValues[i].ToString();
		}

		return sLine;
	}

	private int iTerminationConditionCount = 0;

	private void Save(string path, string filenameWithoutExtension, string filenameExtension, int width, int height, bool SaveRGB = true, bool SaveDepth = true, bool SaveObjectSegmentation = true, bool SaveCategorySegmentation = true)
	{
		bool bStatusChanced = true;
		string[] session_sync_lines = new string[0];
		//double diff = 0.0;

		if (saveStates)
		{
			// Debug.Log("Writing --> " + filenameWithoutExtension);
			// Debug.Log(baxOG.transform.position.ToString() + "--" + baxOG.transform.rotation.ToString());
			sLine = filenameWithoutExtension;

			// curAnimationControl = anim.curAnimationControl;
			// if (anim.currentAnimation != null)
			// {
			// 	// if (anim.currentAnimation.status.Contains("COMPLETED"))
			// 	// {
			// 	curAnimationControl = "\"" + anim.currentAnimation.status + "\"";
			// 	// }
			// }

			if (anim.TestingMode)
			{
				curAnimationControl = anim.AnimStartStopStatus;
				if (prevAnimationControl != curAnimationControl)
				{
					sLine += "," + curAnimationControl;
					prevAnimationControl = curAnimationControl;
					bStatusChanced = true;
				}
				else if (prevAnimationControl == curAnimationControl && curAnimationControl == "UPDATENOW")
				{
					sLine += "," + curAnimationControl;
					bStatusChanced = true;
				}
				else
				{
					sLine += ",";
					bStatusChanced = false;
				}
			}
			else
			{
				curAnimationControl = anim.AnimStartStopStatus;
				if (prevAnimationControl != curAnimationControl)
				{
					sLine += "," + curAnimationControl;
					prevAnimationControl = curAnimationControl;
					bStatusChanced = true;
				}
				else
				{
					sLine += ",";
					bStatusChanced = false;
				}
			}

			// Get Baxter's enabled joints lis, the current transform (world position and rotation)
			sLine += getCurrentPosRot();

			// Get Baxter's the current motor angles (local)
			sLine += getCurrentMotorAngles();

			// // calculate L1 distance between prev_bioJointsTargetValues and bioJointsTargetValues
			// for (int i=0; i < bioJointsTargetValues.Length; i++)
			// {
			// 	diff += System.Math.Abs(prev_bioJointsTargetValues[i] - bioJointsTargetValues[i]);
			// }

			if(bStatusChanced & curAnimationControl == "STARTED")
			{
				// Read pid, ssn, and dbsn from session_sync_path
				session_sync_lines = File.ReadAllLines(session_sync_path);
				// dbsn = session_sync_lines[0].Trim();
				// pid = session_sync_lines[1].Trim();
				// ssn = session_sync_lines[2].Trim();
				

				foreach(string key in objectManipulatedKeys)
				{
					objectManipulated[key] = false;
					objectManiStartTransform[key].position = GameObject.Find(key).transform.position;
				}

				foreach(bool value in objectManipulated.Values)
				{
					sLine+= ",0";
				}
				sLine+= "," + session_sync_lines[0];

			}
			else if (bStatusChanced & curAnimationControl == "STOPPED")
			{
				// Read pid, ssn, and dbsn from session_sync_path
				session_sync_lines = File.ReadAllLines(session_sync_path);

				float[] fDistanceTravelled = new float[objectManipulatedKeys.Count];

				//Find index of max distance travelled
				int i = 0, maxIndex = 0;
				foreach(string key in objectManipulatedKeys)
				{
					fDistanceTravelled[i] = Vector3.Distance(objectManiStartTransform[key].position, GameObject.Find(key).transform.position);

					if (fDistanceTravelled[i] > fDistanceTravelled[maxIndex])
					{
						maxIndex = i;
					}

					i++;
				}

				Debug.Log("Max Distance Travelled by : " + objectManipulatedKeys[maxIndex] + " of " + fDistanceTravelled[maxIndex]);

				i = 0;
				foreach(bool value in objectManipulated.Values)
				{
					if (i == maxIndex)
					{
						if (fDistanceTravelled[maxIndex] > 0.0)
						{
							sLine+= ",1";
						}
						else
						{
							sLine+= ",0";
						}
					}
					else
					{
						sLine+= ",0";
					}
					i++;
				}
				sLine+= "," + session_sync_lines[0];
			}
			else if (anim.TestingMode)
			{
				if(bStatusChanced & (curAnimationControl=="UPDATENOW" || curAnimationControl == "UPDATED"))
				{
					// Read pid, ssn, and dbsn from session_sync_path
					session_sync_lines = File.ReadAllLines(session_sync_path);

					float[] fDistanceTravelled = new float[objectManipulatedKeys.Count];

					//Find index of max distance travelled
					int i = 0, maxIndex = 0;
					foreach(string key in objectManipulatedKeys)
					{
						fDistanceTravelled[i] = Vector3.Distance(objectManiStartTransform[key].position, GameObject.Find(key).transform.position);

						if (fDistanceTravelled[i] > fDistanceTravelled[maxIndex])
						{
							maxIndex = i;
						}

						i++;
					}

					// Debug.Log("Max Distance Travelled by : " + objectManipulatedKeys[maxIndex] + " of " + fDistanceTravelled[maxIndex]);

					i = 0;
					foreach(bool value in objectManipulated.Values)
					{
						if (i == maxIndex)
						{
							if (fDistanceTravelled[maxIndex] > 0.0)
							{
								sLine+= ",1";
							}
							else
							{
								sLine+= ",0";
							}
						}
						else
						{
							sLine+= ",0";
						}
						i++;
					}
					sLine+= "," + session_sync_lines[0];
				}
			}
			else
			{
				foreach(bool value in objectManipulated.Values)
				{
					sLine+= ",";
				}
				sLine+= ",";
			}

			// if (OZ_Banner != null)
			// {
			// 	sLine += "," + OZ_Banner.text;
			// }
			// else
			// {
			// 	sLine += ",";
			// }


			// Debug.Log("diff: " + diff);
			// if (bStatusChanced || diff > 0.05)
			// {
			// 	prev_bioJointsTargetValues = bioJointsTargetValues;
			// }
			// In Testing Mode Update only if bStatusChange (ie STARTED, STOPPED, UPDATENOW)
			if (anim.TestingMode)
			{
				if(bStatusChanced)
				{
					swBaxPosOri.WriteLine(sLine.Replace("(", "").Replace(")", ""));
				}
			}
			else
			{
				swBaxPosOri.WriteLine(sLine.Replace("(", "").Replace(")", ""));
			}



			/*  // INSTEAD POST PROCESS USING COLOR FROM OBJECT ID IMAGE
				var renderers = Object.FindObjectsOfType<Renderer>();
				sBBox = "";
				foreach (var r in renderers)
				{
					var id = r.gameObject.GetInstanceID();
					var layer = r.gameObject.layer;
					var tag = r.gameObject.tag;

					// var hasSaveTag = r.gameObject.find
					SaveObjectTag SOT = r.gameObject.GetComponent<SaveObjectTag>();

					// if (tag.ToLower().Contains("onion") || tag.ToLower().Contains("carrot") || r.gameObject.name.ToLower().Contains("onion") || r.gameObject.name.ToLower().Contains("carrot"))
					if (SOT && SOT.SaveMe)
					{
						var bounds = GUI2dRectWithObject(r.gameObject);

						// Debug.Log(">> " + id + " | " + r.gameObject.name + " | " + tag + " | " + bounds);

						// objectBBox[id] = bounds;
						// objectName[id] = r.gameObject.name;
						sBBox += "," + r.gameObject.name + "," + ColorUtility.ToHtmlStringRGB(objectColors[id]) + "," + bounds.xMin + "," + bounds.yMin + "," + bounds.xMax + "," + bounds.yMax;
					}
				}
				swObjectBBox.WriteLine(sBBox);
			*/
		}

		if(bStatusChanced)
		{
			foreach (var pass in capturePasses)
			{
				// //pass.name == "_img" |
				// if (pass.name == "_normals" | pass.name == "_flow")
				// {}
				// else
				// {
				// 	Save(pass.camera, path, pass.name, filenameWithoutExtension, filenameExtension, width, height, pass.supportsAntialiasing, pass.needsRescale);
				// }

				if (SaveRGB && pass.name == "_img")
				{
					Save(pass.camera, path, pass.name, filenameWithoutExtension, filenameExtension, width, height, pass.supportsAntialiasing, pass.needsRescale);
				}

				if (SaveDepth && pass.name == "_depth")
				{
					Save(pass.camera, path, pass.name, filenameWithoutExtension, filenameExtension, width, height, pass.supportsAntialiasing, pass.needsRescale);
				}

				if (SaveObjectSegmentation && pass.name == "_id")
				{
					Save(pass.camera, path, pass.name, filenameWithoutExtension, filenameExtension, width, height, pass.supportsAntialiasing, pass.needsRescale);
				}

				if (SaveCategorySegmentation && pass.name == "_layer")
				{
					Save(pass.camera, path, pass.name, filenameWithoutExtension, filenameExtension, width, height, pass.supportsAntialiasing, pass.needsRescale);
				}
			}

			if (anim.TestingMode)
			{
				if (anim.AnimStartStopStatus != "UPDATENOW")
				{
					// print("[IMAGE SYNTH SENT] -->> " + anim.AnimStartStopStatus + "," + filenameWithoutExtension);
					anim.udpSocket.SendData(anim.AnimStartStopStatus + "," + filenameWithoutExtension);
				}

				// If Termination done with this Updating after few iterations then change status to UPDATED
				// for now after 5 counts
				if (iTerminationConditionCount > 3)
				{
					anim.TestingModeCommand = "UPDATED";
					iTerminationConditionCount = 0;
				}
				else
				{
					iTerminationConditionCount++;
				}
			}
		}
	}

	private void Save(Camera cam, string path, string name, string filenameWithoutExtension, string filenameExtension, int width, int height, bool supportsAntialiasing, bool needsRescale)
	{
		// string filename = Path.Combine(path + name + "/", filenameWithoutExtension + name + filenameExtension);
		string filename = Path.Combine(path + name + "/", filenameWithoutExtension + filenameExtension);
		var mainCamera = GetComponent<Camera>();
		var depth = 24;
		var format = RenderTextureFormat.Default;
		var readWrite = RenderTextureReadWrite.Default;
		var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

		var finalRT =
			RenderTexture.GetTemporary(width, height, depth, format, readWrite, antiAliasing);
		var renderRT = (!needsRescale) ? finalRT :
			RenderTexture.GetTemporary(mainCamera.pixelWidth, mainCamera.pixelHeight, depth, format, readWrite, antiAliasing);
		var tex = new Texture2D(width, height, TextureFormat.RGB24, false);

		var prevActiveRT = RenderTexture.active;
		var prevCameraRT = cam.targetTexture;

		// render to offscreen texture (readonly from CPU side)
		RenderTexture.active = renderRT;
		cam.targetTexture = renderRT;

		cam.Render();

		if (needsRescale)
		{
			// blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
			RenderTexture.active = finalRT;
			Graphics.Blit(renderRT, finalRT);
			RenderTexture.ReleaseTemporary(renderRT);
		}

		// read offsreen texture contents into the CPU readable texture
		tex.ReadPixels(new Rect(0, 0, tex.width, tex.height), 0, 0);
		tex.Apply();

		// encode texture into PNG
		var bytes = tex.EncodeToPNG();
		File.WriteAllBytes(filename, bytes);					

		// restore state and cleanup
		cam.targetTexture = prevCameraRT;
		RenderTexture.active = prevActiveRT;

		Object.Destroy(tex);
		RenderTexture.ReleaseTemporary(finalRT);
	}

	#if UNITY_EDITOR
	private GameObject lastSelectedGO;
	private int lastSelectedGOLayer = -1;
	private string lastSelectedGOTag = "unknown";
	private bool DetectPotentialSceneChangeInEditor()
	{
		bool change = false;
		// there is no callback in Unity Editor to automatically detect changes in scene objects
		// as a workaround lets track selected objects and check, if properties that are 
		// interesting for us (layer or tag) did not change since the last frame
		if (UnityEditor.Selection.transforms.Length > 1)
		{
			// multiple objects are selected, all bets are off!
			// we have to assume these objects are being edited
			change = true;
			lastSelectedGO = null;
		}
		else if (UnityEditor.Selection.activeGameObject)
		{
			var go = UnityEditor.Selection.activeGameObject;
			// check if layer or tag of a selected object have changed since the last frame
			var potentialChangeHappened = lastSelectedGOLayer != go.layer || lastSelectedGOTag != go.tag;
			if (go == lastSelectedGO && potentialChangeHappened)
				change = true;

			lastSelectedGO = go;
			lastSelectedGOLayer = go.layer;
			lastSelectedGOTag = go.tag;
		}

		return change;
	}
	#endif // UNITY_EDITOR
}


/*
	// DO NOT USE -- The Diff function is not working properly
	// INSTEAD -- Expecting this to be done during post processing...
	private void getCurrentPosRotWithDiff()
	{
		curBaxTransform = baxOG.transform;
		curBaxLHTransform = baxLH.transform;
		curBaxRHTransform = baxRH.transform;
		curBaxHeadTransform = baxHead.transform;

		string sLine = "";

		if (prevFirstTime)
		{
			sLine += "," + curBaxTransform.position.ToString();
			sLine += "," + curBaxTransform.rotation.ToString();
			sLine += "," + curBaxLHTransform.position.ToString();
			sLine += "," + curBaxLHTransform.rotation.ToString();
			sLine += "," + curBaxRHTransform.position.ToString();
			sLine += "," + curBaxRHTransform.rotation.ToString();
			sLine += "," + curBaxHeadTransform.position.ToString();
			sLine += "," + curBaxHeadTransform.rotation.ToString();
			prevFirstTime = false;
		}
		else
		{
			// TODO: Not sure if this actually works or not
			diffBaxTransform = ComputeDifference(curBaxTransform, prevBaxTransform);
			diffBaxLHTransform = ComputeDifference(curBaxLHTransform, prevBaxLHTransform);
			diffBaxRHTransform = ComputeDifference(curBaxRHTransform, prevBaxRHTransform);
			diffBaxHeadTransform = ComputeDifference(curBaxHeadTransform, prevBaxHeadTransform);

			sLine += "," + diffBaxTransform.position.ToString();
			sLine += "," + diffBaxTransform.rotation.ToString();
			sLine += "," + diffBaxLHTransform.position.ToString();
			sLine += "," + diffBaxLHTransform.rotation.ToString();
			sLine += "," + diffBaxRHTransform.position.ToString();
			sLine += "," + diffBaxRHTransform.rotation.ToString();
			sLine += "," + diffBaxHeadTransform.position.ToString();
			sLine += "," + diffBaxHeadTransform.rotation.ToString();
		}

		sLine += "," + curBaxTransform.rotation.eulerAngles.ToString();
		sLine += "," + curBaxLHTransform.rotation.eulerAngles.ToString();
		sLine += "," + curBaxRHTransform.rotation.eulerAngles.ToString();
		sLine += "," + curBaxHeadTransform.rotation.eulerAngles.ToString();

		prevBaxTransform = curBaxTransform;
		prevBaxLHTransform = curBaxLHTransform;
		prevBaxRHTransform = curBaxRHTransform;
		prevBaxHeadTransform = curBaxHeadTransform;

		return sLine;
	}
	*/