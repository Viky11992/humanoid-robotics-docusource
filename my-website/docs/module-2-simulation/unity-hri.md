---
sidebar_position: 3
title: Unity for Human-Robot Interaction
---

# Unity for Human-Robot Interaction

## Introduction to Unity in Robotics

Unity provides a powerful platform for creating high-fidelity visualizations and human-robot interaction (HRI) scenarios. Unlike Gazebo which focuses on physics accuracy, Unity excels in:

- **Photorealistic rendering**: High-quality visuals for demonstrations
- **Immersive environments**: Complex scenes with detailed textures and lighting
- **Interactive interfaces**: User interfaces for robot control and monitoring
- **VR/AR capabilities**: Virtual and augmented reality applications
- **Multi-user environments**: Collaborative spaces for team interaction

## Unity for Robotics Setup

### 1. Installing Unity Robotics Package

Unity provides the **Unity Robotics Hub** and **Unity Robotics Package** for robotics integration:

1. Install Unity Hub from unity.com
2. Install Unity 2021.3 LTS or later
3. Install the Unity Robotics Package from the Package Manager
4. Import the ROS-TCP-Connector for ROS communication

### 2. Basic Unity Robotics Scene Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    // ROS connection
    private ROSConnection ros;

    // Robot joint positions
    public float[] jointPositions = new float[6];

    // Joint objects in Unity scene
    public Transform[] jointObjects;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.instance;

        // Subscribe to joint state topic
        ros.Subscribe<sensor_msgs.JointStateMsg>("joint_states", JointStateCallback);
    }

    void JointStateCallback(sensor_msgs.JointStateMsg jointState)
    {
        // Update joint positions from ROS message
        for (int i = 0; i < jointState.position.Length && i < jointObjects.Length; i++)
        {
            jointPositions[i] = (float)jointState.position[i];

            // Apply rotation to Unity objects
            jointObjects[i].localRotation = Quaternion.Euler(0, 0, jointPositions[i] * Mathf.Rad2Deg);
        }
    }

    void Update()
    {
        // Update visual representation based on joint positions
        UpdateRobotVisuals();
    }

    void UpdateRobotVisuals()
    {
        // Update any visual elements based on current state
        // This could include highlighting active joints, showing force feedback, etc.
    }
}
```

## Creating HRI Scenarios in Unity

### 1. User Interface for Robot Control

```csharp
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotUIController : MonoBehaviour
{
    public Slider[] jointSliders;
    public Button moveButton;
    public InputField targetXInput;
    public InputField targetYInput;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;

        // Set up button listeners
        moveButton.onClick.AddListener(MoveToTarget);

        // Set up slider listeners
        foreach (var slider in jointSliders)
        {
            slider.onValueChanged.AddListener(OnJointSliderChanged);
        }
    }

    void MoveToTarget()
    {
        // Parse target coordinates
        float targetX = float.Parse(targetXInput.text);
        float targetY = float.Parse(targetYInput.text);

        // Create and send goal message
        var goalMsg = new geometry_msgs.PoseMsg();
        goalMsg.position.x = targetX;
        goalMsg.position.y = targetY;

        ros.Publish("/move_base_simple/goal", goalMsg);
    }

    void OnJointSliderChanged(float value)
    {
        // Send individual joint commands
        // This would typically be done as a full joint trajectory
    }
}
```

### 2. Interactive Environment Objects

```csharp
using UnityEngine;

public class InteractiveObject : MonoBehaviour
{
    public bool isGraspable = true;
    public string objectType = "object";

    private bool isSelected = false;
    private Renderer objectRenderer;

    void Start()
    {
        objectRenderer = GetComponent<Renderer>();
    }

    void OnMouseDown()
    {
        if (isGraspable)
        {
            SelectObject();
        }
    }

    void SelectObject()
    {
        isSelected = true;

        // Change appearance to indicate selection
        objectRenderer.material.color = Color.yellow;

        // Send message to robot to grasp this object
        SendGraspCommand();
    }

    void SendGraspCommand()
    {
        // In a real implementation, this would send a ROS message
        // to the robot's manipulation system
        Debug.Log($"Requesting grasp of {objectType} at {transform.position}");
    }

    void OnMouseUp()
    {
        if (isSelected)
        {
            DeselectObject();
        }
    }

    void DeselectObject()
    {
        isSelected = false;
        objectRenderer.material.color = Color.white;
    }
}
```

## Unity Scene Architecture for Humanoid Robots

### 1. Robot Model Import and Setup

```csharp
using UnityEngine;

public class HumanoidRobotModel : MonoBehaviour
{
    // Humanoid joint mapping
    [Header("Torso")]
    public Transform pelvis;
    public Transform torso;
    public Transform chest;
    public Transform neck;
    public Transform head;

    [Header("Left Arm")]
    public Transform leftShoulder;
    public Transform leftUpperArm;
    public Transform leftLowerArm;
    public Transform leftHand;

    [Header("Right Arm")]
    public Transform rightShoulder;
    public Transform rightUpperArm;
    public Transform rightLowerArm;
    public Transform rightHand;

    [Header("Left Leg")]
    public Transform leftHip;
    public Transform leftUpperLeg;
    public Transform leftLowerLeg;
    public Transform leftFoot;

    [Header("Right Leg")]
    public Transform rightHip;
    public Transform rightUpperLeg;
    public Transform rightLowerLeg;
    public Transform rightFoot;

    public void UpdatePose(float[] jointAngles)
    {
        // Update all joints based on provided angles
        // This is a simplified example - real implementation would use
        // forward kinematics or animation systems

        if (jointAngles.Length >= 18) // Example: 18 DOF humanoid
        {
            // Update left arm
            leftShoulder.localRotation = Quaternion.Euler(0, 0, jointAngles[0]);
            leftUpperArm.localRotation = Quaternion.Euler(0, 0, jointAngles[1]);
            leftLowerArm.localRotation = Quaternion.Euler(0, 0, jointAngles[2]);

            // Update right arm
            rightShoulder.localRotation = Quaternion.Euler(0, 0, jointAngles[3]);
            rightUpperArm.localRotation = Quaternion.Euler(0, 0, jointAngles[4]);
            rightLowerArm.localRotation = Quaternion.Euler(0, 0, jointAngles[5]);

            // Continue for other joints...
        }
    }
}
```

### 2. HRI Environment Design

```csharp
using UnityEngine;
using System.Collections.Generic;

public class HRIEnvironment : MonoBehaviour
{
    [Header("Interactive Elements")]
    public List<GameObject> interactiveObjects;
    public Transform[] waypoints;
    public GameObject[] furniture;

    [Header("Human Avatar")]
    public GameObject humanAvatar;

    [Header("Robot Interaction Zones")]
    public List<Collider> interactionZones;

    [Header("Safety Boundaries")]
    public List<Collider> safetyBoundaries;

    void Start()
    {
        SetupEnvironment();
        SetupInteractionZones();
        SetupSafetyBoundaries();
    }

    void SetupEnvironment()
    {
        // Initialize all interactive elements
        foreach (var obj in interactiveObjects)
        {
            var interactive = obj.GetComponent<InteractiveObject>();
            if (interactive == null)
            {
                interactive = obj.AddComponent<InteractiveObject>();
            }
        }
    }

    void SetupInteractionZones()
    {
        // Configure zones where robot can interact with objects
        foreach (var zone in interactionZones)
        {
            zone.isTrigger = true;
            // Add interaction zone behavior
        }
    }

    void SetupSafetyBoundaries()
    {
        // Configure safety boundaries to prevent robot from entering
        // dangerous areas
        foreach (var boundary in safetyBoundaries)
        {
            boundary.isTrigger = true;
            // Add safety boundary behavior
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (safetyBoundaries.Contains(other))
        {
            Debug.LogWarning("Robot entered safety boundary!");
            // In a real implementation, this would send a safety message to ROS
        }
    }
}
```

## Advanced HRI Features

### 1. Gesture Recognition Interface

```csharp
using UnityEngine;
using UnityEngine.XR;
using System.Collections.Generic;

public class GestureRecognition : MonoBehaviour
{
    public Camera mainCamera;
    public GameObject gestureVisualization;

    private List<Vector3> gesturePath = new List<Vector3>();
    private bool isRecordingGesture = false;

    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            StartGestureRecording();
        }

        if (Input.GetMouseButton(0))
        {
            RecordGesturePoint();
        }

        if (Input.GetMouseButtonUp(0))
        {
            EndGestureRecording();
        }
    }

    void StartGestureRecording()
    {
        isRecordingGesture = true;
        gesturePath.Clear();
        gestureVisualization.SetActive(true);
    }

    void RecordGesturePoint()
    {
        if (isRecordingGesture)
        {
            Vector3 mousePos = Input.mousePosition;
            mousePos.z = 10.0f; // Distance from camera
            Vector3 worldPos = mainCamera.ScreenToWorldPoint(mousePos);

            gesturePath.Add(worldPos);

            // Update visualization
            UpdateGestureVisualization();
        }
    }

    void EndGestureRecognition()
    {
        isRecordingGesture = false;
        gestureVisualization.SetActive(false);

        // Process the recorded gesture
        ProcessGesture();
    }

    void UpdateGestureVisualization()
    {
        // Update the visual representation of the gesture
        // This could be a line renderer showing the gesture path
    }

    void ProcessGesture()
    {
        // Analyze the gesture path and determine the intended command
        string gestureCommand = AnalyzeGesturePath(gesturePath);

        if (!string.IsNullOrEmpty(gestureCommand))
        {
            SendGestureCommand(gestureCommand);
        }
    }

    string AnalyzeGesturePath(List<Vector3> path)
    {
        // Simple gesture recognition logic
        // In practice, this would use more sophisticated algorithms
        if (path.Count < 5) return null; // Too short to be meaningful

        // Example: Recognize simple shapes
        if (IsCircle(path)) return "CIRCLE_COMMAND";
        if (IsLine(path)) return "LINE_COMMAND";
        if (IsWave(path)) return "WAVE_COMMAND";

        return null;
    }

    void SendGestureCommand(string command)
    {
        // Send the recognized gesture command to the robot via ROS
        Debug.Log($"Sending gesture command: {command}");
    }

    bool IsCircle(List<Vector3> path)
    {
        // Simple circle detection logic
        // In practice, use more sophisticated shape recognition
        return false;
    }

    bool IsLine(List<Vector3> path)
    {
        // Simple line detection logic
        return false;
    }

    bool IsWave(List<Vector3> path)
    {
        // Simple wave detection logic
        return false;
    }
}
```

### 2. Voice Command Integration

```csharp
using UnityEngine;
using System.Collections;
using UnityEngine.Windows.Speech;

public class VoiceCommandHandler : MonoBehaviour
{
    [Header("Voice Commands")]
    public string[] moveCommands = {"move forward", "move backward", "turn left", "turn right"};
    public string[] actionCommands = {"wave", "point", "sit", "stand"};

    private KeywordRecognizer keywordRecognizer;
    private Dictionary<string, System.Action> keywords = new Dictionary<string, System.Action>();

    void Start()
    {
        SetupVoiceCommands();
    }

    void SetupVoiceCommands()
    {
        // Setup movement commands
        foreach (string command in moveCommands)
        {
            keywords.Add(command, () => HandleMoveCommand(command));
        }

        // Setup action commands
        foreach (string command in actionCommands)
        {
            keywords.Add(command, () => HandleActionCommand(command));
        }

        // Initialize the keyword recognizer
        keywordRecognizer = new KeywordRecognizer(keywords.Keys.ToArray());
        keywordRecognizer.OnPhraseRecognized += OnPhraseRecognized;
        keywordRecognizer.Start();
    }

    void OnPhraseRecognized(PhraseRecognizedEventArgs args)
    {
        System.Action keywordAction;
        if (keywords.TryGetValue(args.text, out keywordAction))
        {
            keywordAction.Invoke();
        }
    }

    void HandleMoveCommand(string command)
    {
        Debug.Log($"Received move command: {command}");
        // Send movement command to robot via ROS
        SendRobotCommand(command);
    }

    void HandleActionCommand(string command)
    {
        Debug.Log($"Received action command: {command}");
        // Send action command to robot via ROS
        SendRobotCommand(command);
    }

    void SendRobotCommand(string command)
    {
        // In a real implementation, this would send a ROS message
        // to the robot's command interface
    }

    void OnDestroy()
    {
        if (keywordRecognizer != null && keywordRecognizer.IsRunning)
        {
            keywordRecognizer.Stop();
        }
    }
}
```

## Integration with ROS

### 1. ROS-TCP-Connector Setup

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnityROSBridge : MonoBehaviour
{
    [Header("ROS Topics")]
    public string jointStateTopic = "joint_states";
    public string cmdVelTopic = "cmd_vel";
    public string robotStateTopic = "robot_state";

    private ROSConnection ros;

    // Robot state
    private JointStateMsg currentJointState;
    private PoseMsg currentPose;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to robot state topics
        ros.Subscribe<JointStateMsg>(jointStateTopic, OnJointStateReceived);
        ros.Subscribe<PoseMsg>(robotStateTopic, OnRobotPoseReceived);
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        currentJointState = jointState;
        UpdateRobotInScene();
    }

    void OnRobotPoseReceived(PoseMsg pose)
    {
        currentPose = pose;
        UpdateRobotPositionInScene();
    }

    void UpdateRobotInScene()
    {
        // Update the Unity representation of the robot
        // based on the received joint states
    }

    void UpdateRobotPositionInScene()
    {
        // Update the robot's position in the Unity scene
        // based on the received pose
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        var cmdVel = new geometry_msgs.TwistMsg();
        cmdVel.linear = new geometry_msgs.Vector3Msg(linearX, 0, 0);
        cmdVel.angular = new geometry_msgs.Vector3Msg(0, 0, angularZ);

        ros.Publish(cmdVelTopic, cmdVel);
    }
}
```

## Performance Optimization for HRI

### 1. Rendering Optimization

```csharp
using UnityEngine;

public class HRIPerformanceManager : MonoBehaviour
{
    [Header("Performance Settings")]
    public int maxFramerate = 60;
    public int targetFramerate = 30;
    public bool enableLOD = true;

    [Header("Quality Settings")]
    public int shadowResolution = 2;  // 0=Low, 1=Medium, 2=High, 3=Very High
    public int textureQuality = 1;    // 0=Low, 1=Medium, 2=High, 3=Ultra

    void Start()
    {
        ApplyPerformanceSettings();
    }

    void ApplyPerformanceSettings()
    {
        // Set maximum framerate to prevent excessive CPU usage
        Application.targetFrameRate = targetFramerate;
        QualitySettings.vSyncCount = 0;  // Disable VSync for consistent frame timing

        // Apply quality settings
        QualitySettings.shadowResolution = (ShadowResolution)shadowResolution;
        QualitySettings.masterTextureLimit = textureQuality;

        // Enable LOD if supported
        if (enableLOD)
        {
            GetComponent<LODGroup>().enabled = true;
        }
    }

    void Update()
    {
        // Monitor performance and adjust if needed
        float currentFPS = 1.0f / Time.deltaTime;

        if (currentFPS < targetFramerate * 0.8f)
        {
            // Performance is degrading, consider reducing quality
            ReduceQualitySettings();
        }
    }

    void ReduceQualitySettings()
    {
        // Reduce rendering quality to maintain performance
        QualitySettings.shadowDistance = Mathf.Max(50, QualitySettings.shadowDistance * 0.8f);
    }
}
```

### 2. Network Optimization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

public class OptimizedROSConnection : MonoBehaviour
{
    [Header("Optimization Settings")]
    public float updateInterval = 0.1f;  // 10 Hz update rate
    public bool compressMessages = true;
    public bool batchMessages = true;

    private ROSConnection ros;
    private float lastUpdateTime = 0f;
    private List<object> messageBatch = new List<object>();

    void Start()
    {
        ros = ROSConnection.instance;
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            SendBatchedMessages();
            lastUpdateTime = Time.time;
        }
    }

    public void QueueMessage(string topic, object message)
    {
        if (batchMessages)
        {
            messageBatch.Add(new { Topic = topic, Message = message });
        }
        else
        {
            ros.Publish(topic, message);
        }
    }

    void SendBatchedMessages()
    {
        if (messageBatch.Count > 0)
        {
            foreach (var item in messageBatch)
            {
                var msgData = (dynamic)item;
                ros.Publish(msgData.Topic, msgData.Message);
            }
            messageBatch.Clear();
        }
    }
}
```

## Safety and Validation in Unity HRI

### 1. Safety Zone Management

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SafetyZoneManager : MonoBehaviour
{
    [Header("Safety Configuration")]
    public List<SafetyZone> safetyZones = new List<SafetyZone>();
    public float safetyCheckInterval = 0.1f;

    private float lastSafetyCheck = 0f;
    private List<GameObject> robotParts = new List<GameObject>();

    void Update()
    {
        if (Time.time - lastSafetyCheck >= safetyCheckInterval)
        {
            PerformSafetyCheck();
            lastSafetyCheck = Time.time;
        }
    }

    void PerformSafetyCheck()
    {
        foreach (var zone in safetyZones)
        {
            foreach (var robotPart in robotParts)
            {
                if (zone.Contains(robotPart.transform.position))
                {
                    TriggerSafetyAlert(zone, robotPart);
                }
            }
        }
    }

    void TriggerSafetyAlert(SafetyZone zone, GameObject robotPart)
    {
        Debug.LogWarning($"SAFETY VIOLATION: {robotPart.name} entered {zone.zoneName}");

        // In a real implementation, this would send an emergency stop
        // command to the physical robot via ROS
        SendEmergencyStop();
    }

    void SendEmergencyStop()
    {
        // Send emergency stop command to robot
        Debug.Log("EMERGENCY STOP SENT TO ROBOT");
    }
}

[System.Serializable]
public class SafetyZone
{
    public string zoneName;
    public Vector3 center;
    public Vector3 size;
    public SafetyZoneType zoneType;

    public bool Contains(Vector3 point)
    {
        Vector3 min = center - size / 2f;
        Vector3 max = center + size / 2f;

        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }
}

public enum SafetyZoneType
{
    NoEntry,
    Warning,
    Emergency
}
```

## Next Steps

Continue to the next section to learn about sensor modeling in simulation.