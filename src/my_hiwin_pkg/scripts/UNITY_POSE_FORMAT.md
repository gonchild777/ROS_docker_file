# Unity 端 Pose 格式說明

## JSON 格式規範

### 完整範例
```json
{
  "points": [
    {
      "position": {
        "x": -0.5,
        "y": 0.8,
        "z": 0.8
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    },
    {
      "position": {
        "x": -0.3,
        "y": 0.6,
        "z": 0.9
      },
      "orientation": {
        "x": 0.0,
        "y": 0.707,
        "z": 0.0,
        "w": 0.707
      }
    }
  ]
}
```

## Unity C# 程式碼範例

### 1. 定義資料結構
```csharp
using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class Position
{
    public float x;
    public float y;
    public float z;
}

[Serializable]
public class Orientation
{
    public float x;
    public float y;
    public float z;
    public float w;
}

[Serializable]
public class PosePoint
{
    public Position position;
    public Orientation orientation;
}

[Serializable]
public class PoseData
{
    public List<PosePoint> points;
}
```

### 2. 從 Transform 建立 Pose
```csharp
public class RobotPosePublisher : MonoBehaviour
{
    // 將 Unity Transform 轉換為 ROS Pose
    public PosePoint TransformToPose(Transform target)
    {
        // Unity 使用左手座標系，ROS 使用右手座標系
        // 需要根據您的座標系設定進行轉換
        
        PosePoint pose = new PosePoint();
        
        // Position (根據需要調整座標系轉換)
        pose.position = new Position
        {
            x = target.position.x,
            y = target.position.y,
            z = target.position.z
        };
        
        // Orientation (Quaternion)
        pose.orientation = new Orientation
        {
            x = target.rotation.x,
            y = target.rotation.y,
            z = target.rotation.z,
            w = target.rotation.w
        };
        
        return pose;
    }
    
    // 建立多點軌跡
    public string CreatePoseJSON(List<Transform> waypoints)
    {
        PoseData poseData = new PoseData();
        poseData.points = new List<PosePoint>();
        
        foreach (Transform waypoint in waypoints)
        {
            poseData.points.Add(TransformToPose(waypoint));
        }
        
        return JsonUtility.ToJson(poseData);
    }
}
```

### 3. 座標系轉換 (Unity → ROS)
```csharp
// 如果需要從 Unity 左手座標系轉換到 ROS 右手座標系
public PosePoint UnityToROSPose(Transform target)
{
    PosePoint pose = new PosePoint();
    
    // 常見轉換: Unity (Y-up) → ROS (Z-up)
    pose.position = new Position
    {
        x = -target.position.z,  // Unity Z → ROS -X
        y = -target.position.x,  // Unity X → ROS -Y
        z = target.position.y    // Unity Y → ROS Z
    };
    
    // Quaternion 轉換
    pose.orientation = new Orientation
    {
        x = target.rotation.z,
        y = target.rotation.x,
        z = -target.rotation.y,
        w = target.rotation.w
    };
    
    return pose;
}
```

### 4. 發送到 ROS (使用 ROS# 或 ROS-TCP-Connector)
```csharp
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

public class ROSPosePublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/target_points_json";
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }
    
    public void SendPosesToROS(List<Transform> waypoints)
    {
        string jsonData = CreatePoseJSON(waypoints);
        
        // 使用 String message
        StringMsg msg = new StringMsg(jsonData);
        ros.Publish(topicName, msg);
        
        Debug.Log($"Sent {waypoints.Count} poses to ROS: {jsonData}");
    }
}
```

### 5. 實用函數：建立特定姿態
```csharp
// 末端執行器朝下（適合抓取）
public Orientation GetGraspingOrientation()
{
    // 四元數表示 180 度繞 X 軸旋轉（末端朝下）
    return new Orientation
    {
        x = 1.0f,
        y = 0.0f,
        z = 0.0f,
        w = 0.0f
    };
}

// 末端執行器水平（適合推動）
public Orientation GetHorizontalOrientation()
{
    // 四元數表示 90 度繞 Y 軸旋轉
    return new Orientation
    {
        x = 0.0f,
        y = 0.707f,
        z = 0.0f,
        w = 0.707f
    };
}

// 無旋轉（預設姿態）
public Orientation GetDefaultOrientation()
{
    return new Orientation
    {
        x = 0.0f,
        y = 0.0f,
        z = 0.0f,
        w = 1.0f
    };
}
```

## 測試範例

### 單點測試
```json
{
  "points": [
    {
      "position": {"x": -0.5, "y": 0.8, "z": 0.8},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  ]
}
```

### 多點軌跡測試
```json
{
  "points": [
    {
      "position": {"x": -0.5, "y": 0.8, "z": 0.8},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
      "position": {"x": -0.4, "y": 0.7, "z": 0.9},
      "orientation": {"x": 0.0, "y": 0.707, "z": 0.0, "w": 0.707}
    },
    {
      "position": {"x": -0.3, "y": 0.6, "z": 1.0},
      "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}
    }
  ]
}
```

## 注意事項

1. **座標系統**：確認 Unity 和 ROS 的座標系統對應關係
2. **單位**：Unity 通常使用公尺 (meter)，與 ROS 相同
3. **四元數歸一化**：確保 `x² + y² + z² + w² = 1`
4. **旋轉順序**：Unity 和 ROS 都使用 Quaternion，應該能直接對應
5. **IK 可達性**：某些姿態組合可能無法達到，需要在 Unity 端預先驗證

## 除錯技巧

在 Unity Console 中檢查發送的 JSON：
```csharp
Debug.Log($"Sending pose: {JsonUtility.ToJson(poseData, true)}");
```

在 ROS 端監聽：
```bash
rostopic echo /target_points_json
```
