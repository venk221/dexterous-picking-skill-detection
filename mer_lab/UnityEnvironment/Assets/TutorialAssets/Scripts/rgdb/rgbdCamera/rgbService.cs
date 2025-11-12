using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;
using RosMessageTypes.Ur3Moveit;

public class rgbService : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    string rosServiceName = "/ur3_moveit/getrgbimage";

    public Camera RGBCamera;

    public string FrameId = "unity_camera/rgb_frame";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    [Range(0, 100)]
    public int qualityLevel = 50;
    private Texture2D texture2D;
    private Rect rect;
    private uint seq_num = 0;
    private CompressedImageMsg message;


    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        ros.ImplementService<getrgbimageRequest, getrgbimageResponse>(rosServiceName, Callback);
        // ros.RegisterPublisher<CompressedImageMsg>(ImagetopicName);
        // ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopicName);

        // Initialize game Object
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        RGBCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        // Camera.onPostRender += UpdateImage;
    }

    private getrgbimageResponse Callback(getrgbimageRequest request)
    {
        Debug.Log("Received RGB image request");

        texture2D.ReadPixels(rect, 0, 0);
            // var timestamp = new TimeStamp(Clock.time);
            // Message
            var timestamp = new TimeStamp(Clock.time);
            CompressedImageMsg message = new CompressedImageMsg
            {
                header = new HeaderMsg
                {
                    seq = seq_num,
                    frame_id = FrameId,
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                format = "jpeg",
                data = texture2D.EncodeToJPG(qualityLevel)
            };
            seq_num += 1;

            // Finally send the message to server_endpoint.py running in ROS
            // ros.Publish(ImagetopicName, message);
            getrgbimageResponse response = new getrgbimageResponse();
            // Camera Info message
            CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(RGBCamera, message.header, 0.0f, 0.01f);
            // ros.Publish(cameraInfoTopicName, cameraInfoMessage);
            response.rgbimage = message;
            response.rgbcamerainfo = cameraInfoMessage;
            return response;
    }


}