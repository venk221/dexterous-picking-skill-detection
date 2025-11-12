using System;
using RosMessageTypes.Geometry;
// using RosMessageTypes.UR3JointStatesMsg;
using RosMessageTypes.Ur3Moveit;
using Unity.Robotics.ROSTCPConnector;
using static Unity.Robotics.ROSTCPConnector.ROSConnection;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
// using RosSharp.Urdf;
using UnityEngine;
using UnityEngine.UI;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

public class Publish_States : MonoBehaviour
{
    const int numRobotJoints = 6;
    private ROSConnection m_ros;
    public static readonly string[] LinkNames = {"world/base_link/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link"};
    // public static readonly string[] actualLinkNames = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    private float timeElapsed;
    [SerializeField]
    string topic_name = "/ur3/joint_states";

    [SerializeField]
    GameObject ur3_with_gripper;

    UrdfJointRevolute[] joint_articulation_bodies;
    public float publishMessageFrequency = 0.5f;

    void Start(){

        m_ros = ROSConnection.GetOrCreateInstance();
        // m_ros = ROSConnection.instance;
        
        m_ros.RegisterPublisher<UR3JointStatesMsg>("/ur3/joint_states");

        joint_articulation_bodies = new UrdfJointRevolute[numRobotJoints];

        var link_name  = string.Empty;

        for (var i = 0; i < numRobotJoints; i++){
            link_name += LinkNames[i];
            joint_articulation_bodies[i] = ur3_with_gripper.transform.Find(link_name).GetComponent<UrdfJointRevolute>();
        }
    }

    // public void Publish(){
    //     var destination_msg = new UR3JointStatesMsg();

    //     for (var i = 0; i < numRobotJoints; i++){
    //         destination_msg.joint_position[i] = joint_articulation_bodies[i].GetPosition();
    //     }

    //     m_ros.Publish(topic_name, destination_msg);
    // }

    public void Update(){

        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            var destination_msg = new UR3JointStatesMsg();
            // Debug.Log("Publishing Joint States");
            for (var i = 0; i < numRobotJoints; i++){
                destination_msg.joint_position[i] = joint_articulation_bodies[i].GetPosition();
            }

            m_ros.Publish(topic_name, destination_msg);
            timeElapsed = 0.0f;
        }
    
    }
}