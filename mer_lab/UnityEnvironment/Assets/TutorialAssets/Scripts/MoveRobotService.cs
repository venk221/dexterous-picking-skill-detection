using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using static Unity.Robotics.ROSTCPConnector.ROSConnection;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Ur3Moveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

public class MoveRobotService : MonoBehaviour
{
    private ROSConnection m_ros;

    private int numRobotJoints = 6;
    private readonly float jointAssignmentWait = 0.1f;

    [SerializeField]
    string rosServiceName = "/ur3_moveit/MoveRobot";

    [SerializeField]
    GameObject ur3_with_gripper;
    
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    bool executed_trajectories = false;
    bool resetted_robots = false;
    void Start()
    {
        m_ros = ROSConnection.GetOrCreateInstance();

        m_ros.ImplementService<MoveRobotRequest,MoveRobotResponse>(rosServiceName, CallBack);
        m_ros.ImplementService<resetRobotRequest, resetRobotResponse>("ur3_moveit/ResetRobot", ResetCallback);
    }

    private MoveRobotResponse CallBack(MoveRobotRequest request)
    {
        Debug.Log("Received Request to Move Robot");
        executed_trajectories = false;
        resetted_robots = false;
        MoveRobotResponse response = new MoveRobotResponse();
        response.success = true;
        if (request.trajectories.Length == 0){
            Debug.Log("Empty Trajectory, Returing false");
            response.success = false;
        }
        else{
            Debug.Log("Executing trajectory");
            StartCoroutine(ExecuteTrajectories(request));
            response.success = true;
        }
    
        return response;
    }

    private resetRobotResponse ResetCallback(resetRobotRequest request){
        resetRobotResponse response = new resetRobotResponse();
        StartCoroutine(MoveToInitialPosition());
        response.success = true;
        return response;
    }

    private IEnumerator ExecuteTrajectories(MoveRobotRequest request)
    {
        for (int poseIndex = 0; poseIndex < request.trajectories.Length; poseIndex++)
        {
          for (int jointConfigIndex = 0; jointConfigIndex < request.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = request.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                // Set the joint values for every joint
                for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive = jointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    jointArticulationBodies[joint].xDrive = joint1XDrive;
                }
                // Wait for robot to achieve pose for all joint assignments
                yield return new WaitForSeconds(jointAssignmentWait);
            }  
        }
        // StartCoroutine(MoveToInitialPosition());
        executed_trajectories = true;
    }
    
    private IEnumerator MoveToInitialPosition()
    {
        Debug.Log("resetting robot");
        bool isRotationFinished = false;
        while (!isRotationFinished)
        {
            isRotationFinished = ResetRobotToDefaultPosition();
            yield return new WaitForSeconds(jointAssignmentWait);
            // yield return new WaitForSeconds(jointAssignmentWait);
        }
        // ServiceButton.interactable = true;
        resetted_robots = true;
    }

    private bool ResetRobotToDefaultPosition()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 180f;

        for (int i = 0; i < numRobotJoints; i++)
        {
            var tempXDrive = jointArticulationBodies[i].xDrive;
            float currentRotation = tempXDrive.target;

            float rotationChange = rotationSpeed * Time.fixedDeltaTime;

            if (currentRotation > 0f) rotationChange *= -1;

            if (Mathf.Abs(currentRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;

            // the new xDrive target is the currentRotation summed with the desired change
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            jointArticulationBodies[i].xDrive = tempXDrive;
        }
        return isRotationFinished;
    }

    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = ur3_with_gripper.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = ur3_with_gripper.transform.Find(arm_link).GetComponent<ArticulationBody>();

        string elbow_link = arm_link + "/forearm_link";
        jointArticulationBodies[2] = ur3_with_gripper.transform.Find(elbow_link).GetComponent<ArticulationBody>();

        string forearm_link = elbow_link + "/wrist_1_link";
        jointArticulationBodies[3] = ur3_with_gripper.transform.Find(forearm_link).GetComponent<ArticulationBody>();

        string wrist_link = forearm_link + "/wrist_2_link";
        jointArticulationBodies[4] = ur3_with_gripper.transform.Find(wrist_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_link + "/wrist_3_link";
        jointArticulationBodies[5] = ur3_with_gripper.transform.Find(hand_link).GetComponent<ArticulationBody>();

        // articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

        // var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
        // gripperJoints = new List<ArticulationBody>();

        // foreach (ArticulationBody articulationBody in robot.GetComponentsInChildren<ArticulationBody>())
        // {
        //     if (gripperJointNames.Contains(articulationBody.name))
        //     {
        //         gripperJoints.Add(articulationBody);
        //     }
        // }
    }
}