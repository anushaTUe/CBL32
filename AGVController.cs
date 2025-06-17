using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using Codice.Client.BaseCommands.WkStatus.Printers;
using rcl_interfaces.msg;


namespace RosSharp.Control
{
    public enum ControlMode { Keyboard, ROS };

    public class AGVController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public ControlMode mode = ControlMode.ROS;

        private ArticulationBody wA1;
        private ArticulationBody wA2;

        public float maxLinearSpeed = 2;
        public float maxRotationalSpeed = 1;
        public float wheelRadius = 0.033f;
        public float trackWidth = 0.288f;
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;
        

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            ros.RegisterPublisher<TwistMsg>("cmd_vel");
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            if (cmdVel.linear.x == 0 && cmdVel.angular.z == 0)
            {
                rosLinear = 0;
                rosAngular = 0;
            }

            if (cmdVel.angular.z == 0)
            {
                rosAngular = 0;
            } else if (cmdVel.angular.z < 0)
            {
                rosAngular = (float)(cmdVel.angular.z - 0.18);
            } else
            {
                rosAngular = (float)(cmdVel.angular.z + 0.18);
            }

            if (cmdVel.linear.x == 0)
            {
                rosLinear = 0;
            } else if (cmdVel.linear.x > 0)
            {
                rosLinear = (float)(cmdVel.linear.x + 0.02);
            } else
            {
                rosLinear = (float)(cmdVel.linear.x - 0.02);
            }
            lastCmdReceived = Time.time;
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.targetVelocity = float.IsNaN(wheelSpeed)
                ? ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg
                : wheelSpeed;
            joint.xDrive = drive;
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wheel1Rotation = (wheel1Rotation + wheelSpeedDiff) * Mathf.Rad2Deg;
                wheel2Rotation = (wheel2Rotation - wheelSpeedDiff) * Mathf.Rad2Deg;
            }
            else
            {
                wheel1Rotation *= Mathf.Rad2Deg;
                wheel2Rotation *= Mathf.Rad2Deg;
            }

            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }

        public void Move(Vector3 point)
        {
            Transform baseLink = transform.GetChild(0).GetChild(2);
            float angle = Mathf.Atan2(point.x - baseLink.position.x, point.z - baseLink.position.z) * Mathf.Rad2Deg;
            if (angle < 0) {angle += 360;}

            float baseAngle = (baseLink.rotation.eulerAngles.y + 180) % 360f;
            
            float angleDiff = angle - baseAngle;
            if (angleDiff > 180) angleDiff -= 360;
            if (angleDiff < -180) angleDiff += 360;

            TwistMsg twist = new TwistMsg();
            if (Mathf.Abs(angleDiff) < 2.5f)
            {
                twist.linear.x = -maxLinearSpeed;
                twist.angular.z = 0;
            }
            else if (angleDiff > 0)
            {
                twist.linear.x = 0;
                twist.angular.z = 0.75;
            }
            else
            {
                twist.linear.x = 0;
                twist.angular.z = -0.75;
            }
            ros.Publish("cmd_vel", twist);
            //remove these for actual robot
            ReceiveROSCmd(twist);
            RobotInput(rosLinear, rosAngular);
        }

        public void Stop()
        {
            TwistMsg twist = new TwistMsg();
            twist.linear.x = 0;
            twist.angular.x = 0;
            ros.Publish("cmd_vel", twist);
            ReceiveROSCmd(twist);
            RobotInput(rosLinear, rosAngular);
        }
    }
}
