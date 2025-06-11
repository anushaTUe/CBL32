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

        private MapManager mapManager;

        public int frontierCount = 0;
        private Vector3 currentGoal;
        private bool navigatingToGoal = false;
        private float goalTolerance = 0.05f;

        private int[,] map;
        private int gridSize = 100;
        private float cellSize = 0.01f;

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            ros.RegisterPublisher<TwistMsg>("cmd_vel");
            mapManager = FindObjectOfType<MapManager>();
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            if (cmdVel.linear.x == 0 && cmdVel.angular.z == 0)
            {
                rosLinear = 0;
                rosAngular = 0;
            }

            else if (cmdVel.linear.x < 0)
            {
                rosLinear = (float)(cmdVel.linear.x - 0.02);
            }
            else if (cmdVel.angular.z < 0)
            {
                rosAngular = (float)(cmdVel.angular.z - 0.18);
            }
            else if (cmdVel.linear.x > 0)
            {
                rosLinear = (float)(cmdVel.linear.x + 0.02);
            }
            else if (cmdVel.angular.z > 0)
            {
                rosAngular = (float)(cmdVel.angular.z + 0.18);
            }
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            //Move(new Vector3(-10, 0, 0));
            /*
            if (mode == ControlMode.Keyboard)
            {
                KeyBoardUpdate();
            }
            else if (mode == ControlMode.ROS)
            {
                ROSUpdate();
            }     
            */

            if (!navigatingToGoal)
            {
                Vector3 nextFrontier;
                if (FindNextFrontier(out nextFrontier))
                {
                    currentGoal = nextFrontier;
                    navigatingToGoal = true;
                    Move(currentGoal);
                }
                else
                {
                    Debug.Log("no frontiers left");
                }
            }
            else
            {
                if (HasReachedGoal(currentGoal))
                {
                    navigatingToGoal = false;
                }
                else
                {
                    Move(currentGoal);
                }
            }
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

        /*
        private void RobotInput(float speed, float rotSpeed)
        {
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);

            float wheel1Rotation = speed / wheelRadius;
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = (rotSpeed * trackWidth) / wheelRadius;

            TwistMsg twist = new TwistMsg();
            twist.linear.x = inputSpeed;
            twist.angular.z = inputRotationSpeed;

            ros.Publish("cmd_vel", twist);
        }
        */



        private void ROSUpdate()
        {

            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            Debug.Log("linear: " + rosLinear);
            Debug.Log("angular: " + rosAngular);
            RobotInput(rosLinear, -rosAngular);
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
            if (angle < 0) angle += 360;

            Debug.Log("Angle Before: " + angle);
            if (point.x >= baseLink.position.x)
            {
                if (point.z < baseLink.position.z)
                {
                    angle = 180 - angle;
                }
            }
            else if (point.x < baseLink.position.x)
            {
                if (point.z > baseLink.position.z)
                {
                    angle = 360 - angle;
                }
                else
                {
                    angle += 180;
                }
            }

            Debug.Log("Angle: " + angle);
            float baseAngle = baseLink.rotation.eulerAngles.y - 270;
            if (baseAngle < 0) baseAngle += 360;

            float angleDiff = angle - baseAngle;
            if (angleDiff > 180) angleDiff -= 360;
            if (angleDiff < -180) angleDiff += 360;

            TwistMsg twist = new TwistMsg();
            if (Mathf.Abs(angleDiff) < 5)
            {
                Debug.Log("Forward");
                twist.linear.x = maxLinearSpeed;
            }
            else if (angleDiff > 0)
            {
                twist.angular.z = 0.75;
            }
            else
            {
                twist.angular.z = -0.75;
            }
            ros.Publish("cmd_vel", twist);
            ReceiveROSCmd(twist);
            RobotInput(rosLinear, rosAngular);
        }

        /// <summary>
        /// 
        /// this method determines if a given coordinate in a map is a frontier for the robot to
        /// explore or not.
        /// 
        /// this method can work for basically any map. however, here this will only work for the 
        /// occupation grid style map, where cells are labelled accordingly:
        /// 1 = occupied, 0 = free, -1 = unknown
        /// 
        /// a frontier here is defined as a free cell (value 0 in this case) next to at least one
        /// unexplored cell (value -1 in this case)
        /// 
        /// THIS METHOD MUST LOOP THROUGH ALL THE COORDINATES IN THE GIVEN MAP TO DETERMINE FRONTIERS.
        /// IT WILL RETURN TRUE IF A COORDINATE IS INDEED A FRONTIER.
        /// IN A LARGER FRONTIER ALGORITHM METHOD, IT CAN USE THIS METHOD, AND THEN USE PATHFINDING TO GET 
        /// TO THE FRONTIER COORDINATE (IN THE SHORTEST AMOUNT OF TIME). 
        /// 
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        bool IsFrontier(int x, int y)
        {
            // getting the map grid from map manager class
            map = mapManager.mapGrid;
            if (map[x, y] != 0) return false;

            if (x > 0 && map[x - 1, y] == -1) return true;
            if (x < gridSize - 1 && map[x + 1, y] == -1) return true;
            if (y > 0 && map[x, y - 1] == -1) return true;
            if (y < gridSize - 1 && map[x, y + 1] == -1) return true;

            // meets none of the cases above so cannot be a frontier
            return false;
        }

        public bool FindNextFrontier(out Vector3 result)
        {
            map = mapManager.mapGrid;
            
            for (int x = 0; x < gridSize; x++)
            {
                for (int y = 0; y < gridSize; y++)
                {
                    if (IsFrontier(x, y))
                    {
                        result = new Vector3(x * cellSize, 0f, y * cellSize);
                        frontierCount++;
                        return true;
                    }
                }
            }
            result = Vector3.zero;
            return false;
        }

        bool HasReachedGoal(Vector3 goal)
        {
            Transform baseLink = transform.GetChild(0).GetChild(2);
            Vector3 flatPos = new Vector3(baseLink.position.x, 0f, baseLink.position.z);
            return Vector3.Distance(flatPos, goal) < goalTolerance;
        }
    }
}
