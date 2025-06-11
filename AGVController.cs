using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

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
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        private MapManager mapManager;
        private int[,] map;
        private int gridSize;
        private float cellSize;

        private Vector3 currentGoal;
        private bool navigatingToGoal = false;
        private float goalTolerance = 0.05f;

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
            map = mapManager.mapGrid;
            gridSize = mapManager.gridSize;
            cellSize = mapManager.cellSize;

            InvokeRepeating("TrySetNextFrontierGoal", 2f, 1f); // call periodically
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0;
                rosAngular = 0;
            }
            RobotInput(rosLinear, rosAngular);

            if (navigatingToGoal && HasReachedGoal(currentGoal))
            {
                navigatingToGoal = false;
            }
        }

        void TrySetNextFrontierGoal()
        {
            if (!navigatingToGoal && FindNextFrontier(out Vector3 nextGoal))
            {
                currentGoal = nextGoal;
                navigatingToGoal = true;
                Move(currentGoal);
            }
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.targetVelocity = wheelSpeed;
            joint.xDrive = drive;
        }

        private void RobotInput(float speed, float rotSpeed)
        {
            speed = Mathf.Clamp(speed, -maxLinearSpeed, maxLinearSpeed);
            rotSpeed = Mathf.Clamp(rotSpeed, -maxRotationalSpeed, maxRotationalSpeed);

            float wheel1Rotation = (speed / wheelRadius);
            float wheel2Rotation = wheel1Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);

            wheel1Rotation = (wheel1Rotation + wheelSpeedDiff) * Mathf.Rad2Deg;
            wheel2Rotation = (wheel2Rotation - wheelSpeedDiff) * Mathf.Rad2Deg;

            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
        }

        public void Move(Vector3 point)
        {
            Transform baseLink = transform.GetChild(0).GetChild(2);
            Vector3 direction = (point - baseLink.position).normalized;
            float angleToGoal = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg;

            float currentAngle = baseLink.eulerAngles.y;
            float angleDiff = Mathf.DeltaAngle(currentAngle, angleToGoal);

            TwistMsg twist = new TwistMsg();
            if (Mathf.Abs(angleDiff) < 10f)
                twist.linear.x = maxLinearSpeed;
            else
                twist.angular.z = Mathf.Sign(angleDiff) * maxRotationalSpeed;

            ros.Publish("cmd_vel", twist);
        }

        public bool FindNextFrontier(out Vector3 result)
        {
            for (int x = 1; x < gridSize - 1; x++)
            {
                for (int y = 1; y < gridSize - 1; y++)
                {
                    if (IsFrontier(x, y))
                    {
                        Vector3 worldPos = mapManager.GridToWorldPosition(x, y);
                        result = new Vector3(worldPos.x, 0f, worldPos.z);
                        return true;
                    }
                }
            }
            result = Vector3.zero;
            return false;
        }

        bool IsFrontier(int x, int y)
        {
            if (map[x, y] != 0) return false;
            return map[x + 1, y] == -1 || map[x - 1, y] == -1 || map[x, y + 1] == -1 || map[x, y - 1] == -1;
        }

        bool HasReachedGoal(Vector3 goal)
        {
            Vector3 pos = transform.position;
            pos.y = 0;
            goal.y = 0;
            return Vector3.Distance(pos, goal) < goalTolerance;
        }
    }
}