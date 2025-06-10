using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

namespace RosSharp.Control
{
    public class LaserScan : MonoBehaviour
    {
        public Transform robotBase;
        public GameObject obstaclePrefab;

        private float[] ranges;
        private float angleStart;
        private float angleEnd;
        private float angleIncrement;
        private float rangeMin;
        private float rangeMax;

        private ROSConnection ros;
        public MapManager mapManager;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<LaserScanMsg>("/scan", OnScanReceived);

            if (robotBase == null)
                robotBase = transform;
        }

        void OnScanReceived(LaserScanMsg msg)
        {
            ranges = msg.ranges;
            angleStart = msg.angle_min;
            angleEnd = msg.angle_max;
            angleIncrement = msg.angle_increment;
            rangeMin = msg.range_min;
            rangeMax = msg.range_max;
            mapManager.UpdateMap();
        }

        void Update()
        {
            if (ranges == null) return;

            Vector3 pos = robotBase.position;
            Quaternion robotRotation = Quaternion.Euler(0, robotBase.eulerAngles.y, 0);

            for (int i = 0; i < ranges.Length; i++)
            {
                float range = ranges[i];
                if (range < rangeMin || range > rangeMax) continue;

                float angleRad = angleStart + angleIncrement * i;

                // Flip angle to match Unity left-handed system if mirrored
                angleRad = -angleRad;

                Vector3 direction = robotRotation * new Vector3(Mathf.Sin(angleRad), 0, Mathf.Cos(angleRad));
                Vector3 hitPoint = pos + direction * range;

                Debug.DrawLine(pos, hitPoint, Color.red, Time.deltaTime);
            }
        }

        public float[] getRanges()
        {
            return ranges;
        }
        
        public float getAngleStart(){return angleStart;}
        public float getAngleEnd(){return angleEnd;}
        public float getAngleIncrement(){return angleIncrement;}
        public int getNumPoints(){return ranges.Length;}
    }
}