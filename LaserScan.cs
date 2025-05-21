using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.UrdfImporter.Control;
using Codice.Client.BaseCommands.WkStatus.Printers;

namespace RosSharp.Control
{
    public class LaserScan : MonoBehaviour
    {
        private float[] ranges;
        private float angleStart;
        private float angleEnd;
        private float angleIncrement;
        
        private ROSConnection ros;

        public GameObject obstacle;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<LaserScanMsg>("/scan", Scan);
        }

        void Scan(LaserScanMsg msg)
        {
            this.ranges = msg.ranges;
            this.angleStart = msg.angle_min;
            this.angleEnd = msg.angle_max;
            this.angleIncrement = msg.angle_min;

            Vector3[] points = new Vector3[this.ranges.Length];
            Vector3 pos = transform.position;

            for (int i = 0; i < this.ranges.Length; i++)
            {
                float angle = angleStart + angleIncrement * i;
                Vector3 point = new Vector3(pos.x + ranges[i] * Mathf.Sin(angle), pos.y, pos.z + ranges[i] * Mathf.Cos(angle));
                Instantiate(obstacle, point, Quaternion.Euler(0, angle, 0));
                points[i] = point;
                
                //GameObject myGameObject = Instantiate(obstacle, point, Quaternion.Euler(0, angle, 0));
                //Destroy(myGameObject, 2f);
            }
        }
    }
}