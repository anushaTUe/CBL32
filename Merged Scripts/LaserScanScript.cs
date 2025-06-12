using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

namespace RosSharp.Control
{
    public class LaserScanScript : MonoBehaviour
    {
        public Transform robotBase; //The base_link transfrom
        public GameObject obstaclePrefab; //Make a simple object prefab make sure it's small tho, attach that to this
        //I made a cube who's scale was x:0.1, y:1.5 ,z:0.1

        private float[] ranges;
        private float angleStart;
        private float angleEnd;
        private float angleIncrement;
        private float rangeMin;
        private float rangeMax;
        private Vector3[] points;
        private float[] angles;

        private ROSConnection ros;
        public MapManager mapManager; //should be the only map managager in the project
        public GameObject objectParent; //An empty object under whcih all the obstacle will be placed
        public float gapSize = 0.01f; //leave as is

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<LaserScanMsg>("/scan", Scan);

            if (robotBase == null)
                robotBase = transform;
        }

        void Scan(LaserScanMsg msg)
        {
            ranges = msg.ranges;
            angleStart = msg.angle_min;
            angleEnd = msg.angle_max;
            angleIncrement = msg.angle_increment;
            rangeMin = msg.range_min;
            rangeMax = msg.range_max;

            Vector3 pos = transform.GetChild(0).GetChild(2).position;
            Quaternion robotRotation = Quaternion.Euler(0, robotBase.eulerAngles.y, 0);
            points = new Vector3[ranges.Length];
            angles = new float[ranges.Length];

            for (int i = 0; i < ranges.Length; i++)
            {
                float range = ranges[i];

                float angle = angleStart + angleIncrement * i;
                //For actual robot                                                                                              !!!!!
                //angle = (angle - (Mathf.PI / 2));
                //For simulation
                angle = -angle;

                angles[i] = angle;

                // Flip angle to match Unity left-handed system if mirrored
                //angleRad = -angleRad;
                
                Vector3 point = new Vector3(pos.x + range * Mathf.Sin(angle), pos.y, pos.z + range * Mathf.Cos(angle));
                
                Vector3 direction = robotRotation * new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle));
                Vector3 hitPoint = pos + direction * range;
   
                //Debug.DrawLine(pos, hitPoint, Color.red, Time.deltaTime);
                
                //For simulation purpose
                point = hitPoint;
                
                points[i] = point;
                
                if (range >= rangeMin && range <= rangeMax && !hit(point) )
                {
                    GameObject obstacle = Instantiate(obstaclePrefab, point, Quaternion.identity);
                    obstacle.transform.parent = objectParent.transform;
                    obstacle.layer = LayerMask.NameToLayer("Ignore Raycast");
                }
                
                
            }
            mapManager.UpdateMap(points, angles, ranges);
        }

        private bool hit(Vector3 point)
        {
            Collider[] hits = Physics.OverlapSphere(point, gapSize);
            int numHits = 0;
            foreach (Collider hit in hits)
            {
                if (hit.tag != "Wall")
                {
                    numHits++;
                }
            }
            return hits.Length > 0;
        }
    }
}

/*
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
           private float angleIncrement;
           private float rangeMin;
           private float rangeMax;
   
           private ROSConnection ros;
   
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
               angleIncrement = msg.angle_increment;
               rangeMin = msg.range_min;
               rangeMax = msg.range_max;
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
       }
   }
*/