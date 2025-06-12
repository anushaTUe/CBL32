using System.Collections;
using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;

public class CentralController : MonoBehaviour
{
    public AGVController moveController; //The AGVController thats under the turtlebot
    public MapManager mapManager; //The only map manager in the scene
    public Transform robotTransform; //The base_link transform
    public Pathfinding pathFinder; //The only Pathfinding in the scene

    private bool scanningStarted = false;

    private bool frontiersFound = false;
    private bool done = false;

    private bool endpointSet = false;
    Vector3 endPoint = Vector3.zero;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (!scanningStarted)
        {
            //Waiting until we are recieving LIDAR messages before moving
            scanningStarted = mapManager.ScannerStarted();
        }
        else if(!frontiersFound)
        {
            //Finding and moving to frontiers
            Vector3 frontier = mapManager.NextFrontierPoint();
            if (frontier.y == -100)
            {
                Debug.Log("No frontiers left");
                frontiersFound = true;
            }
            else
            {
                List<Vector3> path = pathFinder.FindPathMap(robotTransform.position, frontier);
                bool reached = followPath(path);
                if (reached)
                {
                    moveController.Stop();
                }
            }
        }
        else if (!done)
        {
            //Making a random endpoint and then going towards it
            Debug.Log("Going to endpoint");
            if (endpointSet == false)
            {
                endPoint = mapManager.RandomPoint();
                endpointSet = true;
            }

            List<Vector3> path = pathFinder.FindPathMap(robotTransform.position, endPoint);
            bool reached = followPath(path);
            if (reached)
            {
                moveController.Stop();
                done = true;
            }
        }

        if (done)
        {
            //Done 
            moveController.Stop();
        }
    }

    private bool followPath(List<Vector3> path)
    {
        int currTarget = 0;
        while (currTarget < path.Count && Vector3.Distance(robotTransform.position, path[currTarget]) < 0.2f)
        {
            currTarget++;
        }

        if (currTarget < path.Count)
        {
            Debug.Log(path[currTarget]);
            moveController.Move(path[currTarget]);
        }
        else
        {  
            Debug.Log("Path end");
            return true;
        }
        
        return false;
        //Debug.Log(path[currTarget]);
    }

}
