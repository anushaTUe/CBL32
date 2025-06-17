using System.Collections;
using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.UI;
using UnityEditor;

public class CentralController : MonoBehaviour
{
    public AGVController moveController;
    public MapManager mapManager;
    public Transform robotTransform;
    public Pathfinding pathFinder;
    public LaserScanScript scanner;

    private bool scanningStarted = false;

    private bool frontiersFound = false;
    private bool done = false;

    private bool endpointSet = false;
    Vector3 endPoint = Vector3.zero;
    private int currTarget = 0;
    private List<Vector3> currentPath;
    private Vector3 currFrontier;
    public SoundManager soundManager;
    public Button button;
    private bool finalActionDone = false;
    private bool pathFoundToFrontier = true;
    private bool pathFoundToEndPoint = true;
    private bool arrivedAtEndPoint = false;
    private bool scanningDone = false;


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
        else if (!frontiersFound)
        {
            //Finding and moving to frontiers
            Vector3 frontier = mapManager.NextFrontierPoint();
            if (frontier.y == -100)
            {
                Debug.Log("No frontiers left");
                frontiersFound = true;
                handleSound();
            }
            else if (mapManager.updated)
            {
                List<Vector3> path = pathFinder.FindPathMap(robotTransform.position, frontier);
                currentPath = path;
                currTarget = 0;
                bool reached = followPath(path);
                if (reached)
                {
                    moveController.Stop();
                }
                mapManager.updated = false;
            }
            else
            {
                bool reached = followPath(currentPath);
                if (reached)
                {
                    moveController.Stop();
                }
            }
        }
        else if (!done)
        {
            //Making a random endpoint and then going towards it
            if (endpointSet == false)
            {
                scanner.shouldScan = false;
                Debug.Log("Going to endpoint");
                endPoint = mapManager.RandomPoint();
                currTarget = 0;
                List<Vector3> path = pathFinder.FindPathMap(robotTransform.position, endPoint);
                currentPath = path;

                endpointSet = true;
            }
            bool reached = followPath(currentPath);

            if (reached)
            {
                moveController.Stop();
                done = true;
                handleSound();
            }


        }

        if (done && !finalActionDone && pathFoundToEndPoint)
        {
            //Done 
            moveController.Stop();
            Debug.Log("Turning on the button");
            button.gameObject.SetActive(true);
            finalActionDone = true;
        }
    }

    private bool followPath(List<Vector3> path)
    {
        pathFoundToEndPoint = true;
        while (currTarget < path.Count && Vector3.Distance(robotTransform.position, path[currTarget]) < 0.2f)
        {
            currTarget++;
        }

        if (path.Count == 0)
        {
            Debug.Log("No path found!");
            pathFoundToFrontier = false;
            pathFoundToEndPoint = false;
            return true;
        }
        else if (currTarget < path.Count)
        {
            //Debug.Log(path[currTarget]);
            moveController.Move(path[currTarget]);
        }
        else
        {
            Debug.Log("Path end");
            arrivedAtEndPoint = true;
            return true;
        }

        return false;
        //Debug.Log(path[currTarget]);
    }

    void OnDrawGizmos()
    {
        if (currentPath == null || currentPath.Count < 2)
            return;

        Gizmos.color = Color.blue;

        for (int i = 0; i < currentPath.Count - 1; i++)
        {
            Gizmos.DrawLine(currentPath[i], currentPath[i + 1]);
        }

        // Optionally draw small spheres at each waypoint
        foreach (Vector3 point in currentPath)
        {
            Gizmos.DrawSphere(point, 0.05f);
        }
    }

    public void onClickRestart()
    {
        done = false;
        endpointSet = false;
        finalActionDone = false;
        button.gameObject.SetActive(false);
    }


    void handleSound()
    {
        if (frontiersFound && !scanningDone)
        {
            scanningDone = true;
            soundManager.beep(3);
        }
        else if (!pathFoundToEndPoint && done && scanningDone)
        {
            soundManager.beep(2);
        }
        else if (arrivedAtEndPoint && scanningDone)
        {
            soundManager.beep(1);
        }

    }

}
