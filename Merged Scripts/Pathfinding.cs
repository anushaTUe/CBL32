using UnityEngine;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour
{
    public MapManager map;
    public SoundManager soundManager;
    [SerializeField] private LayerMask obstacleMask; // assign in Inspector for raycasting

    void Awake()
    {
        map = GetComponent<MapManager>();
    }

    List<Vector3> SmoothPath(List<Vector3> path)
    {
        if (path == null || path.Count < 2)
            return path;

        List<Vector3> smoothed = new List<Vector3>();
        smoothed.Add(path[0]);

        int currentIndex = 0;

        while (currentIndex < path.Count - 1)
        {
            int nextIndex = currentIndex + 1;

            // Try to jump ahead
            for (int i = path.Count - 1; i > nextIndex; i--)
            {
                if (HasLineOfSight(path[currentIndex], path[i]))
                {
                    nextIndex = i;
                    break;
                }
            }

            smoothed.Add(path[nextIndex]);
            currentIndex = nextIndex;
        }

        return smoothed;
    }

    bool HasLineOfSight(Vector3 start, Vector3 end)
    {
        Vector3 direction = end - start;
        float distance = direction.magnitude;

        // Lift above ground to avoid floor collisions
        start.y += 0.5f;
        end.y += 0.5f;

        return !Physics.Raycast(start, direction.normalized, distance, obstacleMask);
    }
    
    public List<Vector3> FindPathMap(Vector3 startPos, Vector3 targetPos)
    {
        MapNode startNode = map.NodeFromWorldPoint(startPos);
        MapNode targetNode = map.NodeFromWorldPoint(targetPos);

        if (targetNode.state == 1)
        {
            Debug.LogWarning("Target is not walkable!");
            return null;
        }

        List<MapNode> openSet = new List<MapNode>();
        HashSet<MapNode> closedSet = new HashSet<MapNode>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            MapNode currentNode = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].FCost < currentNode.FCost ||
                    openSet[i].FCost == currentNode.FCost && openSet[i].HCost < currentNode.HCost)
                    currentNode = openSet[i];
            }

            openSet.Remove(currentNode);
            closedSet.Add(currentNode);

            if (currentNode == targetNode)
            {
                List<Vector3> rawPath = RetracePathMap(startNode, targetNode);
                return SmoothPath(rawPath);
            }

            foreach (MapNode neighbour in map.GetNeighbours(currentNode))
            {
                if (neighbour.state == 1 || closedSet.Contains(neighbour))
                    continue;

                int newCostToNeighbour = currentNode.GCost + GetDistanceMap(currentNode, neighbour);
                if (newCostToNeighbour < neighbour.GCost || !openSet.Contains(neighbour))
                {
                    neighbour.GCost = newCostToNeighbour;
                    neighbour.HCost = GetDistanceMap(neighbour, targetNode);
                    neighbour.Parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }

        Debug.Log("No path found to endpoint!");
        //Play two sounds
        soundManager.beep(2);
        return null;
    }
    
    List<Vector3> RetracePathMap(MapNode startNode, MapNode endNode)
    {
        List<MapNode> path = new List<MapNode>();
        MapNode currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.Parent;
        }

        path.Reverse();
        map.gridPath = path;

        List<Vector3> waypoints = new List<Vector3>();
        foreach (MapNode node in path)
        {
            waypoints.Add(node.WorldPosition);
        }

        return waypoints;
    }
    
    int GetDistanceMap(MapNode a, MapNode b)
    {
        int dstX = Mathf.Abs(a.GridX - b.GridX);
        int dstY = Mathf.Abs(a.GridY - b.GridY);

        return (dstX > dstY)
            ? 14 * dstY + 10 * (dstX - dstY)
            : 14 * dstX + 10 * (dstY - dstX);
    }
}
