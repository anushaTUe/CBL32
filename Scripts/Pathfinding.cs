using UnityEngine;
using System.Collections.Generic;

public class Pathfinding : MonoBehaviour
{
    GridManager grid;

    [SerializeField] private LayerMask obstacleMask; // assign in Inspector for raycasting

    void Awake()
    {
        grid = GetComponent<GridManager>();
    }

    public List<Vector3> FindPath(Vector3 startPos, Vector3 targetPos)
    {
        PathNode startNode = grid.NodeFromWorldPoint(startPos);
        PathNode targetNode = grid.NodeFromWorldPoint(targetPos);

        if (!targetNode.Walkable)
        {
            Debug.LogWarning("Target is not walkable!");
            return null;
        }

        List<PathNode> openSet = new List<PathNode>();
        HashSet<PathNode> closedSet = new HashSet<PathNode>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            PathNode currentNode = openSet[0];
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
                List<Vector3> rawPath = RetracePath(startNode, targetNode);
                return SmoothPath(rawPath);
            }

            foreach (PathNode neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.Walkable || closedSet.Contains(neighbour))
                    continue;

                int newCostToNeighbour = currentNode.GCost + GetDistance(currentNode, neighbour);
                if (newCostToNeighbour < neighbour.GCost || !openSet.Contains(neighbour))
                {
                    neighbour.GCost = newCostToNeighbour;
                    neighbour.HCost = GetDistance(neighbour, targetNode);
                    neighbour.Parent = currentNode;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }

        return null;
    }

    List<Vector3> RetracePath(PathNode startNode, PathNode endNode)
    {
        List<PathNode> path = new List<PathNode>();
        PathNode currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.Parent;
        }

        path.Reverse();
        grid.gridPath = path;

        List<Vector3> waypoints = new List<Vector3>();
        foreach (PathNode node in path)
        {
            waypoints.Add(node.WorldPosition);
        }

        return waypoints;
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

    int GetDistance(PathNode a, PathNode b)
    {
        int dstX = Mathf.Abs(a.GridX - b.GridX);
        int dstY = Mathf.Abs(a.GridY - b.GridY);

        return (dstX > dstY)
            ? 14 * dstY + 10 * (dstX - dstY)
            : 14 * dstX + 10 * (dstY - dstX);
    }
}
