using UnityEngine;
using System.Collections.Generic;
using RosSharp.Control;

public class MapManager : MonoBehaviour
{
    public Vector2 gridWorldSize;
    public float cellSize;
    MapNode[,] grid;
    
    int gridSizeX, gridSizeY;
    private LaserScanScript scanner;
    public Transform robotTransform;
    public bool updated = false;

    void Awake()
    {
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / cellSize);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / cellSize);
        CreateGrid();
    }

    //This method creates an empty grid
    void CreateGrid()
    {
        grid = new MapNode[gridSizeX, gridSizeY];
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * cellSize + cellSize/2)
                                                    + Vector3.forward * (y * cellSize + cellSize/2);
                grid[x, y] = new MapNode(-1, worldPoint, x, y);
            }
        }
    }
    
    public List<MapNode> GetNeighbours(MapNode node)
    {
        List<MapNode> neighbours = new List<MapNode>();

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if ((x == 0 && y == 0) || Mathf.Abs(x) + Mathf.Abs(y) == 2)
                    continue;

                int checkX = node.GridX + x;
                int checkY = node.GridY + y;

                if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
                    neighbours.Add(grid[checkX, checkY]);
            }
        }

        return neighbours;
    }

    //Updates the map based on the LIDAR input
    public void UpdateMap(Vector3[] hitPoints, float[] angles, float[] ranges)
    {
        Vector3 robotPosition = robotTransform.position;
        Quaternion robotRotation = Quaternion.Euler(0, robotTransform.eulerAngles.y, 0);
        MapNode robotNode = NodeFromWorldPoint(robotPosition);
        robotNode.state = 0;

        for (int i = 0; i < hitPoints.Length; i++)
        {
            Vector3 laserHitPoint = hitPoints[i];
            MapNode hitNode = NodeFromWorldPoint(laserHitPoint);
            
            Vector3 currPos = robotTransform.position;
            float distance = cellSize;
            while (distance < ranges[i])
            {
                Vector3 direction = robotRotation * new Vector3(Mathf.Sin(angles[i]), 0, Mathf.Cos(angles[i]));
                Vector3 point = robotPosition + direction * distance;

                MapNode currNode = NodeFromWorldPoint(point);
                if (currNode.state == -1)
                {
                    currNode.state = 0;
                    updated = true;
                }

                distance += cellSize;
            }

            if (hitNode.state != 1)
            {
                hitNode.state = 1;
                updated = true;
            }
        }
    }

    //Converts a vector3 to a node
    public MapNode NodeFromWorldPoint(Vector3 worldPosition)
    {
        float percentX = Mathf.Clamp01((worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x);
        float percentY = Mathf.Clamp01((worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y);

        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }
    public List<MapNode> gridPath;
    
    //Draws the map in the scene
    void OnDrawGizmos()
    {
        Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

        if (grid == null)
            return;

        foreach (MapNode node in grid)
        {
            Gizmos.color = node.state == 0 ? Color.white : Color.yellow;

            if (gridPath != null && gridPath.Contains(node))
            {
                Gizmos.color = Color.green;
            } 
            
            if (node.state == 1)
            {
                Gizmos.color = Color.red;
            }

            Gizmos.DrawCube(node.WorldPosition, Vector3.one * (cellSize));
        }
    }
    
    //Method to get the next frontier in the map
    public Vector3 NextFrontierPoint()
    {
        foreach (MapNode node in grid)
        {
            if (IsFrontier(node))
            {
                return node.WorldPosition;
            }
        }

        return new Vector3(0, -100, 0);
    }

    //Method to check if a node is a frontier
    private bool IsFrontier(MapNode node)
    {
        if (node.state == 0 || node.state == 1)
        {
            return false;
        }
        
        foreach (MapNode neighbor in GetNeighbours(node))
        {
            if (neighbor.state == 0)
            {
                return true;
            }
        }

        return false;
    }
    
    //Method to check if the scanning has started
    public bool ScannerStarted()
    {
        foreach (MapNode node in grid)
        {
            if (node.state == 1 || node.state == 0)
            {
                return true;
            }
        }

        return false;
    }
    
    //Method to get a random endpoint in the map
    public Vector3 RandomPoint()
    {
        List<MapNode> nodes = new List<MapNode>();
        foreach (MapNode node in grid)
        {
            if (node.state == 0)
            {
                nodes.Add(node);
            }
        }

        MapNode target = nodes[Random.Range(0, nodes.Count)];
        return target.WorldPosition;
    }
    
    //Method to get a point out of bounds
    public Vector3 GetOutOfBounds()
    {
        List<MapNode> nodes = new List<MapNode>();
        foreach (MapNode node in grid)
        {
            if (node.state == -1)
            {
                nodes.Add(node);
            }
        }

        MapNode target = nodes[Random.Range(0, nodes.Count)];
        return target.WorldPosition;
    }
}
