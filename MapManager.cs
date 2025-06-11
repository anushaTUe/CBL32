using UnityEngine;

public class MapManager : MonoBehaviour
{
    public int gridSize = 100;
    public float cellSize = 0.1f;
    public int[,] mapGrid;

    private Vector3 originPosition;

    void Awake()
    {
        mapGrid = new int[gridSize, gridSize];
        originPosition = transform.position;
        InitializeMap();
    }

    void InitializeMap()
    {
        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                mapGrid[x, y] = -1; // unexplored
            }
        }
    }

    public Vector3 GridToWorldPosition(int x, int y)
    {
        float worldX = originPosition.x + (x - gridSize / 2) * cellSize + cellSize / 2;
        float worldZ = originPosition.z + (y - gridSize / 2) * cellSize + cellSize / 2;
        return new Vector3(worldX, originPosition.y, worldZ);
    }

    public Vector2Int WorldToGridPosition(Vector3 position)
    {
        int x = Mathf.FloorToInt((position.x - originPosition.x) / cellSize + gridSize / 2);
        int y = Mathf.FloorToInt((position.z - originPosition.z) / cellSize + gridSize / 2);
        return new Vector2Int(x, y);
    }

    public void UpdateMap(Vector3 position, int value)
    {
        Vector2Int gridPos = WorldToGridPosition(position);
        if (gridPos.x >= 0 && gridPos.x < gridSize && gridPos.y >= 0 && gridPos.y < gridSize)
        {
            mapGrid[gridPos.x, gridPos.y] = value;
        }
    }

    public void SetExplored(Vector3 position)
    {
        UpdateMap(position, 0); // 0 = explored
    }

    public void SetOccupied(Vector3 position)
    {
        UpdateMap(position, 1); // 1 = occupied
    }
}
