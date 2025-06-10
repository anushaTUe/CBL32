using UnityEngine;

public class PathNode
{
    public bool Walkable;
    public Vector3 WorldPosition;
    public int GridX;
    public int GridY;

    public int GCost;
    public int HCost;
    public PathNode Parent;

    public int FCost => GCost + HCost;

    public PathNode(bool walkable, Vector3 worldPos, int x, int y)
    {
        Walkable = walkable;
        WorldPosition = worldPos;
        GridX = x;
        GridY = y;
    }
}
