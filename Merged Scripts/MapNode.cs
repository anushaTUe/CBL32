using UnityEngine;

public class MapNode
{
    public int state = -1;
    public Vector3 WorldPosition;
    public int GridX;
    public int GridY;

    public int GCost;
    public int HCost;
    public MapNode Parent;

    public int FCost => GCost + HCost;

    public MapNode(int state, Vector3 worldPos, int x, int y)
    {
        this.state = state;
        WorldPosition = worldPos;
        GridX = x;
        GridY = y;
    }
}