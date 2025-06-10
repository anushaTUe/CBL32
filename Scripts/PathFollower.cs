using UnityEngine;
using System.Collections.Generic;

public class PathFollower : MonoBehaviour
{
    public Transform target;
    public float speed = 2f;
    public float rotationSpeed = 5f;

    private Pathfinding pathfinding;
    private List<Vector3> path;
    private int currentIndex = 0;

    void Start()
    {
        pathfinding = FindObjectOfType<Pathfinding>();
        RecalculatePath();
    }

    void Update()
    {
        if (path == null || currentIndex >= path.Count)
            return;

        Vector3 targetPos = new Vector3(path[currentIndex].x, transform.position.y, path[currentIndex].z);
        Vector3 direction = (targetPos - transform.position).normalized;

        // Smoothly rotate toward target
        if (direction != Vector3.zero)
        {
            Quaternion lookRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, rotationSpeed * Time.deltaTime);
        }

        // Move forward
        transform.position = Vector3.MoveTowards(transform.position, targetPos, speed * Time.deltaTime);

        // Proceed to next waypoint if close enough
        if (Vector3.Distance(transform.position, targetPos) < 0.05f)
        {
            currentIndex++;
        }
    }

    public void RecalculatePath()
    {
        path = pathfinding.FindPath(transform.position, target.position);
        currentIndex = 0;

        if (path == null || path.Count == 0)
        {
            Debug.LogError("No path found.");
            enabled = false;
        }
        else
        {
            enabled = true;
        }
    }

    void OnDrawGizmos()
    {
        if (path == null || path.Count < 2)
            return;

        Gizmos.color = Color.blue;
        for (int i = 0; i < path.Count - 1; i++)
        {
            Vector3 start = new Vector3(path[i].x, transform.position.y + 0.1f, path[i].z);
            Vector3 end = new Vector3(path[i + 1].x, transform.position.y + 0.1f, path[i + 1].z);
            Gizmos.DrawLine(start, end);
        }
    }

}
