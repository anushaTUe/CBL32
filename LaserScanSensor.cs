using System.Collections.Generic;
using UnityEngine;

public class LaserScanSensor : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 5f;
    public float angleStart = 0f;
    public float angleEnd = 360f;
    public LayerMask obstacleLayer;
    public bool showDebug = true;

    private MapManager mapManager;

    void Start()
    {
        mapManager = FindObjectOfType<MapManager>();
        if (mapManager == null)
        {
            Debug.LogError("MapManager not found in the scene.");
        }
    }

    void Update()
    {
        PerformScan();
    }

    void PerformScan()
    {
        float angleIncrement = (angleEnd - angleStart) / numRays;
        Vector3 origin = transform.position;

        for (int i = 0; i < numRays; i++)
        {
            float angle = Mathf.Repeat(angleStart + i * angleIncrement, 360f);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            if (Physics.Raycast(origin, direction, out RaycastHit hit, maxRange, obstacleLayer))
            {
                if (showDebug)
                    Debug.DrawLine(origin, hit.point, Color.red);

                mapManager?.UpdateMap(origin, hit.point, true);
            }
            else
            {
                Vector3 missPoint = origin + direction * maxRange;
                if (showDebug)
                    Debug.DrawLine(origin, missPoint, Color.green);

                mapManager?.UpdateMap(origin, missPoint, false);
            }
        }
    }
}