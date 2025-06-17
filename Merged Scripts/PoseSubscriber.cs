namespace RosSharp.Control;

public class PoseSubscriber
{
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class PoseSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>("/odom", CorrectPosistion);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void CorrectPosistion(OdometryMsg msg)
    {
        Vector3 pos = new Vector3(0, this.transform.position.y, 0);
        pos.x = (float) msg.pose.pose.position.x;
        pos.z = (float) msg.pose.pose.position.y;
        this.transform.position = pos;
    }
}

}