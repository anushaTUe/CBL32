using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections;
using RosMessageTypes.Turtlebot3;


public class SoundManager : MonoBehaviour
{
    ROSConnection ros;

    public string soundTopicName = "/sound";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<SoundMsg>(soundTopicName);

        SendBeep(1);
    }

    void SendBeep(byte value)
    {
        SoundMsg beepMsg = new SoundMsg(value); // value = 1 for beep
        ros.Publish(soundTopicName, beepMsg);
        Debug.Log("Sent beep command with value: " + value);
    }

    IEnumerator BeepMultipleTimes(int count, float delay)
    {
        for (int i = 0; i < count; i++)
        {
            SendBeep(1);
            yield return new WaitForSeconds(delay);
        }
    }

    public void beep(int times)
    {
        if (times == 1)
        {
            SendBeep(1);
        } else if (times == 2)
        {
            StartCoroutine(BeepMultipleTimes(2, 0.4f));
        } else if (times == 3)
        {
            StartCoroutine(BeepMultipleTimes(3, 0.4f));
        }

    }
}
