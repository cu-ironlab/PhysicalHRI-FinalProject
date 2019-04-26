using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Net.Sockets;
using System.Net;
using System;

#if !UNITY_EDITOR
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
#endif

public class ViconTracking : MonoBehaviour
{

    int port = 51002;
    int packetByteSize = 256;

#if !UNITY_EDITOR
    bool isFirstUpdate = true;
    bool hasHololensLocation = false;

    static string holoLensIP = "192.168.10.127";
    DatagramSocket socket = new DatagramSocket();
    HostName host = new HostName(holoLensIP);
#endif

    int xOffset_Object1 = 32,
        yOffset_Object1 = 40,
        zOffset_Object1 = 48,
        rOffset_Object1 = 56,
        pOffset_Object1 = 64,
        qOffset_Object1 = 72;
    int xOffset_Object2 = 107,
        yOffset_Object2 = 115,
        zOffset_Object2 = 123;

    Vector3 translation_Object1;
    Vector3 rotation_Object1;
    Vector3 translation_Object2;
    Vector3 rotation_Object2;


    public Vector3 adjustVec = new Vector3(0,-1f,0);
    public bool adjustVecHasBeenSet = false;

    const int INT_SIZE = 8;
    int scale = 1000;
    Byte[] receiveBytes;

    // Use this for initialization
    void Start()
    {
#if !UNITY_EDITOR
        socket.MessageReceived += socket_MessageReceived;
        System.Diagnostics.Debug.WriteLine("Attempting to Connect..");
        // await socket.BindEndpointAsync(host, port.ToString());
        socket.BindEndpointAsync(host, port.ToString());
        System.Diagnostics.Debug.WriteLine("Listening...");
#endif
    }



    // Update is called once per frame
    void Update()
    {

#if !UNITY_EDITOR
        if (isFirstUpdate && hasHololensLocation)
        {
            adjustVec -= translation_Object2;
            adjustVecHasBeenSet = true;
            isFirstUpdate = false;
        }

        this.transform.position = adjustVec + translation_Object1;
        rotation_Object1.x = 0;
        rotation_Object1.z = 0;
        this.transform.rotation = Quaternion.Euler(rotation_Object1);
#endif
    }

#if !UNITY_EDITOR
    void socket_MessageReceived(DatagramSocket sender, DatagramSocketMessageReceivedEventArgs args)
    {
        receiveBytes = args.GetDataReader().ReadBuffer(256).ToArray();

        System.Buffer.BlockCopy(receiveBytes, xOffset_Object1, receiveBytes, 0, INT_SIZE);
        translation_Object1.x = -(float)BitConverter.ToDouble(receiveBytes, 0) / scale;
        System.Buffer.BlockCopy(receiveBytes, yOffset_Object1, receiveBytes, 0, INT_SIZE);
        translation_Object1.z = -(float)BitConverter.ToDouble(receiveBytes, 0) / scale;
        System.Buffer.BlockCopy(receiveBytes, zOffset_Object1, receiveBytes, 0, INT_SIZE);
        translation_Object1.y = (float)BitConverter.ToDouble(receiveBytes, 0) / scale;

        System.Buffer.BlockCopy(receiveBytes, rOffset_Object1, receiveBytes, 0, INT_SIZE);
        rotation_Object1.x = (float)BitConverter.ToDouble(receiveBytes, 0) * Mathf.Rad2Deg;
        System.Buffer.BlockCopy(receiveBytes, pOffset_Object1, receiveBytes, 0, INT_SIZE);
        rotation_Object1.z = (float)BitConverter.ToDouble(receiveBytes, 0) * Mathf.Rad2Deg;
        System.Buffer.BlockCopy(receiveBytes, qOffset_Object1, receiveBytes, 0, INT_SIZE);
        rotation_Object1.y = -(float)BitConverter.ToDouble(receiveBytes, 0) * Mathf.Rad2Deg;

        if (!hasHololensLocation)
        {
            System.Buffer.BlockCopy(receiveBytes, xOffset_Object2, receiveBytes, 0, INT_SIZE);
            translation_Object2.x = -(float)BitConverter.ToDouble(receiveBytes, 0) / scale;
            System.Buffer.BlockCopy(receiveBytes, yOffset_Object2, receiveBytes, 0, INT_SIZE);
            translation_Object2.z = -(float)BitConverter.ToDouble(receiveBytes, 0) / scale;
            System.Buffer.BlockCopy(receiveBytes, zOffset_Object2, receiveBytes, 0, INT_SIZE);
            translation_Object2.y = (float)BitConverter.ToDouble(receiveBytes, 0) / scale;

            hasHololensLocation = true;
        }
    }
#endif

}
