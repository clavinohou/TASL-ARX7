using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.XR;

// --------------------------------------------------------------------------
// Controller state structures (packed, little-endian)
// --------------------------------------------------------------------------
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct ControllerState
{
    public Vector3 pos;        // 12 bytes
    public Quaternion rot;     // 16 bytes
    public Vector2 joy;        //  8 bytes
    public float trigger;      //  4 bytes
    public float grip;         //  4 bytes
    public byte buttons;       //  1 byte (bitflags)
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct FramePacket
{
    public double          time;   //  8 bytes
    public ControllerState left;   // 45 bytes
    public ControllerState right;  // 45 bytes
    // TOTAL = 98 bytes
}

public class SendControllerPose : MonoBehaviour
{
    [Header("UDP Settings")]
    public string remoteIp   = "127.0.0.1";
    public int    remotePort = 7000;

    // Pre-allocated buffer and socket
    private static readonly int    PacketSize = Marshal.SizeOf<FramePacket>();
    private static readonly byte[] buffer     = new byte[PacketSize];
    private Socket                sock;
    private EndPoint              remoteEndPoint;

    // XR devices
    private InputDevice leftDevice, rightDevice;

    // Cache of the most recent controller sample
    private FramePacket latestPacket;

    // ------------------------------------------------------------------
    // Life-cycle
    // ------------------------------------------------------------------
    void Awake()
    {
        QualitySettings.vSyncCount  = 0;
        Application.targetFrameRate = -1;

        // Send loop at 200 Hz
        Time.fixedDeltaTime = 0.005f;

        // Initialise UDP socket
        sock = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIp), remotePort);
    }

    void Start()
    {
        FindDevices();
        InputDevices.deviceConnected    += _ => FindDevices();
        InputDevices.deviceDisconnected += _ => FindDevices();
    }

    void OnDestroy()
    {
        sock?.Close();
    }

    // ------------------------------------------------------------------
    // Device discovery
    // ------------------------------------------------------------------
    void FindDevices()
    {
        leftDevice  = GetDevice(InputDeviceCharacteristics.Left  |
                                InputDeviceCharacteristics.Controller |
                                InputDeviceCharacteristics.HeldInHand);
        rightDevice = GetDevice(InputDeviceCharacteristics.Right |
                                InputDeviceCharacteristics.Controller |
                                InputDeviceCharacteristics.HeldInHand);
    }

    InputDevice GetDevice(InputDeviceCharacteristics traits)
    {
        var devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(traits, devices);
        return devices.Count > 0 ? devices[0] : default;
    }

    // ------------------------------------------------------------------
    // Sample every Update() (render cadence)
    // ------------------------------------------------------------------
    void Update()
    {
        latestPacket.time = Time.timeAsDouble;
        FillState(leftDevice,  ref latestPacket.left);
        FillState(rightDevice, ref latestPacket.right);
    }

    // ------------------------------------------------------------------
    // Send every FixedUpdate() (physics cadence ~200 Hz)
    // ------------------------------------------------------------------
    void FixedUpdate()
    {
        MemoryMarshal.Write(buffer, ref latestPacket);
        sock.SendTo(buffer, remoteEndPoint);
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    void FillState(InputDevice device, ref ControllerState state)
    {
        if (!device.isValid)
        {
            state.pos     = Vector3.zero;
            state.rot     = Quaternion.identity;
            state.joy     = Vector2.zero;
            state.trigger = 0f;
            state.grip    = 0f;
            state.buttons = 0;
            return;
        }

        device.TryGetFeatureValue(CommonUsages.devicePosition, out state.pos);
        device.TryGetFeatureValue(CommonUsages.deviceRotation, out state.rot);
        device.TryGetFeatureValue(CommonUsages.primary2DAxis,  out state.joy);
        device.TryGetFeatureValue(CommonUsages.trigger,       out state.trigger);
        device.TryGetFeatureValue(CommonUsages.grip,          out state.grip);

        state.buttons = 0;
        AddButton(device, CommonUsages.primaryButton,   0, ref state.buttons);
        AddButton(device, CommonUsages.secondaryButton, 1, ref state.buttons);
        AddButton(device, CommonUsages.triggerButton,   2, ref state.buttons);
        AddButton(device, CommonUsages.gripButton,      3, ref state.buttons);
        AddButton(device, CommonUsages.menuButton,      4, ref state.buttons);
    }

    void AddButton(InputDevice device, InputFeatureUsage<bool> feature, int bit, ref byte field)
    {
        if (device.TryGetFeatureValue(feature, out bool pressed) && pressed)
            field |= (byte)(1 << bit);
    }
}
