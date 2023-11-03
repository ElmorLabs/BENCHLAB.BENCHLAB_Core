using Microsoft.Win32;
using System;
using System.Drawing;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Text;
using static BENCHLAB_Core.Device_BENCHLAB;

namespace BENCHLAB_Core;

public class Device_BENCHLAB
{

    #region Device-specific Consts

    public const int VENDOR_ID = 0xEE;
    public const int PRODUCT_ID = 0x10;
    public const int FIRMWARE_VERSION = 0x01;

    public const int FAN_PROFILE_NUM = 3;

    public const int FAN_NUM = 9;
    public const int FAN_CURVE_NUM_POINTS = 2;

    public const int SENSOR_TS_NUM = 4;

    public const int SENSOR_VIN_NUM = 13;
    public const int SENSOR_POWER_NUM = 11;

    public const int RGB_PROFILE_NUM = 2;

    readonly string[] PowerSensorNames = { "EPS1", "EPS2", "ATX3V", "ATX5V", "ATX5VSB", "ATX12V", "PCIE1", "PCIE2", "PCIE3", "HPWR1", "HPWR2" };


    #endregion

    #region Device-specific Structs

    public struct VendorDataStruct
    {
        public byte VendorId;
        public byte ProductId;
        public byte FwVersion;
    }

    //[StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct PowerSensor
    {
        public Int32 Voltage;
        public Int32 Current;
        public Int32 Power;
    };

    public struct FanSensor
    {
        public byte Enable;
        public byte Duty;
        public ushort Tach;
    };

    //[StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct SensorStruct
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_VIN_NUM)] public short[] Vin;
        public ushort Vdd;
        public ushort Vref;
        public short Tchip;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_TS_NUM)] public short[] Ts;
        public short Tamb;
        public ushort Hum;
        public FAN_SWITCH_STATUS FanSwitch;
        public RGB_SWITCH_STATUS RgbSwitch;
        public RGB_EXT_STATUS RgbExtStatus;
        public byte FanExtDuty;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_POWER_NUM)] public PowerSensor[] PowerReadings;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FAN_NUM)] public FanSensor[] Fans;
    }

    public struct FanConfigStruct
    {
        public FAN_MODE FanMode;
        public TEMP_SRC TempSource;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FAN_CURVE_NUM_POINTS)] public short[] Temp;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FAN_CURVE_NUM_POINTS)] public byte[] Duty;
        public byte RampStep;
        public byte FixedDuty;
        public byte MinDuty;
        public byte MaxDuty;
    }

    /*public struct ProfileConfigStruct
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FAN_NUM)] public FanConfigStruct[] FanConfig;
    }*/

    public struct RGBConfigStruct
    {
        public RGB_MODE Mode;
        public byte Red;
        public byte Green;
        public byte Blue;
        public RGB_DIRECTION Direction;
        public byte Speed;
    };

    public struct CalibrationValueStruct
    {
        public Int16 Offset;
        public Int16 GainOffset;
    };

    public struct CalibrationStruct {

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_VIN_NUM)] public CalibrationValueStruct[] Vin;
        public CalibrationValueStruct Vdd;
        public CalibrationValueStruct Vref;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_TS_NUM)] public CalibrationValueStruct[] Ts;
        public CalibrationValueStruct Tamb;
        public CalibrationValueStruct Hum;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_POWER_NUM)] public CalibrationValueStruct[] PowerReadingVoltage;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = SENSOR_POWER_NUM)] public CalibrationValueStruct[] PowerReadingCurrent;
    };

#endregion

    #region Device-specific Enums

public enum FAN_MODE : byte
    {
        FAN_MODE_TEMP_CONTROL,
        FAN_MODE_FIXED,
        FAN_MODE_EXT
    }

    public enum TEMP_SRC : byte
    {
        TEMP_SRC_AUTO,
        TEMP_SRC_TS1,
        TEMP_SRC_TS2,
        TEMP_SRC_TS1A,
        TEMP_SRC_TS2A,
        TEMP_SRC_TAMB
    }

    public enum FAN_SWITCH_STATUS : byte {
        FAN_SWITCH_STATUS_AUTO,
        FAN_SWITCH_STATUS_50_PERCENT,
        FAN_SWITCH_STATUS_100_PERCENT
    };

    public enum RGB_SWITCH_STATUS : byte
    {
        RGB_SWITCH_STATUS_WORK,
        RGB_SWITCH_STATUS_PLAY
    };
    public enum RGB_EXT_STATUS : byte
    {
        RGB_EXT_STATUS_NOT_DETECTED,
        RGB_EXT_STATUS_DETECTED
    };

    private enum UART_CMD : byte
    {
        UART_CMD_WELCOME,
        UART_CMD_READ_SENSORS,
        UART_CMD_BUTTON_PRESS,
        UART_CMD_READ_NAME,
        UART_CMD_WRITE_NAME,
        UART_CMD_READ_FAN_PROFILE,
        UART_CMD_WRITE_FAN_PROFILE,
        UART_CMD_READ_RGB,
        UART_CMD_WRITE_RGB,
        UART_CMD_READ_CALIBRATION,
        UART_CMD_WRITE_CALIBRATION,
        UART_CMD_READ_UID,
        UART_CMD_RSVD1
    }

    public enum RGB_MODE : byte {
        RGB_MODE_STATIC,
        RGB_MODE_RAINBOW,
        RGB_MODE_RAINBOW_CYCLE,
        RGB_MODE_RAINBOW_COLOR_CHASE,
        RGB_MODE_FADE_IN_OUT,
        RGB_MODE_FADE,
        RGB_MODE_TWINKLE,
        RGB_MODE_METEOR_SHOWER,
        RGB_MODE_COLOR_WIPE,
        RGB_MODE_THEATRE_CHASE,
        RGB_MODE_SINGLE_COLOR_CHASE
    };

    public enum RGB_DIRECTION : byte {
        RGB_DIRECTION_CLOCKWISE,
        RGB_DIRECTION_COUNTER_CLOCKWISE
    };


private static byte[] ToByteArray(UART_CMD uartCMD, int len = 0)
    {
        byte[] returnArray = new byte[len + 1];
        returnArray[0] = (byte)uartCMD;
        return returnArray;
    }

    private enum UART_NVM_CMD : byte
    {
        CONFIG_SAVE,
        CONFIG_LOAD,
        CONFIG_RESET
    }

    public enum BUTTON : byte
    {
        BUTTON_POWER,
        BUTTON_RESET,
        BUTTON_OTHER
    };

    #endregion

    #region Variables

    #region Identifiers

    public string Name => "BENCHLAB";
    public Guid Guid { get; private set; } = Guid.Empty;

    #endregion

    #region Status indicators

    public DeviceStatus Status { get; private set; } = DeviceStatus.DISCONNECTED;
    public int Version { get; private set; }
    public string Port { get; private set; } = String.Empty;

    #endregion

    #region Private variables

    private readonly List<byte> RxData = new();
    private SerialPort? _serialPort;
    private byte[] oled_fb = new byte[128 * 64 / 8];

    #endregion

    public List<Sensor> SensorList = new();

    #endregion

    #region Public Methods

    #region Connection

    public virtual bool Connect(string comPort = "COM34")
    {

        Port = comPort;
        Status = DeviceStatus.CONNECTING;

        try
        {
            _serialPort = new SerialPort(comPort)
            {
                BaudRate = 115200,
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                ReadTimeout = 500,
                WriteTimeout = 500,
                RtsEnable = true,
                DtrEnable = true
            };

        }
        catch (Exception e)
        {
            Status = DeviceStatus.ERROR;
            return false;
        }

        try
        {
            _serialPort.Open();
            _serialPort.DataReceived += SerialPortOnDataReceived;
        }
        catch (Exception e)
        {
            Status = DeviceStatus.ERROR;
            return false;
        }

        SensorList.Clear();
        int sensorCount;
        for (sensorCount = 0; sensorCount < SENSOR_VIN_NUM; sensorCount++)
        {
            SensorList.Add(new Sensor(sensorCount, $"V{sensorCount + 1}", $"Voltage #{sensorCount + 1}", SensorType.Voltage));
        }

        SensorList.Add(new Sensor(sensorCount++, "VDD", "Supply Voltage", SensorType.Voltage));
        SensorList.Add(new Sensor(sensorCount++, "VREF", "Reference Voltage", SensorType.Voltage));
        SensorList.Add(new Sensor(sensorCount++, "T_CHIP", "Chip Temperature", SensorType.Temperature));

        for (int i = 0; i < SENSOR_TS_NUM; i++)
        {
            SensorList.Add(new Sensor(sensorCount++, $"TS{i + 1}", $"Temperature Sensor #{i + 1}", SensorType.Temperature));
        }

        SensorList.Add(new Sensor(sensorCount++, "T_AMB", "Ambient Temperature", SensorType.Temperature));
        SensorList.Add(new Sensor(sensorCount++, "HUM", "Humidity", SensorType.Humidity));
        //SensorList.Add(new Sensor(sensorCount++, "FAN_SEL", "Fan Select", SensorType.Voltage));
        SensorList.Add(new Sensor(sensorCount++, "FAN_EXT", "Fan External", SensorType.Duty));

        SensorList.Add(new Sensor(sensorCount++, $"SYS_P", "System Power", SensorType.Power));
        SensorList.Add(new Sensor(sensorCount++, $"CPU_P", "CPU Power", SensorType.Power));
        SensorList.Add(new Sensor(sensorCount++, $"GPU_P", "GPU Power", SensorType.Power));
        SensorList.Add(new Sensor(sensorCount++, $"MB_P", "Motherboard Power", SensorType.Power));

        foreach (string power_sensor in PowerSensorNames)
        {
            SensorList.Add(new Sensor(sensorCount++, $"{power_sensor}_V", $"{power_sensor} Voltage", SensorType.Voltage));
            SensorList.Add(new Sensor(sensorCount++, $"{power_sensor}_I", $"{power_sensor} Current", SensorType.Current));
            SensorList.Add(new Sensor(sensorCount++, $"{power_sensor}_P", $"{power_sensor} Power", SensorType.Power));
        }

        for (int i = 0; i < FAN_NUM; i++)
        {
            SensorList.Add(new Sensor(sensorCount++, $"FAN{i + 1}_T", $"Fan Speed #{i + 1}", SensorType.Revolutions));
            SensorList.Add(new Sensor(sensorCount++, $"FAN{i + 1}_D", $"Fan Duty #{i + 1}", SensorType.Duty));
        }


        bool connected = CheckWelcomeMessage();

        if (connected)
        {
            // Update UID
            connected = UpdateUID();
        }

        if (connected)
        {
            Status = DeviceStatus.CONNECTED;
        }
        else
        {
            Status = DeviceStatus.ERROR;
        }

        _serialPort.DiscardInBuffer();

        return connected;
    }

    public virtual bool Disconnect()
    {
        Status = DeviceStatus.DISCONNECTING;

        if (_serialPort != null)
        {
            try
            {
                _serialPort.DataReceived -= SerialPortOnDataReceived;
                _serialPort.DiscardInBuffer();
                _serialPort.Close();
                _serialPort.Dispose();
                _serialPort = null;
            }
            catch
            {
                Status = DeviceStatus.ERROR;
                return false;
            }
        }
        else
        {
            return false;
        }

        Status = DeviceStatus.DISCONNECTED;
        return true;
    }

    /*public virtual bool Reset(bool bootloaderMode) {

        byte[] txBuffer = ToByteArray(bootloaderMode ? UART_CMD.UART_CMD_BOOTLOADER : UART_CMD.UART_CMD_RESET);

        // Send command to device
        try {
            SendCommand(txBuffer, out _, 0);
        } catch {
            return false;
        }

        return true;

    }*/

    public static void GetAvailableDevices(out List<Device_BENCHLAB> deviceList)
    {

        deviceList = new List<Device_BENCHLAB>();

        List<string> ports = GetBenchlabPorts();

        // Connect to first available device
        foreach (string port in ports)
        {
            Device_BENCHLAB temp_device = new Device_BENCHLAB();
            if (temp_device.Connect(port))
            {
                deviceList.Add(temp_device);
            }
            //temp_device.Disconnect();
        }

    }

    private static List<string> GetBenchlabPorts()
    {
        List<string> ports = new();

        if (OperatingSystem.IsWindows())
        {
            // Open registry to find matching CH340 USB-Serial ports
            RegistryKey? masterRegKey = null;

            try
            {
                masterRegKey = Registry.LocalMachine.OpenSubKey(@"SYSTEM\CurrentControlSet\Enum\USB\VID_0483&PID_5740");
            }
            catch
            {
                return ports;
            }

            if (masterRegKey == null) return ports;
            foreach (string subKey in masterRegKey.GetSubKeyNames())
            {
                // Name must contain either VCP or Serial to be valid. Process any entries NOT matching
                // Compare to subKey (name of RegKey entry)
                try
                {
                    RegistryKey? subRegKey = masterRegKey.OpenSubKey($"{subKey}\\Device Parameters");
                    if (subRegKey == null) continue;

                    string? value = (string?)subRegKey.GetValue("PortName");

                    if (subRegKey.GetValueKind("PortName") != RegistryValueKind.String) continue;

                    if (value != null) ports.Add(value);
                }
                catch
                {
                    continue;
                }
            }

            masterRegKey.Close();
        }

        else if (OperatingSystem.IsLinux() || OperatingSystem.IsMacOS())
        {
            try
            {
                string[] ttyList = Directory.GetDirectories("/sys/bus/usb-serial/devices/");

                foreach (string portFile in ttyList)
                {
                    string realPortFile = Mono.Unix.UnixPath.GetRealPath(portFile);
                    string checkPath = Path.GetFullPath(Path.Combine(realPortFile, "../uevent"));
                    string[] lines = File.ReadAllLines(checkPath);
                    string productLine = lines.First(l => l.StartsWith("PRODUCT="));
                    string[] productSplit = productLine.Remove(0, 8).Trim().Split('/');

                    if (productSplit[0] == "1a86" && productSplit[1] == "7523")
                    {
                        string portDevPath = Path.Combine("/dev/", Path.GetFileName(portFile));
                        ports.Add(portDevPath);
                    }
                }
            }
            catch
            {
                return ports;
            }
        }

        return ports;
    }

    public SerialPort? BorrowSerialPort()
    {
        if (_serialPort != null)
        {
            _serialPort.DataReceived -= SerialPortOnDataReceived;
        }
        return _serialPort;
    }

    public void ReturnSerialPort()
    {
        if (_serialPort != null)
        {
            _serialPort.DataReceived += SerialPortOnDataReceived;
        }
    }

    public bool Refresh()
    {

        bool connected = CheckWelcomeMessage();

        if (connected)
        {
            // Update UID
            //connected = UpdateUID();
        }

        if (connected)
        {
            Status = DeviceStatus.CONNECTED;
        }
        else
        {
            Status = DeviceStatus.ERROR;
        }

        if (_serialPort != null)
        {
            _serialPort.DiscardInBuffer();
        }

        return connected;
    }

    #endregion

    #region Functionality

    public virtual bool UpdateSensors()
    {
        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_SENSOR_VALUES);
        byte[] rxBuffer;

        SensorStruct sensorStruct = new()
        {
            // Prevent null possibility
            Vin = new short[SENSOR_VIN_NUM],
            Ts = new short[SENSOR_TS_NUM],
            PowerReadings = new PowerSensor[SENSOR_POWER_NUM]
        };

        // Get struct size
        int size = Marshal.SizeOf(sensorStruct);

        // Get values from device
        try
        {
            bool commandResult = SendCommand(txBuffer, out rxBuffer, size);
            if (!commandResult) return false;
        }
        catch
        {
            return false;
        }

        // Convert byte array to struct

        IntPtr ptr = IntPtr.Zero;
        try
        {
            ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(rxBuffer, 0, ptr, size);

            object? structObj = Marshal.PtrToStructure(ptr, typeof(SensorStruct));
            if (structObj != null)
            {
                sensorStruct = (SensorStruct)structObj;
            }
        }
        catch
        {
            return false;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }

        // Update sensor list
        int sensorCount;
        for (sensorCount = 0; sensorCount < SENSOR_VIN_NUM; sensorCount++)
        {
            SensorList[sensorCount].Value = sensorStruct.Vin[sensorCount] == 0x7FFF ? double.MinValue : sensorStruct.Vin[sensorCount] / 1000.0f;
        }

        SensorList[sensorCount++].Value = sensorStruct.Vdd / 1000.0f;
        SensorList[sensorCount++].Value = sensorStruct.Vref / 1000.0f;
        SensorList[sensorCount++].Value = sensorStruct.Tchip;

        for (int i = 0; i < SENSOR_TS_NUM; i++)
        {
            SensorList[sensorCount++].Value = sensorStruct.Ts[i] == 0x7FFF ? double.MinValue : sensorStruct.Ts[i] / 10.0f;
        }

        SensorList[sensorCount++].Value = sensorStruct.Tamb / 10.0f;
        SensorList[sensorCount++].Value = sensorStruct.Hum / 10.0f;
        //SensorList[sensorCount++].Value = sensorStruct.FanSwitchStatus;
        SensorList[sensorCount++].Value = sensorStruct.FanExtDuty == 255 ? double.MinValue : sensorStruct.FanExtDuty;

        // Calculate total power, cpu power, gpu power, mb power
        // readonly string[] PowerSensorNames = { "EPS1", "EPS2", "ATX3V", "ATX5V", "ATX5VSB", "ATX12V", "PCIE1", "PCIE2", "PCIE3", "HPWR1", "HPWR2" };

        double cpu_power = sensorStruct.PowerReadings[0].Power + sensorStruct.PowerReadings[1].Power; // EPS1 + EPS2
        double gpu_power = sensorStruct.PowerReadings[6].Power + sensorStruct.PowerReadings[7].Power + sensorStruct.PowerReadings[8].Power + sensorStruct.PowerReadings[9].Power + sensorStruct.PowerReadings[10].Power;
        double mb_power = sensorStruct.PowerReadings[2].Power + sensorStruct.PowerReadings[3].Power + sensorStruct.PowerReadings[4].Power + sensorStruct.PowerReadings[5].Power;
        double total_power = cpu_power + gpu_power + mb_power;

        SensorList[sensorCount++].Value = total_power / 1000.0f;
        SensorList[sensorCount++].Value = cpu_power / 1000.0f;
        SensorList[sensorCount++].Value = gpu_power / 1000.0f;
        SensorList[sensorCount++].Value = mb_power / 1000.0f;

        for (int i = 0; i < SENSOR_POWER_NUM; i++)
        {
            SensorList[sensorCount++].Value = sensorStruct.PowerReadings[i].Voltage / 1000.0f;
            SensorList[sensorCount++].Value = sensorStruct.PowerReadings[i].Current / 1000.0f;
            SensorList[sensorCount++].Value = sensorStruct.PowerReadings[i].Power / 1000.0f;
        }

        for (int i = 0; i < FAN_NUM; i++)
        {
            SensorList[sensorCount++].Value = sensorStruct.Fans[i].Tach;
            SensorList[sensorCount++].Value = sensorStruct.Fans[i].Duty;
        }

        return true;
    }

    /*public virtual bool SetFanDuty(int fanId, int fanDuty) {
        // Duty = txBuffer[2]
        // 0~100 (0x00~0x64) for duty control in percentage
        // 255 (0xFF) for MCU embedded control

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WRITE_FAN_DUTY, 2);
        txBuffer[1] = (byte)fanId;
        txBuffer[2] = (byte)fanDuty;

        try {
            SendCommand(txBuffer, out _, 0);
        } catch {
            return false;
        }

        return true;
    }*/

    // Get device configuration
    /*public virtual bool GetConfigItems(out DeviceConfigStruct deviceConfigStruct) {

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_CONFIG);
        byte[] rxBuffer;

        deviceConfigStruct = new DeviceConfigStruct();

        // Get data from device
        try {
            SendCommand(txBuffer, out rxBuffer, 220);
        } catch {
            return false;
        }

        // Calc CRC16
        ushort crc16Calc = Helper.CRC16_Calc(rxBuffer, 2, rxBuffer.Length - 2);

        // Convert byte array to struct
        int size = Marshal.SizeOf(deviceConfigStruct);
        IntPtr ptr = IntPtr.Zero;
        try {
            ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(rxBuffer, 0, ptr, size);

            object? structObj = Marshal.PtrToStructure(ptr, typeof(DeviceConfigStruct));
            if(structObj != null) {
                deviceConfigStruct = (DeviceConfigStruct)structObj;
            }
        } finally {
            Marshal.FreeHGlobal(ptr);
        }

        // Firmware version 01 reports Crc = 0 if not loaded from NVM
        if(Version != 0x01 && crc16Calc != deviceConfigStruct.Crc) {
            return false;
        }

        return true;
    }

    public virtual bool SetConfigItems(DeviceConfigStruct deviceConfigStruct) {

        // Firmware version 01 write config is bugged, so fail directly
        if(Version == 0x01) {
            return false;
        }

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WRITE_CONFIG);

        // Send initial command
        try {
            SendCommand(txBuffer, out _, 0);
        } catch {
            return false;
        }

        // Convert struct to byte array
        int size = Marshal.SizeOf(deviceConfigStruct);
        txBuffer = new byte[size];
        IntPtr ptr = IntPtr.Zero;
        try {
            ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(deviceConfigStruct, ptr, true);
            Marshal.Copy(ptr, txBuffer, 0, size);
        } finally {
            Marshal.FreeHGlobal(ptr);
        }

        // Calc CRC16
        ushort crc16Calc = Helper.CRC16_Calc(txBuffer, 2, txBuffer.Length - 2);
        txBuffer[0] = (byte)crc16Calc;
        txBuffer[1] = (byte)(crc16Calc >> 8);

        // Send config data to device
        try {
            SendCommand(txBuffer, out _, 0);
        } catch {
            return false;
        }

        return true;
    }*/

    /*public virtual bool SaveConfig() {
        return SendNvmCommand(UART_NVM_CMD.CONFIG_SAVE);
    }
    public virtual bool LoadConfig() {
        return SendNvmCommand(UART_NVM_CMD.CONFIG_LOAD);
    }
    public virtual bool ResetConfig() {
        return SendNvmCommand(UART_NVM_CMD.CONFIG_RESET);
    */

    public bool SendButtonPress(BUTTON button)
    {
        byte[] txBuffer = new byte[] { (byte)UART_CMD.UART_CMD_BUTTON_PRESS, 0 };
        txBuffer[1] = (byte)(1 << (int)button);
        return SendCommand(txBuffer, out byte[] rxBuffer, 0);
    }

    #endregion

    #endregion

    #region Private Methods

    // Check device welcome message
    private bool CheckWelcomeMessage()
    {
        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WELCOME);
        if (!SendCommand(txBuffer, out byte[] rxBuffer, 13))
        {
            return false;
        }

        string welcome_str = Encoding.ASCII.GetString(rxBuffer, 0, 13);

        if (string.Compare(welcome_str, "BENCHLAB") != 0) return false;

        return true;
    }

    // Compare device id and store firmware version
    /*private bool CheckVendorData() {
        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_ID);
        if(!SendCommand(txBuffer, out byte[] rxBuffer, 3)) {
            return false;
        }

        if(rxBuffer[0] != 0xEE || rxBuffer[1] != 0x0E) return false;

        Version = rxBuffer[2];
        return true;
    }*/

    private bool UpdateUID() {

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_UID);
        if(!SendCommand(txBuffer, out byte[] rxBuffer, 12)) {
            return false;
        }

        Guid = new Guid(rxBuffer);

        return true;
    }

    // Data reception event
    private void SerialPortOnDataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        SerialPort serialPort = (SerialPort)sender;
        byte[] data = new byte[serialPort.BytesToRead];
        serialPort.Read(data, 0, data.Length);
        RxData.AddRange(data.ToList());
    }

    // Send command to BENCHLAB
    private bool SendCommand(byte[] txBuffer, out byte[] rxBuffer, int rxLen)
    {
        if (_serialPort == null) throw new Exception("Serial port has not been initialized!"); //return false;

        if (!_serialPort.IsOpen) _serialPort.Open();

        rxBuffer = new byte[rxLen];

        try
        {
            RxData.Clear();
            _serialPort.Write(txBuffer, 0, txBuffer.Length);
            int timeout = 50;

            while (timeout-- > 0 && RxData.Count != rxLen)
            {
                Thread.Sleep(10);
            }

            if (RxData.Count != rxBuffer.Length)
            {
                //throw new Exception($"Buffer size mismatch! Expected {rxLen}, got {RxData.Count}");
                return false;
            }

            rxBuffer = RxData.ToArray();
        }
        catch
        {
            //throw;
            return false;
        }

        return true;
    }

    /*private bool SendNvmCommand(UART_NVM_CMD cmd) {

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_NVM_CONFIG, 3);

        // Write key
        txBuffer[1] = 0x34;
        txBuffer[2] = 0x12;
        txBuffer[3] = (byte)cmd;

        // Send command
        try {
            SendCommand(txBuffer, out _, 0);
        } catch {
            return false;
        }

        return true;

    }*/

    public bool ReadRgbConfig(int profileId, out RGBConfigStruct rgbConfigStruct)
    {

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_RGB, 1);
        txBuffer[1] = (byte)profileId;
        byte[] rxBuffer;

        rgbConfigStruct = new() { };

        if (profileId >= RGB_PROFILE_NUM)
        {
            return false;
        }

        // Get struct size
        int size = Marshal.SizeOf(rgbConfigStruct);

        // Get values from device
        try
        {
            bool commandResult = SendCommand(txBuffer, out rxBuffer, size);
            if (!commandResult) return false;
        }
        catch
        {
            return false;
        }

        IntPtr ptr = IntPtr.Zero;
        try
        {
            ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(rxBuffer, 0, ptr, size);

            object? structObj = Marshal.PtrToStructure(ptr, typeof(RGBConfigStruct));
            if (structObj != null)
            {
                rgbConfigStruct = (RGBConfigStruct)structObj;
            }
        }
        catch
        {
            return false;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }

        return true;
    }

    public bool WriteRgbConfig(int profileId, RGBConfigStruct rgbConfigStruct)
    {

        // Get struct size
        int size = Marshal.SizeOf(rgbConfigStruct);

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WRITE_RGB, 1 + size);
        txBuffer[1] = (byte)profileId;

        if (profileId >= RGB_PROFILE_NUM)
        {
            return false;
        }

        IntPtr ptr = IntPtr.Zero;
        try
        {
            ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(rgbConfigStruct, ptr, true);
            Marshal.Copy(ptr, txBuffer, 2, size);
        }
        catch
        {
            return false;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }

        // Send values to device
        try
        {
            bool commandResult = SendCommand(txBuffer, out _, size);
            if (!commandResult) return false;
        }
        catch
        {
            return false;
        }

        return true;
    }

    public bool ReadFanConfig(int profileId, out FanConfigStruct fanConfigStruct)
    {

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_READ_FAN_PROFILE, 1);
        txBuffer[1] = (byte)profileId;
        byte[] rxBuffer;

        fanConfigStruct = new() { };

        if (profileId >= FAN_NUM)
        {
            return false;
        }

        // Get struct size
        int size = Marshal.SizeOf(fanConfigStruct);

        // Get values from device
        try
        {
            bool commandResult = SendCommand(txBuffer, out rxBuffer, size);
            if (!commandResult) return false;
        }
        catch
        {
            return false;
        }

        IntPtr ptr = IntPtr.Zero;
        try
        {
            ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(rxBuffer, 0, ptr, size);

            object? structObj = Marshal.PtrToStructure(ptr, typeof(FanConfigStruct));
            if (structObj != null)
            {
                fanConfigStruct = (FanConfigStruct)structObj;
            }
        }
        catch
        {
            return false;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }

        return true;
    }

    public bool WriteFanConfig(int profileId, FanConfigStruct fanConfigStruct)
    {

        // Get struct size
        int size = Marshal.SizeOf(fanConfigStruct);

        byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WRITE_FAN_PROFILE, 1 + size);
        txBuffer[1] = (byte)profileId;

        if (profileId >= RGB_PROFILE_NUM)
        {
            return false;
        }

        IntPtr ptr = IntPtr.Zero;
        try
        {
            ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(fanConfigStruct, ptr, true);
            Marshal.Copy(ptr, txBuffer, 2, size);
        }
        catch
        {
            return false;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }

        // Send values to device
        try
        {
            bool commandResult = SendCommand(txBuffer, out _, size);
            if (!commandResult) return false;
        }
        catch
        {
            return false;
        }

        return true;
    }

    #endregion
}