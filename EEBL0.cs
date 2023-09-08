using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static System.Net.Mime.MediaTypeNames;

namespace BENCHLAB_Core
{
    public class EEBL0 {

        private const string WELCOME_STRING = "EEBL0";
        private const byte RESULT_SUCCESS = (byte)'s';

        private enum UART_CMD : byte {
            UART_CMD_WELCOME = 0x00,
            UART_CMD_SIZE = (byte)'#',
            UART_CMD_RUN_APP = (byte)'a',
            UART_CMD_ERASE = (byte)'e',
            UART_CMD_PROGRAM_PAGE = (byte)'p',
            UART_CMD_VERIFY_PAGE = (byte)'v',
            UART_CMD_RESET = (byte)'r'
        }
        private static byte[] ToByteArray(UART_CMD uartCMD, int len = 0) {
            byte[] returnArray = new byte[len + 1];
            returnArray[0] = (byte)uartCMD;
            return returnArray;
        }

        private readonly List<byte> RxData = new();
        private SerialPort? _serialPort;

        #region Connection

        public virtual bool Connect(string comPort = "COM34")
        {

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
            catch
            {
                return false;
            }

            try
            {
                _serialPort.Open();
                _serialPort.DataReceived += SerialPortOnDataReceived;
            }
            catch
            {
                return false;
            }

            if (CheckWelcomeString())
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public virtual bool Connect(SerialPort? borrowedSerialPort)
        {
            _serialPort = borrowedSerialPort;

            try
            {
                _serialPort.DataReceived += SerialPortOnDataReceived;
            }
            catch
            {
                return false;
            }

            if (CheckWelcomeString())
            {
                return true;
            }
            else
            {
                ReturnSerialPort();
                return false;
            }
        }

        public virtual bool Disconnect()
        {

            if (_serialPort != null)
            {
                try
                {
                    _serialPort.DataReceived -= SerialPortOnDataReceived;
                    _serialPort.Close();
                    _serialPort.Dispose();
                    _serialPort = null;
                }
                catch
                {
                    return false;
                }
            }
            else
            {
                return false;
            }

            return true;
        }

        public void ReturnSerialPort()
        {
            if (_serialPort != null)
            {
                _serialPort.DataReceived -= SerialPortOnDataReceived;
            }
        }

        #endregion

        #region Functionality

        public virtual bool Erase() {

            byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_ERASE);
            try {
                SendCommand(txBuffer, out byte[] rxBuffer, 1);
                return rxBuffer[0] == RESULT_SUCCESS;
            } catch {
                return false;
            }

        }
        public virtual bool Program(byte[] in_data) {

            // Require 16K-1K-256B sized firmware
            byte[] data = new byte[15104];
            for(int i = in_data.Length; i<data.Length; i++) {
                data[i] = 0xFF;
            }
            Array.Copy(in_data, 0, data, 0, in_data.Length > data.Length ? data.Length : in_data.Length);

            for(int page = 0; page < data.Length / 64; page++) {
                byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_PROGRAM_PAGE);
                try {

                    // Start page program
                    SendCommand(txBuffer, out byte[] rxBuffer, 1);
                    if(rxBuffer[0] != RESULT_SUCCESS) {
                        return false;
                    }

                    // Send page data
                    txBuffer = new byte[64];
                    Array.Copy(data, page * 64, txBuffer, 0, txBuffer.Length);
                    SendCommand(txBuffer, out rxBuffer, 1);
                    if(rxBuffer[0] != RESULT_SUCCESS) {
                        return false;
                    }
                } catch {
                    return false;
                }
            }

            return true;

        }
        public virtual bool Verify(byte[] in_data) {

            // Require 16K-1K-256B sized firmware
            byte[] data = new byte[15104];
            for(int i = in_data.Length; i < data.Length; i++) {
                data[i] = 0xFF;
            }
            Array.Copy(in_data, 0, data, 0, in_data.Length > data.Length ? data.Length : in_data.Length);

            for(int page = 0; page < data.Length / 64; page++) {
                byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_VERIFY_PAGE);
                try {

                    // Send verify command
                    SendCommand(txBuffer, out byte[] rxBuffer, 65);
                    if(rxBuffer[0] != RESULT_SUCCESS) {
                        return false;
                    }

                    // Compare page data
                    for(int i = 0; i < 64; i++) {
                        if(data[page * 64 + i] != rxBuffer[i + 1]) {
                            return false;
                        }
                    }
                } catch {
                    return false;
                }
            }

            return true;

        }

        public virtual bool Reset() {
            byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_RESET);
            try
            {
                SendCommand(txBuffer, out _, 0);
            } catch
            {
                return false;
            }
            return true;
        }

        public virtual bool RunApplication() {

            byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_RUN_APP);
            try {
                SendCommand(txBuffer, out _, 0);
            } catch {
                return false;
            }
            return true;
        }


        #endregion

        #region Private functions

        // Check if welcome message is correct
        private bool CheckWelcomeString() {
            byte[] txBuffer = ToByteArray(UART_CMD.UART_CMD_WELCOME);
            SendCommand(txBuffer, out byte[] rxBuffer, 6);

            string welcome_str = System.Text.Encoding.ASCII.GetString(rxBuffer);
            if(string.Compare(welcome_str, WELCOME_STRING) != 0) return false;

            return true;
        }

        // Data reception event
        private void SerialPortOnDataReceived(object sender, SerialDataReceivedEventArgs e) {
            SerialPort serialPort = (SerialPort)sender;
            byte[] data = new byte[serialPort.BytesToRead];
            serialPort.Read(data, 0, data.Length);
            RxData.AddRange(data.ToList());
        }

        // Send command to bootloader
        private bool SendCommand(byte[] txBuffer, out byte[] rxBuffer, int rxLen) {
            rxBuffer = new byte[rxLen];

            if(_serialPort == null) return false;

            try {
                RxData.Clear();
                _serialPort.Write(txBuffer, 0, txBuffer.Length);
                int timeout = 50;

                while(timeout-- > 0 && RxData.Count != rxLen) {
                    Thread.Sleep(10);
                }

                if(RxData.Count != rxBuffer.Length) {
                    return false;
                }

                rxBuffer = RxData.ToArray();
            } catch {
                return false;
            }

            return true;
        }

        #endregion
    }
}
