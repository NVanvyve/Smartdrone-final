using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace TagComm
{
    public class TagHandler
    {
        public readonly List<Action<List<uint>>> Callbacks = new List<Action<List<uint>>>();
        public readonly List<Action<bool>> FunctionalCallbacks = new List<Action<bool>>();
        private readonly Action<string> _infoLogger;
        private readonly Action<string> _errorLogger;
        private readonly SerialPort _serialPort;
        private bool _functional;

        public bool Functional
        {
            get { return _functional; }
            private set
            {
                _functional = value;
                FunctionalCallbacks.ForEach(callback => callback(Functional));
            }
        }

        public TagHandler(string portName) : this(portName, Console.Out.WriteLine, Console.Error.WriteLine)
        {
        }

        public TagHandler(string portName, Action<string> infoLogger, Action<string> errorLogger)
        {
            _infoLogger = infoLogger;
            _errorLogger = errorLogger;

            _serialPort = new SerialPort
            {
                PortName = portName,
                BaudRate = 115200,
                Parity = Parity.None,
                DataBits = 8,
                StopBits = StopBits.One,
                Handshake = Handshake.None,
                ReadTimeout = 1000
            };
        }

        private void WaitSerialPortOpening()
        {
            while (!_serialPort.IsOpen) //TODO Check for interrupt ?
            {
                try
                {
                    _serialPort.Open();
                }
                catch (IOException)
                {
                    _errorLogger("Could not find Decawave tag on port " + _serialPort.PortName);
                    Thread.Sleep(3000);
                }
            }
        }

        public void ReadProcess()
        {
            try
            {
                WaitSerialPortOpening();

                Functional = true;

                _serialPort.WriteLine("deca?");
                while (true)
                {
                    try
                    {
                        try
                        {
                            var line = _serialPort.ReadLine();
                            if (line == string.Empty)
                            {
                                Functional = false;
                                // Happens when disconnecting USB.
                                _errorLogger("Reading empty data. Unplugged USB assumed.");
                                _serialPort.Close();
                                WaitSerialPortOpening();
                            }

                            var report = new RangeReport(line);

                            if (report.mid != "mc") continue; // Only used corrected bias reports

                            var distances = new List<uint>();
                            for (byte anchorNumber = 0; anchorNumber < 4; anchorNumber++)
                            {
                                if (report.HasAnchor(anchorNumber))
                                {
                                    distances.Add(report.ranges[anchorNumber]);
                                }
                            }

                            Callbacks.ForEach(callback => callback(distances));
                        }
                        catch (ArgumentException)
                        {
                            _infoLogger("Could not parse => report ignored.");
                        }
                    }
                    catch (TimeoutException)
                    {
                        _errorLogger("The device is not answering. No signal from anchor 0 available ?");
                    }
                }
            }
            finally
            {
                Functional = false;
                _serialPort.Close();
            }
        }
    }
}