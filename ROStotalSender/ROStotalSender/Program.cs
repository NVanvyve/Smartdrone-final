using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;
using IronPython.Hosting;
using Microsoft.Scripting.Hosting;

namespace ROStotalSender
{
    public class Coord : EventArgs
    {
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public DateTime timestamp { get; set;}
        public int status { get; set; }
        public string toString()
        {
            //return this.status + "(" + this.x + "," + this.y + "," + this.z + ")";
            return (this.x + ";" + this.y + ";" + this.z +";" + this.timestamp).Replace(',', '.');
        }
    }

    public class Program
    {
        public event EventHandler<Coord> ReceiveCoord;

        Thread thread;
        bool isContinue;
        ScriptRuntime ipy;
        dynamic script;
        static Socket s;
        static IPEndPoint ip;

        public Program()
        { }

        public void Open(String port, string baudrate)
        {
            s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            const int buffSize = 4 * 1024;
            ip = new IPEndPoint(IPAddress.Parse("192.168.0.150"), 15000);
            ipy = Python.CreateRuntime();
            script = ipy.UseFile("track_h.py");
            script.open(port, baudrate);
            isContinue = true;
            thread = new Thread(Run);
            thread.IsBackground = true;
            thread.Start();
        }

        public void Run()
        {
            while (isContinue)
            {
                // Receive coord
                string rcv = (script.get_measure()).Replace('.', ',');
                Console.WriteLine(rcv);
                String[] sxyz = rcv.Split(';');
                Coord c = new Coord();    // Parser coord
                c.status = int.Parse(sxyz[0]);
                if (c.status == 0 || c.status == 1)
                {
                    c.x = double.Parse(sxyz[1]);
                    c.y = double.Parse(sxyz[2]);
                    c.z = double.Parse(sxyz[3]);
                    c.timestamp = DateTime.Now;

                    ReceiveCoord?.Invoke(this, c);
                }
            }
            script.close();
        }

        static void eReceiveCoord(Object sender, Coord c)
        {
            byte[] data = Encoding.ASCII.GetBytes(c.toString());
            s.SendTo(data,SocketFlags.None,(EndPoint)(ip));
        }

        static void Main(string[] args)
        {
            Program p = new Program();
            p.Open(args[0], args[1]);
            p.ReceiveCoord += eReceiveCoord;
            while (true) {
                System.Threading.Thread.Sleep(100);
            }
        }
    }
}
