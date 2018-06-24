using System;
using System.Collections.Generic;
using System.Net;

namespace Uml.Robotics.Ros
{
    public static class Network
    {
        public static string Host;
        public static int TcpRosServerPort;

        public static bool SplitUri(string uri, out string host, out int port)
        {
            host = null;
            port = 11311;
            if (String.IsNullOrEmpty(uri))
                return false;

            // remove URI scheme
            if (uri.Substring(0, 7) == "http://")
                host = uri.Substring(7);
            else if (uri.Substring(0, 9) == "rosrpc://")
                host = uri.Substring(9);

            string[] split = host.Split(':');
            if (split.Length < 2)
                return false;

            string portPart = split[1];
            portPart = portPart.Trim('/');
            port = int.Parse(portPart);
            host = split[0];
            return true;
        }

        public static void Init(IDictionary<string, string> remappings)
        {
            if (remappings.ContainsKey("__hostname"))
            {
                Host = remappings["__hostname"];
            }
            else if (remappings.ContainsKey("__ip"))
            {
                Host = remappings["__ip"];
            }

            if (remappings.ContainsKey("__tcpros_server_port"))
            {
                TcpRosServerPort = int.Parse(remappings["__tcpros_server_port"]);
            }

            if (string.IsNullOrEmpty(Host))
                Host = Dns.GetHostName();
        }
    }
}
