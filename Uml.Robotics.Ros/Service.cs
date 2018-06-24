using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;

namespace Uml.Robotics.Ros
{
    public static class Service
    {
        public static async Task<bool> Exists(string serviceName, bool logFailureReason = false)
        {
            string mappedName = Names.Resolve(serviceName);

            string host;
            int port;

            try
            {
                (host, port) = await ServiceManager.Instance.LookupServiceAsync(mappedName);
            }
            catch
            {
                if (logFailureReason)
                {
                    ROS.Info()("waitForService: Service[{0}] has not been advertised, waiting...", mappedName);
                }
                return false;
            }

            using (var tcpClient = new TcpClient())
            {
                try
                {
                    await tcpClient.ConnectAsync(host, port);
                }
                catch
                {
                    if (logFailureReason)
                    {
                        ROS.Info()("waitForService: Service[{0}] could not connect to host [{1}:{2}], waiting...", mappedName, host, port);
                    }

                    return false;
                }

                var headerFields = new Dictionary<string, string>
                {
                    { "probe", "1" },
                    { "md5sum", "*" },
                    { "callerid", ThisNode.Name },
                    { "service", mappedName }
                };

                Header.Write(headerFields, out byte[] headerbuf, out int size);

                byte[] sizebuf = BitConverter.GetBytes(size);

                var stream = tcpClient.GetStream();
                await stream.WriteAsync(sizebuf, 0, sizebuf.Length);
                await stream.WriteAsync(headerbuf, 0, size);
            }

            return true;
        }

        public static async Task<bool> WaitForService(string serviceName, TimeSpan timeout)
        {
            DateTime startTime = DateTime.UtcNow;
            bool printed = false;

            while (ROS.OK)
            {
                if (await Exists(serviceName, !printed))
                {
                    break;
                }

                printed = true;

                if (timeout >= TimeSpan.Zero)
                {
                    if (DateTime.UtcNow - startTime > timeout)
                        return false;
                }

                await Task.Delay(ROS.WallDuration);
            }

            if (printed && ROS.OK)
            {
                string mappedName = Names.Resolve(serviceName);
                ROS.Info()("waitForService: Service[{0}] is now available.", mappedName);
            }
            return true;
        }

        public static Task<bool> WaitForService(string serviceName, int timeout)
        {
            return WaitForService(serviceName, TimeSpan.FromMilliseconds(timeout));
        }
    }
}
