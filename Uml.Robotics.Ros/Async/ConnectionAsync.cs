using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;

namespace Uml.Robotics.Ros.Async
{
    public class ConnectionAsync
    {
        public enum DropReason
        {
            TransportDisconnect,
            HeaderError,
            Destructing
        }

        ILogger Logger { get; } = ApplicationLogging.CreateLogger<Connection>();
        NetworkStream stream;

        public ConnectionAsync(NetworkStream stream)
        {
            this.stream = stream;
        }



    }
}
