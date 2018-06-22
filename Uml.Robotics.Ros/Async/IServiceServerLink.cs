using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    public interface IServiceServerLink
    {
        bool IsValid { get; }

        System.Net.Sockets.Socket Socket { get;  }
        NetworkStream Stream { get; }

        Task<bool> Call(RosService srv);
        Task<(bool, RosMessage)> Call(RosMessage req);
    }
}
