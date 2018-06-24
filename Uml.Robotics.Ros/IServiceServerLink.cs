using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Uml.Robotics.Ros
{
    public interface IServiceServerLinkAsync
        : IDisposable
    {
        bool IsValid { get; }

        Socket Socket { get;  }
        NetworkStream Stream { get; }

        Task<bool> Call(RosService srv);
        Task<(bool, RosMessage)> Call(RosMessage req);
    }
}
