using System;
using System.Net.Sockets;
using System.Threading.Tasks;

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
