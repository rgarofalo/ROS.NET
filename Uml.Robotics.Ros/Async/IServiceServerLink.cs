using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    public interface IServiceServerLink
    {
        bool IsValid { get; }

        Task<bool> Call(RosService srv);
        Task<(bool, RosMessage)> Call(RosMessage req);
    }
}
