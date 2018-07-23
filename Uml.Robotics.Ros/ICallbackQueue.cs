using System;

namespace Uml.Robotics.Ros
{
    public interface ICallbackQueue
    {
        void AddCallback(CallbackInterface callback, long ownerId);
        void RemoveById(long ownerId);

        void CallAvailable(int timeout = ROS.WallDuration);

        void Enable();
        void Disable();
        void Clear();
    }
}
