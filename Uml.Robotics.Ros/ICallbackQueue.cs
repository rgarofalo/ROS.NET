using System;

namespace Uml.Robotics.Ros
{
    public enum CallOneResult
    {
        Called,
        TryAgain,
        Disabled,
        Empty
    }

    public interface ICallbackQueue
        : IDisposable
    {
        bool IsEnabled { get; }
        bool IsEmpty { get; }

        void AddCallback(CallbackInterface callback, object owner = null);
        void RemoveByOwner(object owner);

        void CallAvailable(int timeout = ROS.WallDuration);
        CallOneResult CallOne(int timeout = ROS.WallDuration);

        void Enable();
        void Disable();
        void Clear();
    }
}
