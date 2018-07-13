using std_msgs = Messages.std_msgs;

namespace Uml.Robotics.Ros.Transforms
{
    public enum TfStatus
    {
        NoError,
        LookupError,
        ConnectivityError,
        ExtrapolationError
    }

    public enum WalkEnding
    {
        Identity,
        TargetParentOfSource,
        SourceParentOfTarget,
        FullPath
    }

    public class Stamped<T>
    {
        public T Data { get; set; }
        public string FrameId { get; set; }
        public std_msgs.Time Stamp { get; set; }

        public Stamped()
        {
        }

        public Stamped(std_msgs.Time stamp, string frameId, T data)
        {
            this.Stamp = stamp;
            this.FrameId = frameId;
            this.Data = data;
        }
    }

    public struct TimeAndFrameId
    {
        public uint FrameId { get; set; }
        public ulong Time { get; set; }

        public TimeAndFrameId(ulong time, uint frameId)
        {
            this.Time = time;
            this.FrameId = frameId;
        }
    }

    public class TransformStorage
    {
        public uint ChildFrameId { get; set; }
        public uint FrameId { get; set; }
        public Quaternion Rotation { get; set; }
        public ulong Stamp { get; set; }
        public Vector3 Translation { get; set; }

        public TransformStorage()
        {
            this.Rotation = new Quaternion();
            this.Translation = new Vector3();
        }

        public TransformStorage(Transform data, uint frameId, uint childFrameId)
        {
            this.Rotation = data.Basis;
            this.Translation = data.Origin;
            this.Stamp = TimeCache.ToLong(data.Stamp.data);
            this.FrameId = frameId;
            this.ChildFrameId = childFrameId;
        }
    }
}
