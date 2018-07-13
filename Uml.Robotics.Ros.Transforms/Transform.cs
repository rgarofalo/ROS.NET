using Messages.std_msgs;

namespace Uml.Robotics.Ros.Transforms
{
    public class Transform
    {
        public string ChildFrameId { get; set; }
        public string FrameId { get; set; }

        public Quaternion Basis { get; set; }
        public Time Stamp { get; set; }
        public Vector3 Origin { get; set; }

        public Transform()
            : this(new Quaternion(), new Vector3(), new Time(new TimeData()), "", "")
        {
        }

        public Transform(Messages.geometry_msgs.TransformStamped msg)
            : this(
                new Quaternion(msg.transform.rotation),
                new Vector3(msg.transform.translation),
                msg.header.stamp,
                msg.header.frame_id,
                msg.child_frame_id
            )
        {
        }

        public Transform(Quaternion basis, Vector3 origin, Time stamp = null, string frameId = null, string childFrameId = null)
        {
            this.Basis = basis;
            this.Origin = origin;
            this.Stamp = stamp;
            this.FrameId = frameId;
            this.ChildFrameId = childFrameId;
        }

        public static Transform operator *(Transform t, Transform v)
        {
            return new Transform(t.Basis * v.Basis, t * v.Origin);
        }

        public static Vector3 operator *(Transform t, Vector3 v)
        {
            Matrix3x3 mat = new Matrix3x3(t.Basis);
            return new Vector3(mat.m_el[0].Dot(v) + t.Origin.X,
                mat.m_el[1].Dot(v) + t.Origin.Y,
                mat.m_el[2].Dot(v) + t.Origin.Z);
        }

        public static Quaternion operator *(Transform t, Quaternion q)
        {
            return t.Basis * q;
        }

        public override string ToString()
        {
            return "\ttranslation: " + Origin + "\n\trotation: " + Basis;
        }
    }
}