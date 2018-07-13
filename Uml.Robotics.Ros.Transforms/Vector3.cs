using System;
using System.Collections.Generic;
using System.Text;


namespace Uml.Robotics.Ros.Transforms
{
    public class Vector3
    {
        public double X;
        public double Y;
        public double Z;

        public Vector3()
            : this(0, 0, 0)
        {
        }

        public Vector3(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public Vector3(Vector3 v)
            : this(v.X, v.Y, v.Z)
        {
        }

        public Vector3(Messages.geometry_msgs.Vector3 msg)
            : this(msg.x, msg.y, msg.z)
        {
        }

        public Messages.geometry_msgs.Vector3 ToMsg()
        {
            return new Messages.geometry_msgs.Vector3 { x = X, y = Y, z = Z };
        }

        public static Vector3 operator +(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
        }

        public static Vector3 operator -(Vector3 v1)
        {
            return new Vector3(-v1.X, -v1.Y, -v1.Z);
        }

        public static Vector3 operator *(Vector3 v1, float d)
        {
            return v1 * ((double)d);
        }

        public static Vector3 operator *(Vector3 v1, int d)
        {
            return v1 * ((double)d);
        }

        public static Vector3 operator *(Vector3 v1, double d)
        {
            return new Vector3(d * v1.X, d * v1.Y, d * v1.Z);
        }

        public static Vector3 operator *(float d, Vector3 v1)
        {
            return v1 * ((double)d);
        }

        public static Vector3 operator *(int d, Vector3 v1)
        {
            return v1 * ((double)d);
        }

        public static Vector3 operator *(double d, Vector3 v1)
        {
            return v1 * d;
        }

        public double Dot(Vector3 v2)
        {
            return X * v2.X + Y * v2.Y + Z * v2.Z;
        }

        public override string ToString()
        {
            return string.Format("({0:F4},{1:F4},{2:F4})", X, Y, Z);
        }

        public static Vector3 Lerp(Vector3 v0, Vector3 v1, double rt)
        {
            double s = 1.0 - rt;
            return new Vector3(
                x:  s * v0.X + rt * v1.X,
                y:  s * v0.Y + rt * v1.Y,
                z:  s * v0.Z + rt * v1.Z
            );
        }
    }
}
