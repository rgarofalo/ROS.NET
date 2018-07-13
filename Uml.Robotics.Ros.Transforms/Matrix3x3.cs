using System;
using System.Collections.Generic;
using System.Text;

namespace Uml.Robotics.Ros.Transforms
{
    public class Matrix3x3
    {
        public Vector3[] m_el = new Vector3[3];

        public Matrix3x3()
            : this(0, 0, 0, 0, 0, 0, 0, 0, 0)
        {
        }

        public Matrix3x3(
            double xx, double xy, double xz,
            double yx, double yy, double yz,
            double zx, double zy, double zz)
        {
            SetValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
        }

        public Matrix3x3(Quaternion q)
        {
            SetRotation(q);
        }

        public void SetValue(
            double xx, double xy, double xz,
            double yx, double yy, double yz,
            double zx, double zy, double zz)
        {
            m_el[0] = new Vector3(xx, xy, xz);
            m_el[1] = new Vector3(yx, yy, yz);
            m_el[2] = new Vector3(zx, zy, zz);
        }

        public void SetRotation(Quaternion q)
        {
            double d = q.Length2;
            double s = 2.0 / d;
            double xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
            double wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
            double xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
            double yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
            SetValue(1.0 - (yy + zz), xy - wz, xz + wy,
                xy + wz, 1.0 - (xx + zz), yz - wx,
                xz - wy, yz + wx, 1.0 - (xx + yy));
        }

        public static Vector3 operator *(Matrix3x3 mat1, Vector3 v1)
        {
            return new Vector3(mat1.m_el[0].X * v1.X + mat1.m_el[0].Y * v1.Y + mat1.m_el[0].Z * v1.Z,
                               mat1.m_el[1].X * v1.X + mat1.m_el[1].Y * v1.Y + mat1.m_el[1].Z * v1.Z,
                               mat1.m_el[2].X * v1.X + mat1.m_el[2].Y * v1.Y + mat1.m_el[2].Z * v1.Z);
        }

        internal Vector3 GetYPR(uint solution_number = 1)
        {
            Euler euler_out;
            Euler euler_out2; //second solution

            // Check that pitch is not at a singularity
            if (Math.Abs(m_el[2].X) >= 1)
            {
                euler_out.Yaw = 0;
                euler_out2.Yaw = 0;

                // From difference of angles formula
                if (m_el[2].X < 0) //gimbal locked down
                {
                    double delta = Math.Atan2(m_el[0].Y, m_el[0].Z);
                    euler_out.Pitch = Math.PI / 2.0d;
                    euler_out2.Pitch = Math.PI / 2.0d;
                    euler_out.Roll = delta;
                    euler_out2.Roll = delta;
                }
                else // gimbal locked up
                {
                    double delta = Math.Atan2(-m_el[0].Y, -m_el[0].Z);
                    euler_out.Pitch = -Math.PI / 2.0d;
                    euler_out2.Pitch = -Math.PI / 2.0d;
                    euler_out.Roll = delta;
                    euler_out2.Roll = delta;
                }
            }
            else
            {
                euler_out.Pitch = -Math.Asin(m_el[2].X);
                euler_out2.Pitch = Math.PI - euler_out.Pitch;

                euler_out.Roll = Math.Atan2(m_el[2].Y / Math.Cos(euler_out.Pitch),
                    m_el[2].Z / Math.Cos(euler_out.Pitch));
                euler_out2.Roll = Math.Atan2(m_el[2].Y / Math.Cos(euler_out2.Pitch),
                    m_el[2].Z / Math.Cos(euler_out2.Pitch));

                euler_out.Yaw = Math.Atan2(m_el[1].X / Math.Cos(euler_out.Pitch),
                    m_el[0].X / Math.Cos(euler_out.Pitch));
                euler_out2.Yaw = Math.Atan2(m_el[1].X / Math.Cos(euler_out2.Pitch),
                    m_el[0].X / Math.Cos(euler_out2.Pitch));
            }

            if (solution_number == 1)
            {
                return new Vector3(euler_out.Yaw, euler_out.Pitch, euler_out.Roll);
            }

            return new Vector3(euler_out2.Yaw, euler_out2.Pitch, euler_out2.Roll);
        }

        public override string ToString()
        {
            return string.Format("({0:F4},{1:F4},{2:F4}; {3:F4},{4:F4},{5:F4}; {6:F4},{7:F4},{8:F4})",
                                 m_el[0].X, m_el[0].Y, m_el[0].Z,
                                 m_el[1].X, m_el[1].Y, m_el[1].Z,
                                 m_el[2].X, m_el[2].Y, m_el[2].Z);
        }

        public struct Euler
        {
            public double Pitch;
            public double Roll;
            public double Yaw;
        }
    }
}
