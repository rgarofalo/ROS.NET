using Xunit;
using Uml.Robotics.Ros;
using tf = Uml.Robotics.Ros.Transforms;
using System.Collections.Generic;
using Xunit.Extensions;
using System;

namespace Uml.Robotics.Ros.UnitTests
{
    public class Matrix3x3Theories
    {
        [Theory, MemberData(nameof(RotationData))]
        public void Should_SetRotationMatrixFromQuaternion(tf.Quaternion quaternion, tf.Vector3 vector, tf.Vector3 expectedResult = null)
        {
            //Utils.WaitForDebugger();

            Console.WriteLine("");
            Console.WriteLine("Test: SetRotationMatrixFromQuaternion");
            Console.WriteLine("=====================================");
            //var matrix = new tf.Matrix3x3(quaternion);
            var matrix = new tf.Matrix3x3();
            matrix.SetRotation(quaternion);
            Console.WriteLine("Quaternion for rotation:");
            Console.WriteLine(quaternion.ToString());
            Console.WriteLine("Corresponding rotation matrix:");
            Console.WriteLine(matrix.ToString());
            Console.WriteLine("Vector to be rotated:");
            Console.WriteLine(vector.ToString());

            Console.WriteLine("Result of rotation via quaternion-vector-mult (qpq^-1):");
            var resultQuaternion = quaternion * vector;
            //Console.WriteLine(resultQuaternion.ToString());
            var resultVector1 = new tf.Vector3(resultQuaternion.X, resultQuaternion.Y, resultQuaternion.Z);
            Console.WriteLine(resultVector1.ToString());

            Console.WriteLine("Result of rotation via quaternion conversion into rotation matrix:");
            var resultVector2 = matrix * vector;
            Console.WriteLine(resultVector2.ToString());

            var tolerance = 1E-05;
            Assert.InRange(Math.Abs(resultVector1.X - resultVector2.X), 0.0, tolerance);
            Assert.InRange(Math.Abs(resultVector1.Y - resultVector2.Y), 0.0, tolerance);
            Assert.InRange(Math.Abs(resultVector1.Z - resultVector2.Z), 0.0, tolerance);
            if (expectedResult != null) {
                Assert.InRange(Math.Abs(resultVector1.X - expectedResult.X), 0.0, tolerance);
                Assert.InRange(Math.Abs(resultVector1.Y - expectedResult.Y), 0.0, tolerance);
                Assert.InRange(Math.Abs(resultVector1.Z - expectedResult.Z), 0.0, tolerance);
            }
        }

        public static IEnumerable<object[]> RotationData
        {
            get
            {
                return new[]
                {
                    // Remember quaternion initialization order: x, y, z, w
                    new object[] {new tf.Quaternion(0, 0, 0, 1), new tf.Vector3(1, 1, 1)},
                    new object[] {new tf.Quaternion(2, 2, 2, 1), new tf.Vector3(1, 1, 1)},
                    new object[] {new tf.Quaternion(3, 2, 4, 1), new tf.Vector3(1, 2, 3)},
                    // The following should rotate (1,0,0) around the y-axis with angle pi/2.
                    // Hence, the result should be (0,0,-1).
                    // See: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
                    // and http://run.usc.edu/cs520-s12/quaternions/quaternions-cs520.pdf, slide 18.
                    new object[] {new tf.Quaternion(0, Math.Sqrt(2.0)/2.0, 0, Math.Sqrt(2.0)/2.0), new tf.Vector3(1, 0, 0), new tf.Vector3(0, 0, -1)}
                };
            }
        }
    }
}