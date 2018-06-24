using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    public class Header
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<Header>();
        private readonly IDictionary<string, string> values = new Dictionary<string, string>();

        public IDictionary<string, string> Values => values;

        public Header()
        {
        }

        public Header(IDictionary<string, string> values)
        {
            this.values = values;
        }

        public bool Parse(byte[] buffer, int size, out string errorMsg)
        {
            int i = 0;
            while (i < size)
            {
                int length = BitConverter.ToInt32(buffer, i);
                i += 4;
                byte[] lineBuffer = new byte[length];
                Array.Copy(buffer, i, lineBuffer, 0, length);
                string line = Encoding.ASCII.GetString(lineBuffer);
                string[] chunks = line.Split('=');
                if (chunks.Length != 2)
                {
                    i += length;
                    continue;
                }
                values[chunks[0].Trim()] = chunks[1].Trim();
                i += length;
            }

            if (i != size)
            {
                errorMsg = "Could not parse connection header.";
                logger.LogWarning(errorMsg);
                return false;
            }

            errorMsg = null;
            return true;
        }

        public static void Write(IDictionary<string, string> fields, out byte[] buffer, out int totalLength)
        {
            var ms = new MemoryStream();
            using (var writer = new BinaryWriter(ms, Encoding.ASCII))
            {
                foreach (string k in fields.Keys)
                {
                    byte[] key = Encoding.ASCII.GetBytes(k);
                    byte[] val = Encoding.ASCII.GetBytes(fields[k]);
                    int lineLength = val.Length + key.Length + 1;

                    writer.Write(lineLength);
                    writer.Write(key);
                    writer.Write('=');
                    writer.Write(val);
                }
            }

            ms.TryGetBuffer(out ArraySegment<byte> result);
            buffer = new byte[result.Count];
            Array.Copy(result.Array, result.Offset, buffer, 0, result.Count);
            totalLength = result.Count;
        }

        public static byte[] ByteLength(int num) =>
            BitConverter.GetBytes(num);

        public static byte[] ByteLength(uint num) =>
            BitConverter.GetBytes(num);
    }
}
