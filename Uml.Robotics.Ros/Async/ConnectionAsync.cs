using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    public class ConnectionError : Exception
    {
        public ConnectionError(string message)
            : base(message)
        {
        }
    }

    public class ConnectionAsync
    {
        public const int MESSAGE_SIZE_LIMIT = 1000000000;

        readonly ILogger logger;
        TcpClient client;
        NetworkStream stream;
        public Header header = new Header();

        // connection values read from header
        string topic;

        bool sendingHeaderError;

        public ConnectionAsync(TcpClient client)
        {
            this.client = client;
            this.stream = client.GetStream();
            this.logger = ApplicationLogging.CreateLogger<ConnectionAsync>();
        }

        public NetworkStream Stream => stream;
        public System.Net.Sockets.Socket Socket => client.Client;

        /// <summary>Returns the ID of the connection</summary>
        public string CallerId
        {
            get
            {
                if (header != null && header.Values.ContainsKey("callerid"))
                    return header.Values["callerid"];
                return string.Empty;
            }
        }

        public async Task<IDictionary<string, string>> ReadHeader(CancellationToken cancel)
        {
            var lengthBuffer = new byte[4];
            await this.ReadBlock(lengthBuffer, cancel);
            int length = BitConverter.ToInt32(lengthBuffer, 0);

            if (length > MESSAGE_SIZE_LIMIT)
                throw new ConnectionError("Invalid header length received");

            byte[] headerBuffer = await this.ReadBlock(length, cancel);

            if (!header.Parse(headerBuffer, length, out string errorMessage))
            {
                throw new ConnectionError(errorMessage);
            }

            if (header.Values.ContainsKey("error"))
            {
                string error = header.Values["error"];
                logger.LogInformation("Received error message in header for connection to [{0}]: [{1}]",
                    "TCPROS connection to [" + this.Socket.RemoteEndPoint + "]", error);
                throw new ConnectionError(error);
            }

            if (topic == null && header.Values.ContainsKey("topic"))
            {
                topic = header.Values["topic"];
            }

            if (header.Values.ContainsKey("tcp_nodelay"))
            {
                if (header.Values["tcp_nodelay"] == "1")
                {
                    this.Socket.NoDelay = true;
                }
            }

            return header.Values;
        }

        public async Task SendHeaderError(string errorMessage, CancellationToken cancel)
        {
            var header = new Dictionary<string, string>
            {
                { "error", errorMessage }
            };

            sendingHeaderError = true;
            await WriteHeader(header, cancel);
        }

        public async Task WriteHeader(IDictionary<string, string> headerValues, CancellationToken cancel)
        {
            Header.Write(headerValues, out byte[] headerBuffer, out int headerLength);
            int messageLength = (int)headerLength + 4;

            byte[] messageBuffer = new byte[messageLength];
            Buffer.BlockCopy(BitConverter.GetBytes(headerLength), 0, messageBuffer, 0, 4);
            Buffer.BlockCopy(headerBuffer, 0, messageBuffer, 4, headerBuffer.Length);

            await stream.WriteAsync(messageBuffer, 0, messageBuffer.Length, cancel);
        }

        public async Task<byte[]> ReadBlock(int size, CancellationToken cancel)
        {
            var buffer = new byte[size];
            await ReadBlock(new ArraySegment<byte>(buffer), cancel);
            return buffer;
        }

        public async Task ReadBlock(ArraySegment<byte> buffer, CancellationToken cancel)
        {
            if (!await stream.ReadBlockAsync(buffer.Array, buffer.Offset, buffer.Count, cancel))
            {
                throw new EndOfStreamException("Connection closed gracefully");
            }
        }

        public async Task Write(byte[] buffer, int offset, int count, CancellationToken cancel)
        {
            await stream.WriteAsync(buffer, offset, count, cancel);
        }

        public void Dispose()
        {
            client?.Dispose();
            client = null;
        }
    }
}
