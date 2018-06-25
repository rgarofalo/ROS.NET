using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    public class ConnectionError : RosException
    {
        public ConnectionError(string message)
            : base(message)
        {
        }
    }

    public delegate void ConnectionDisposeHandler(Connection connection);

    public class Connection
        : IDisposable
    {
        public const int MESSAGE_SIZE_LIMIT = 1000000000;

        private readonly ILogger logger;
        private TcpClient client;

        private bool disposed;
        private string topic;

        public NetworkStream Stream { get; }
        public Socket Socket => client.Client;
        public Header Header { get; } = new Header();

        public Connection(TcpClient client)
        {
            this.client = client;
            this.Stream = client.GetStream();
            this.logger = ApplicationLogging.CreateLogger<Connection>();
        }

        public void Dispose()
        {
            if (disposed)
                return;

            disposed = true;
            client?.Dispose();
            client = null;
            Disposed?.Invoke(this);
        }

        public event ConnectionDisposeHandler Disposed;

        public bool IsValid
             => !disposed && client.Connected;

        /// <summary>Returns the ID of the connection</summary>
        public string CallerId
        {
            get
            {
                if (Header != null && Header.Values.ContainsKey("callerid"))
                    return Header.Values["callerid"];
                return string.Empty;
            }
        }

        public async Task<IDictionary<string, string>> ReadHeader(CancellationToken cancel)
        {
            int length = await this.ReadInt32(cancel);
            if (length > MESSAGE_SIZE_LIMIT)
                throw new ConnectionError("Invalid header length received");

            byte[] headerBuffer = await this.ReadBlock(length, cancel);

            if (!Header.Parse(headerBuffer, length, out string errorMessage))
            {
                throw new ConnectionError(errorMessage);
            }

            if (Header.Values.ContainsKey("error"))
            {
                string error = Header.Values["error"];
                logger.LogInformation("Received error message in header for connection to [{0}]: [{1}]",
                    "TCPROS connection to [" + this.Socket.RemoteEndPoint + "]", error);
                throw new ConnectionError(error);
            }

            if (topic == null && Header.Values.ContainsKey("topic"))
            {
                topic = Header.Values["topic"];
            }

            if (Header.Values.ContainsKey("tcp_nodelay"))
            {
                if (Header.Values["tcp_nodelay"] == "1")
                {
                    this.Socket.NoDelay = true;
                }
            }

            return Header.Values;
        }

        public async Task SendHeaderError(string errorMessage, CancellationToken cancel)
        {
            var header = new Dictionary<string, string>
            {
                { "error", errorMessage }
            };

            await WriteHeader(header, cancel);
        }

        public async Task WriteHeader(IDictionary<string, string> headerValues, CancellationToken cancel)
        {
            Header.Write(headerValues, out byte[] headerBuffer, out int headerLength);
            int messageLength = (int)headerLength + 4;

            byte[] messageBuffer = new byte[messageLength];
            Buffer.BlockCopy(BitConverter.GetBytes(headerLength), 0, messageBuffer, 0, 4);
            Buffer.BlockCopy(headerBuffer, 0, messageBuffer, 4, headerBuffer.Length);
            await Stream.WriteAsync(messageBuffer, 0, messageBuffer.Length, cancel);
        }

        public async Task<int> ReadInt32(CancellationToken cancel)
        {
            byte[] lengthBuffer = await ReadBlock(4, cancel);
            return BitConverter.ToInt32(lengthBuffer, 0);
        }

        public async Task<byte[]> ReadBlock(int size, CancellationToken cancel)
        {
            var buffer = new byte[size];
            await ReadBlock(new ArraySegment<byte>(buffer), cancel);
            return buffer;
        }

        public async Task ReadBlock(ArraySegment<byte> buffer, CancellationToken cancel)
        {
            using (cancel.Register(() => Stream.Close()))
            {
                if (!await Stream.ReadBlockAsync(buffer.Array, buffer.Offset, buffer.Count, cancel))
                {
                    throw new EndOfStreamException("Connection closed gracefully");
                }
            }
        }

        public async Task WriteBlock(ArraySegment<byte> buffer, CancellationToken cancel)
        {
            await WriteBlock(buffer.Array, buffer.Offset, buffer.Count, cancel);
        }

        public async Task WriteBlock(byte[] buffer, int offset, int count, CancellationToken cancel)
        {
            using (cancel.Register(() => Stream.Close()))
            {
                await Stream.WriteAsync(buffer, offset, count, cancel);
            }
        }
    }
}
