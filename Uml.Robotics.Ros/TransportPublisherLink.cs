using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    /// <summary>
    /// Establishes a connection to a publisher and reads messages from it.
    /// </summary>
    internal class TransportPublisherLink
        : PublisherLink
    {
        private static readonly TimeSpan BASE_RETRY_DELAY = TimeSpan.FromMilliseconds(50);
        private static readonly TimeSpan MAX_RETRY_DELAY = TimeSpan.FromSeconds(2);

        private readonly ILogger logger = ApplicationLogging.CreateLogger<TransportPublisherLink>();

        Connection connection;
        bool dropping;

        string host;
        int port;

        CancellationTokenSource cts;
        CancellationToken cancel;
        Task receiveLoop;
        TimeSpan retryDelay;

        public TransportPublisherLink(Subscription parent, string xmlRpcUri)
            : base(parent, xmlRpcUri)
        {
            retryDelay = BASE_RETRY_DELAY;
            cts = new CancellationTokenSource();
            cancel = cts.Token;
        }

        public override void Dispose()
        {
            dropping = true;
            cts.Cancel();
            Parent.RemovePublisherLink(this);
            if (receiveLoop != null)
            {
                receiveLoop.WhenCompleted().Wait();       // wait for publisher loop to terminate
            }
        }

        private async Task WriteHeader()
        {
            var header = new Dictionary<string, string>
            {
                ["topic"] = Parent.Name,
                ["md5sum"] = Parent.Md5Sum,
                ["callerid"] = ThisNode.Name,
                ["type"] = Parent.DataType,
                ["tcp_nodelay"] = "1"
            };
            await connection.WriteHeader(header, cancel);
        }

        private async Task HandleConnection()
        {
            // establish connection
            using (var client = new TcpClient())
            {
                await client.ConnectAsync(host, port);
                client.NoDelay = true;

                try
                {
                    this.connection = new Connection(client);

                    // write/read header handshake
                    await WriteHeader();
                    var headerFields = await connection.ReadHeader(cancel);
                    SetHeader(new Header(headerFields));

                    while (!cancel.IsCancellationRequested)
                    {
                        // read message length
                        int length = await connection.ReadInt32(cancel);
                        if (length > Connection.MESSAGE_SIZE_LIMIT)
                        {
                            var message = $"Message received in TransportPublisherLink exceeds length limit of {Connection.MESSAGE_SIZE_LIMIT}. Dropping connection";
                            throw new RosException(message);
                        }

                        // read message
                        var messageBuffer = await connection.ReadBlock(length, cancel);

                        // deserialize message
                        RosMessage msg = RosMessage.Generate(Parent.DataType);
                        msg.Serialized = messageBuffer;
                        msg.connection_header = this.Header.Values;
                        HandleMessage(msg);

                        // reset retry delay after first successfully processed message
                        retryDelay = BASE_RETRY_DELAY;
                    }

                    client.Close();
                }
                finally
                {
                    this.connection = null;
                }
            }
        }

        public async Task RunReceiveLoopAsync()
        {
            await Task.Yield();     // do not block the thread starting the loop

            while (true)
            {
                cancel.ThrowIfCancellationRequested();

                try
                {
                    await HandleConnection();
                }
                catch (HeaderErrorException)
                {
                    logger.LogError($"Error in the Header: {Parent?.Name ?? "unknown"}");
                    return;     // no retry in case of header error
                }
                catch (Exception e)
                {
                    if (dropping || cancel.IsCancellationRequested)
                    {
                        return;     // no retry when disposing
                    }

                    logger.LogError(e, e.Message);

                    retryDelay = retryDelay * 2;
                    if (retryDelay > MAX_RETRY_DELAY)
                    {
                        retryDelay = MAX_RETRY_DELAY;
                    }

                    // wait abortable for retry
                    await Task.Delay(retryDelay, cancel);
                }
            }
        }

        public void Initialize(string host, int port)
        {
            logger.LogDebug("Init transport publisher link: " + Parent.Name);

            this.host = host;
            this.port = port;

            receiveLoop = RunReceiveLoopAsync();
        }

        public void HandleMessage<T>(T m)
            where T : RosMessage, new()
        {
            Stats.BytesReceived += m.Serialized.Length;
            Stats.MessagesReceived++;
            m.connection_header = this.Header.Values;
            if (Parent != null)
                Stats.Drops += Parent.HandleMessage(m, true, false, connection.Header.Values, this);
            else
                Console.WriteLine($"{nameof(Parent)} is null");
        }
    }
}
