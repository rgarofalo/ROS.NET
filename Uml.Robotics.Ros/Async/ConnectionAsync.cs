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
    public struct Block
    {
        public static readonly Block Null = new Block(null);

        public readonly byte[] Data;

        public Block(byte[] data)
        {
            this.Data = data;
        }
    }

    public class BlockReceivedArgs : EventArgs
    {
        public Block Block { get; }

        public BlockReceivedArgs(Block block)
        {
            this.Block = block;
        }
    }

    public interface IConnection
        : IDisposable
    {
        bool TryAddToOutputQueue(Block block);
        Task AddToOutputQueue(Block block, CancellationToken cancel);
        Task WhenComplete { get; }
        event EventHandler<BlockReceivedArgs> BlockReceived;
        Task Close(TimeSpan? timeout = null);
    }

    public class ConnectionAsync
        : IConnection
    {
        public enum DropReason
        {
            TransportDisconnect,
            HeaderError,
            Destructing
        }

        readonly ILogger logger;
        readonly NetworkStream stream;
        public Header header = new Header();
        CancellationTokenSource cancelSource = new CancellationTokenSource();
        AsyncQueue<Block> outputQueue;

        public ConnectionAsync(NetworkStream stream, CancellationToken cancel = default(CancellationToken))
        {
            this.stream = stream;
            this.logger = ApplicationLogging.CreateLogger<ConnectionAsync>();
            this.cancelSource = CancellationTokenSource.CreateLinkedTokenSource(cancel);
            this.outputQueue = new AsyncQueue<Block>(1024);
        }

        public event EventHandler<BlockReceivedArgs> BlockReceived;
        public Task ConnectionTask { get; private set; }
        public Task SendTask { get; private set; }
        public Task ReceiveTask { get; private set; }
        public NetworkStream Stream { get; private set; }

        /// <summary>Returns the ID of the connection</summary>
        public string CallerID
        {
            get
            {
                if (header != null && header.Values.ContainsKey("callerid"))
                    return header.Values["callerid"];
                return string.Empty;
            }
        }

        public void Start()
        {
            if (this.ConnectionTask == null)
            {
                this.ConnectionTask = HandleConnection();
            }
        }

        public bool TryAddToOutputQueue(Block block)
        {
            return outputQueue.TryOnNext(block);
        }

        public async Task AddToOutputQueue(Block block, CancellationToken cancel)
        {
            await outputQueue.OnNext(block, cancel);
        }

        private async Task HandleConnection()
        {
            var cancel = cancelSource.Token;

            await this.Socket.ConnectAsync(uri, cancel);

            this.SendTask = Send();
            this.ReceiveTask = Receive();

            await Task.WhenAll(this.SendTask, this.ReceiveTask);
        }

        private async Task Send()
        {
            try
            {
                var cancel = cancelSource.Token;
                while (await outputQueue.MoveNext(cancel))
                {
                    Block block = outputQueue.Current;
                    var obj = block.Message;

                    if (obj == null)
                    {
                        // close websocket
                        await this.Socket.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "bye", cancel);
                        return;     // done
                    }
                    else
                    {
                        // serialize message to json
                        var ms = new MemoryStream();
                        using (var jsonWriter = new JsonTextWriter(new StreamWriter(ms)))
                        {
                            serializer.Serialize(jsonWriter, obj);
                        }

                        // send message to client
                        ms.TryGetBuffer(out ArraySegment<byte> data);
                        await this.stream.WriteAsync(data, WebSocketMessageType.Text, true, cancel);

                        // optionally send binary attachments of message
                        if (block.Attachments != null)
                        {
                            foreach (var attachment in block.Attachments)
                            {
                                await this.Socket.SendAsync(attachment, WebSocketMessageType.Binary, true, cancel);
                            }
                        }
                    }
                }
            }
            catch
            {
                cancelSource.Cancel();      // cancel receive task
                throw;
            }
        }

        private Block pendingReceive;
        private int remainingAttachments;

        private int GetAttachmentCount(object message)
        {
            if (message is JObject obj)
            {
                var attachmentCount = obj.GetValue("binaryAttachmentCount");        // well-known field name to indicate binary attachments will follow
                if (attachmentCount != null && attachmentCount.Type == JTokenType.Integer)
                {
                    return (int)attachmentCount;
                }
            }
            return 0;
        }

        private async Task Receive()
        {
            try
            {
                var cancel = cancelSource.Token;
                var ms = new MemoryStream();
                var buffer = new ArraySegment<byte>(new byte[64 * 1024]);
                while (!cancel.IsCancellationRequested)
                {
                    var receiveResult = await this.stream.ReceiveAsync(buffer, cancel);
                    if (receiveResult.MessageType == WebSocketMessageType.Close)
                    {
                        if (receiveResult.CloseStatus != WebSocketCloseStatus.NormalClosure)
                        {
                            throw new Exception($"Websocket closed connection unexpectedly. CloseStatus: '{receiveResult.CloseStatusDescription}' ({receiveResult.CloseStatus})");
                        }
                        this.TryAddToOutputQueue(Block.Null);     // close on flush
                        return;     // done
                    }

                    ms.Write(buffer.Array, 0, receiveResult.Count);
                    if (receiveResult.EndOfMessage)
                    {
                        if (receiveResult.MessageType == WebSocketMessageType.Text)
                        {
                            if (remainingAttachments > 0)
                                throw new Exception("Protocol error: Binary message expected.");

                            ms.Position = 0;
                            using (var jsonReader = new JsonTextReader(new StreamReader(ms)))
                            {
                                var msg = serializer.Deserialize(jsonReader);
                                remainingAttachments = GetAttachmentCount(msg);
                                if (remainingAttachments > 0)
                                    pendingReceive = new Block(msg, new List<byte[]>());
                                else
                                    OnBlockReceived(new Block(msg));
                            }
                        }
                        else
                        {
                            Debug.Assert(receiveResult.MessageType == WebSocketMessageType.Binary);

                            if (remainingAttachments < 1)
                                throw new Exception("Protocol error: No binary messge expected.");

                            pendingReceive.Attachments.Add(ms.ToArray());
                            remainingAttachments -= 1;
                            if (remainingAttachments == 0)
                            {
                                var msg = pendingReceive;
                                pendingReceive = Block.Null;
                                OnBlockReceived(msg);
                            }
                        }
                        ms = new MemoryStream();
                    }
                }
            }
            catch
            {
                cancelSource.Cancel();      // cancel send task
                throw;
            }
        }

        public Task WhenComplete => this.ConnectionTask;

        public async Task Close(TimeSpan? timeout = null)
        {
            var cancel = cancelSource.Token;

            await Task.WhenAll(AddToOutputQueue(Block.Null, cancel), this.SendTask).WhenCompleted().TimeoutAfter(timeout);

            try
            {
                await this.WhenComplete.WhenCompleted().TimeoutAfter(timeout);
                cancelSource.Cancel();
            }
            finally
            {
                Dispose();
            }
        }

        public void Dispose()
        {
            outputQueue.Dispose();
            cancelSource.Cancel();
            stream.Dispose();
        }

        protected virtual void OnBlockReceived(Block block)
        {
            try
            {
                this.BlockReceived?.Invoke(this, new BlockReceivedArgs(block));
            }
            catch (Exception error)
            {
                logger?.LogError(default(EventId), error, "Exception in MessageReceived event callback of Subscriber");
            }
        }

    }
}
