﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class TransportSubscriberLink
        : SubscriberLink
        , IDisposable
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<TransportSubscriberLink>();
        private readonly object gate = new object();

        private Connection connection;
        private AsyncQueue<MessageAndSerializerFunc> outbox;

        private CancellationTokenSource cts;
        private CancellationToken cancel;
        private Task sendLoop;

        public TransportSubscriberLink()
        {
            this.cts = new CancellationTokenSource();
            this.cancel = cts.Token;
        }

        public override void Dispose()
        {
            UnregisterSubscriberLink();
        }

        private void UnregisterSubscriberLink()
        {
            lock (gate)
            {
                if (parent != null)
                {
                    parent.RemoveSubscriberLink(this);
                }
            }

            outbox?.Dispose();
            connection?.Dispose();
        }

        public void Initialize(Connection connection, Header header)
        {
            lock (gate)
            {
                if (sendLoop != null)
                    throw new InvalidOperationException();

                if (parent != null)
                {
                    logger.LogDebug("Init transport subscriber link: " + parent.Name);
                }

                this.connection = connection;
                sendLoop = RunSendLoopAsync(header);
            }
        }

        private async Task RunSendLoopAsync(Header header)
        {
            await Task.Yield();

            try
            {
                // header handshake
                try
                {
                    await HandleHeader(header);
                }
                catch (Exception e)
                {
                    logger.LogWarning(e, e.Message);
                    await connection.SendHeaderError(e.Message, cancel);
                    connection.Dispose();

                    throw;
                }

                // read messages from queue and send them
                while (await outbox.MoveNext(cancel))
                {
                    cancel.ThrowIfCancellationRequested();

                    var current = outbox.Current;
                    await WriteMessage(current);
                }
            }
            finally
            {
                UnregisterSubscriberLink();
            }
        }

        private async Task WriteMessage(MessageAndSerializerFunc message)
        {
            if (message.msg.Serialized == null)
                message.msg.Serialized = message.serfunc();

            int length = message.msg.Serialized.Length;
            await connection.Write(BitConverter.GetBytes(length), 0, 4, cancel);
            await connection.Write(message.msg.Serialized, 0, length, cancel);

            Stats.MessagesSent++;
            Stats.BytesSent += length + 4;
            Stats.MessageDataSent += length;
        }

        private async Task HandleHeader(Header header)
        {
            if (!header.Values.ContainsKey("topic"))
            {
                throw new RosException("Header from subscriber did not have the required element: topic");
            }

            string name = header.Values["topic"];
            string clientCallerId = header.Values["callerid"];

            Publication pt = TopicManager.Instance.LookupPublication(name);
            if (pt == null)
            {
                throw new RosException($"Received a connection for a nonexistent topic [{name}] from [{connection.Socket.RemoteEndPoint}] [{clientCallerId}]");
            }

            if (!pt.ValidateHeader(header, out string errorMessage))
            {
                throw new RosException(errorMessage);
            }

            DestinationCallerId = clientCallerId;
            connectionId = ConnectionManager.Instance.GetNewConnectionId();
            name = pt.Name;
            parent = pt;

            this.outbox = new AsyncQueue<MessageAndSerializerFunc>(parent.MaxQueue);

            var m = new Dictionary<string, string>
            {
                ["type"] = pt.DataType,
                ["md5sum"] = pt.Md5Sum,
                ["message_definition"] = pt.MessageDefinition,
                ["callerid"] = ThisNode.Name,
                ["latching"] = Convert.ToString(pt.Latch)
            };

            await connection.WriteHeader(m, cancel);
            pt.AddSubscriberLink(this);
            logger.LogDebug("Finalize transport subscriber link for " + name);
        }

        public override void EnqueueMessage(MessageAndSerializerFunc holder)
        {
            if (!outbox.TryOnNext(holder))
            {
                // TODO: handle queue full case
            }
        }

        public override void GetPublishTypes(ref bool ser, ref bool nocopy, string type_info)
        {
            throw new NotImplementedException();
        }
    }
}
