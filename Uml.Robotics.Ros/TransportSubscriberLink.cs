using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Extensions.Logging;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class TransportSubscriberLink
        : SubscriberLink
        , IDisposable
    {
        readonly ILogger logger = ApplicationLogging.CreateLogger<TransportSubscriberLink>();
        ConnectionAsync connection;
        bool headerWritten;
        int maxQueue;
        AsyncQueue<MessageAndSerializerFunc> outbox;
        bool queueFull;
        bool writingMessage;

        public TransportSubscriberLink(Publication parent)
        {
            parent.MaxQueue
            outbox = new AsyncQueue<MessageAndSerializerFunc>(queueSize);
            writingMessage = false;
            headerWritten = false;
            queueFull = false;
        }

        public void Dispose()
        {
            Drop();
        }

        public bool Initialize(ConnectionAsync connection)
        {
            if (parent != null)
                logger.LogDebug("Init transport subscriber link: " + parent.Name);
            this.connection = connection;
            connection.DroppedEvent += OnConnectionDropped;
            return true;
        }

        public bool HandleHeader(Header header)
        {
            if (!header.Values.ContainsKey("topic"))
            {
                string msg = "Header from subscriber did not have the required element: topic";
                logger.LogWarning(msg);
                connection.sendHeaderError(ref msg);
                return false;
            }
            string name = (string) header.Values["topic"];
            string client_callerid = (string) header.Values["callerid"];
            Publication pt = TopicManagerAsync.Instance.LookupPublication(name);
            if (pt == null)
            {
                string msg = "received a connection for a nonexistent topic [" + name + "] from [" +
                             connection.transport + "] [" + client_callerid + "]";
                logger.LogWarning(msg);
                connection.sendHeaderError(ref msg);
                return false;
            }
            string error_message = "";
            if (!pt.ValidateHeader(header, ref error_message))
            {
                connection.sendHeaderError(ref error_message);
                logger.LogError(error_message);
                return false;
            }
            destination_caller_id = client_callerid;
            connection_id = ConnectionManager.Instance.GetNewConnectionId();
            name = pt.Name;
            parent = pt;
            lock (parent)
            {
                maxQueue = parent.MaxQueue;
            }

            var m = new Dictionary<string, string>
            {
                ["type"] = pt.DataType,
                ["md5sum"] = pt.Md5Sum,
                ["message_definition"] = pt.MessageDefinition,
                ["callerid"] = ThisNode.Name,
                ["latching"] = Convert.ToString(pt.Latch)
            };
            connection.writeHeader(m, OnHeaderWritten);
            pt.AddSubscriberLink(this);
            logger.LogDebug("Finalize transport subscriber link for " + name);
            return true;
        }

        internal override void EnqueueMessage(MessageAndSerializerFunc holder)
        {
            lock (outbox)
            {
                if (maxQueue > 0 && outbox.Count >= maxQueue)
                {
                    outbox.Dequeue();
                    queueFull = true;
                }
                else
                {
                    queueFull = false;
                }
                outbox.Enqueue(holder);
            }
            StartMessageWrite(false);
        }

        public override void Drop()
        {
            if (connection.sendingHeaderError)
                connection.DroppedEvent -= OnConnectionDropped;
            else
                connection.drop(Connection.DropReason.Destructing);
        }

        private void OnConnectionDropped(Connection conn, Connection.DropReason reason)
        {
            if (conn != connection || parent == null)
                return;

            lock (parent)
            {
                parent.RemoveSubscriberLink(this);
            }
        }

        private bool OnHeaderWritten(Connection conn)
        {
            headerWritten = true;
            StartMessageWrite(true);
            return true;
        }

        private bool OnMessageWritten(Connection conn)
        {
            writingMessage = false;
            StartMessageWrite(true);
            return true;
        }

        private void StartMessageWrite(bool immediateWrite)
        {
            MessageAndSerializerFunc holder = null;
            if (writingMessage || !headerWritten)
                return;

            lock (outbox)
            {
                if (outbox.Count > 0)
                {
                    writingMessage = true;
                    holder = outbox.Dequeue();
                }
                if (outbox.Count < maxQueue)
                    queueFull = false;
            }

            if (holder != null)
            {
                if (holder.msg.Serialized == null)
                    holder.msg.Serialized = holder.serfunc();
                byte[] outbuf = new byte[holder.msg.Serialized.Length + 4];
                Array.Copy(holder.msg.Serialized, 0, outbuf, 4, holder.msg.Serialized.Length);
                Array.Copy(BitConverter.GetBytes(holder.msg.Serialized.Length), outbuf, 4);
                Stats.MessagesSent++;
                //Logger.LogDebug("Message backlog = " + (triedtosend - stats.messages_sent));
                Stats.BytesSent += outbuf.Length;
                Stats.MessageDataSent += outbuf.Length;
                connection.write(outbuf, outbuf.Length, OnMessageWritten, immediateWrite);
            }
        }
    }
}
