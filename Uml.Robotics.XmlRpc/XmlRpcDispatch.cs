using System;
using System.Linq;
using System.Collections.Generic;
using System.Net.Sockets;

namespace Uml.Robotics.XmlRpc
{
    public class XmlRpcDispatch
    {
        [Flags]
        public enum EventType
        {
            NoEvent = 0,
            ReadableEvent = 1,
            WritableEvent = 2,
            Exception = 4
        }

        private class DispatchRecord
        {
            public XmlRpcSource Client { get; set; }
            public EventType Mask { get; set; }
        }

        private List<DispatchRecord> sources = new List<DispatchRecord>();

        public void AddSource(XmlRpcSource source, EventType eventMask)
        {
            lock (sources)
            {
                sources.Add(new DispatchRecord { Client = source, Mask = eventMask });
            }
        }

        public void RemoveSource(XmlRpcSource source)
        {
            lock (sources)
            {
                sources.RemoveAll(x => x.Client == source);
            }
        }

        public void SetSourceEvents(XmlRpcSource source, EventType eventMask)
        {
            lock (sources)
            {
                foreach (var record in sources.Where(x => x.Client == source))
                {
                    record.Mask |= eventMask;
                }
            }
        }

        private void CheckSources(IEnumerable<DispatchRecord> sources, TimeSpan timeout, List<XmlRpcSource> toRemove)
        {
            const EventType ALL_EVENTS = EventType.ReadableEvent | EventType.WritableEvent | EventType.Exception;

            var checkRead = new List<Socket>();
            var checkWrite = new List<Socket>();
            var checkError = new List<Socket>();

            foreach (var src in sources)
            {
                var sock = src.Client.Socket;
                if (sock == null)
                    continue;

                var mask = src.Mask;
                if (mask.HasFlag(EventType.ReadableEvent))
                    checkRead.Add(sock);
                if (mask.HasFlag(EventType.WritableEvent))
                    checkWrite.Add(sock);
                if (mask.HasFlag(EventType.Exception))
                    checkError.Add(sock);
            }

            // Check for events
            Socket.Select(checkRead, checkWrite, checkError, (int)(timeout.Milliseconds * 1000.0));

            if (checkRead.Count + checkWrite.Count + checkError.Count == 0)
                return;

            // Process events
            foreach (var record in sources)
            {
                XmlRpcSource src = record.Client;
                EventType newMask = ALL_EVENTS;
                Socket sock = src.Socket;
                if (sock == null)
                    continue;

                // if you select on multiple event types this could be ambiguous
                if (checkRead.Contains(sock))
                    newMask &= src.HandleEvent(EventType.ReadableEvent);
                if (checkWrite.Contains(sock))
                    newMask &= src.HandleEvent(EventType.WritableEvent);
                if (checkError.Contains(sock))
                    newMask &= src.HandleEvent(EventType.Exception);

                if (newMask == EventType.NoEvent)
                {
                    toRemove.Add(src);
                }
                else
                {
                    record.Mask = newMask;
                }
            }
        }

        public void Work(TimeSpan timeSlice)
        {
            var endTime = DateTime.UtcNow.Add(timeSlice);

            while (sources.Count > 0)
            {
                List<DispatchRecord> sourcesCopy;
                lock (sources)
                {
                    sourcesCopy = sources.ToList();
                }

                var toRemove = new List<XmlRpcSource>();
                CheckSources(sourcesCopy, timeSlice, toRemove);

                if (toRemove.Count > 0)
                {
                    lock (sources)
                    {
                        foreach (var src in toRemove)
                        {
                            RemoveSource(src);
                            src.Close();
                        }
                    }
                }

                // check whether end time has been passed
                if (DateTime.UtcNow > endTime)
                    break;
            }
        }

        public void Clear()
        {
            sources.Clear();
        }
    }
}
