using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    public class ConnectionManager
    {
        private static Lazy<ConnectionManager> instance = new Lazy<ConnectionManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        public static ConnectionManager Instance =>
            instance.Value;

        internal static void Terminate() =>
            Instance.Shutdown();

        internal static void Reset() =>
            instance = new Lazy<ConnectionManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        private readonly ILogger logger = ApplicationLogging.CreateLogger<ConnectionManager>();
        private readonly object gate = new object();

        private int nextConnectionId;

        private List<Connection> connections = new List<Connection>();
        private TcpListener listener;
        private Task acceptLoop;
        private CancellationTokenSource cts;
        private CancellationToken cancel;

        public ConnectionManager()
        {
            cts = new CancellationTokenSource();
            cancel = cts.Token;
        }

        public int TCPPort
        {
            get
            {
                if (listener == null || listener.LocalEndpoint == null)
                    return -1;

                return ((IPEndPoint)listener.LocalEndpoint).Port;
            }
        }

        public int GetNewConnectionId() =>
            Interlocked.Increment(ref nextConnectionId);

        public void AddConnection(Connection connection)
        {
            lock (gate)
            {
                connections.Add(connection);
                connection.Disposed += Connection_Disposed;
            }
        }

        public void Clear()
        {
            Connection[] connections = null;

            lock (gate)
            {
                connections = this.connections.ToArray();
                this.connections.Clear();
            }

            foreach (Connection c in connections)
            {
                c.Dispose();
            }
        }

        public void Shutdown()
        {
            cts.Cancel();

            if (listener != null)
            {
                listener.Stop();
                listener = null;
            }

            Clear();
        }

        private async Task StartReadHeader(Connection connection)
        {
            try
            {
                var headerFields = await connection.ReadHeader(cancel);

                if (headerFields.ContainsKey("topic"))
                {
                    TransportSubscriberLink subscriberLink = new TransportSubscriberLink();
                    subscriberLink.Initialize(connection, new Header(headerFields));
                }
                else if (headerFields.ContainsKey("service"))
                {
                    IServiceClientLink serviceClientLink = new IServiceClientLink();
                    serviceClientLink.Initialize(connection, new Header(headerFields));
                }
                else
                {
                    throw new RosException("Received a connection for a type other than topic or service.");
                }
            }
            catch (Exception e)
            {
                logger.LogWarning(e, e.Message);
                connection.Dispose();
            }
        }

        public async Task CheckAndAccept()
        {
            while (listener != null)
            {
                cancel.ThrowIfCancellationRequested();

                var tcpClient = await listener.AcceptTcpClientAsync();
                var connection = new Connection(tcpClient);
                AddConnection(connection);
                var t = StartReadHeader(connection);
            }
        }

        public void Start()
        {
            listener = new TcpListener(IPAddress.Any, Network.TcpRosServerPort);
            listener.Start(16);
            acceptLoop = CheckAndAccept();
        }

        private void Connection_Disposed(Connection connection)
        {
            lock (gate)
            {
                this.connections.Remove(connection);
            }
        }
    }
}
