using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Uml.Robotics.XmlRpc;

namespace Uml.Robotics.Ros
{
    public class ServiceManager
    {
        private static Lazy<ServiceManager> instance = new Lazy<ServiceManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        public static ServiceManager Instance => instance.Value;
        internal static void Terminate() => Instance.Shutdown();
        internal static void Reset() => instance = new Lazy<ServiceManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        private readonly ILogger logger = ApplicationLogging.CreateLogger<ServiceManager>();
        private readonly object gate = new object();
        private ConnectionManager connectionManager;
        private List<IServicePublication> servicePublications = new List<IServicePublication>();
        private HashSet<IServiceServerLinkAsync> serviceServerLinksAsync = new HashSet<IServiceServerLinkAsync>();
        private bool shuttingDown;
        private XmlRpcManager xmlRpcManager;

        public void Start()
        {
            connectionManager = ConnectionManager.Instance;
            xmlRpcManager = XmlRpcManager.Instance;
        }

        internal IServicePublication LookupServicePublication(string name)
        {
            lock (gate)
            {
                return servicePublications.FirstOrDefault(x => x.name == name);
            }
        }

        internal async Task<(string host, int port)> LookupServiceAsync(string name)
        {
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            args.Set(1, name);

            if (!await Master.ExecuteAsync("lookupService", args, result, payload, false))
            {
                throw new Exception($"The Service '{name}' is not available at ROS master.");
            }

            string uri = payload.GetString();
            if (string.IsNullOrWhiteSpace(uri))
            {
                throw new Exception("An Empty server URI was returned from ROS master.");
            }

            if (!Network.SplitUri(uri, out string host, out int port))
            {
                throw new Exception($"Bad service URI received: '{uri}]");
            }

            return (host, port);
        }

        private async Task<IServiceServerLinkAsync> CreateServiceServerLinkAsync(
            string service,
            bool persistent,
            string requestMd5Sum,
            string responseMd5Sum,
            IDictionary<string, string> headerValues,
            Action<ServiceServerLink> initialize
        )
        {
            (string host, int port) = await LookupServiceAsync(service);

            var client = new TcpClient();
            await client.ConnectAsync(host, port);
            client.NoDelay = true;

            var connection = new Connection(client);
            var link = new ServiceServerLink(connection, service, persistent, requestMd5Sum, responseMd5Sum, headerValues);
            initialize(link);

            lock (gate)
            {
                serviceServerLinksAsync.Add(link);
            }

            return link;
        }

        internal async Task<IServiceServerLinkAsync> CreateServiceServerLinkAsync<S>(
            string service,
            bool persistent,
            string requestMd5Sum,
            string responseMd5Sum,
            IDictionary<string, string> headerValues
        )
            where S : RosService, new()
        {
            return await CreateServiceServerLinkAsync(service, persistent, requestMd5Sum, responseMd5Sum, headerValues, link => link.Initialize<S>());
        }

        internal async Task<IServiceServerLinkAsync> CreateServiceServerLinkAsync<Req, Res>(
            string service,
            bool persistent,
            string requestMd5Sum,
            string responseMd5Sum,
            IDictionary<string, string> headerValues
        )
            where Req : RosMessage, new()
            where Res : RosMessage, new()
        {
            return await CreateServiceServerLinkAsync(service, persistent, requestMd5Sum, responseMd5Sum, headerValues, link => link.Initialize<Req, Res>());
        }

        internal void RemoveServiceServerLinkAsync(IServiceServerLinkAsync link)
        {
            lock (gate)
            {
                if (shuttingDown)
                    return;

                serviceServerLinksAsync.Remove(link);
            }
        }

        internal bool AdvertiseService<MReq, MRes>(AdvertiseServiceOptions<MReq, MRes> ops) where MReq : RosMessage, new() where MRes : RosMessage, new()
        {
            lock (gate)
            {
                if (shuttingDown)
                    return false;

                if (IsServiceAdvertised(ops.service))
                {
                    logger.LogWarning($"Tried to advertise a service that is already advertised in this node [{ops.service}].");
                    return false;
                }
                if (ops.helper == null)
                    ops.helper = new ServiceCallbackHelper<MReq, MRes>(ops.srv_func);
                ServicePublication<MReq, MRes> pub = new ServicePublication<MReq, MRes>(ops.service, ops.md5sum, ops.datatype, ops.req_datatype, ops.res_datatype, ops.helper, ops.callback_queue);
                servicePublications.Add(pub);
            }

            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            args.Set(1, ops.service);
            args.Set(2, string.Format("rosrpc://{0}:{1}", Network.Host, connectionManager.TCPPort));
            args.Set(3, xmlRpcManager.Uri);
            if (!Master.Execute("registerService", args, result, payload, true))
            {
                throw new RosException($"RPC \"registerService\" for service '{ops.service}' failed.");
            }

            return true;
        }

        internal bool UnadvertiseService(string service)
        {
            IServicePublication pub = null;
            lock (gate)
            {
                if (shuttingDown)
                    return false;

                foreach (IServicePublication sp in servicePublications)
                {
                    if (sp.name == service)
                    {
                        pub = sp;
                        servicePublications.Remove(sp);
                        break;
                    }
                }
            }

            if (pub != null)
            {
                UnregisterService(pub.name);
                pub.Drop();
                return true;
            }
            return false;
        }

        internal void Shutdown()
        {
            List<IServiceServerLinkAsync> localAsyncServiceClients;
            lock (gate)
            {
                if (shuttingDown)
                    return;
                shuttingDown = true;

                foreach (IServicePublication sp in servicePublications)
                {
                    UnregisterService(sp.name);
                    sp.Drop();
                }
                servicePublications.Clear();

                localAsyncServiceClients = serviceServerLinksAsync.ToList();
                serviceServerLinksAsync.Clear();
            }

            foreach (IServiceServerLinkAsync link in localAsyncServiceClients)
            {
                link.Dispose();
            }
        }

        internal bool LookupService(string name, ref string serviceHost, ref int servicePort)
        {
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            args.Set(1, name);
            if (!Master.Execute("lookupService", args, result, payload, false))
            {
                logger.LogWarning("Service [{0}]: Not available at ROS master", name);
                return false;
            }
            string serviceUri = payload.GetString();
            if (serviceUri.Length == 0)
            {
                logger.LogError("Service [{0}]: Empty server URI returned from master", name);
                return false;
            }
            if (!Network.SplitUri(serviceUri, out serviceHost, out servicePort))
            {
                logger.LogError("Service [{0}]: Bad service uri [{0}]", name, serviceUri);
                return false;
            }
            return true;
        }

        internal bool LookUpService(string mappedName, ref string host, ref int port)
        {
            return LookupService(mappedName, ref host, ref port);
        }

        private bool IsServiceAdvertised(string serviceName)
        {
            lock (gate)
            {
                return servicePublications.Any(s => s.name == serviceName && !s.isDropped);
            }
        }

        private bool UnregisterService(string service)
        {
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            args.Set(1, service);
            args.Set(2, string.Format("rosrpc://{0}:{1}", Network.Host, connectionManager.TCPPort));

            bool unregisterSuccess = false;

            try
            {
                unregisterSuccess = Master.Execute("unregisterService", args, result, payload, false);
            }
            catch
            {
                // ignore exception during unregister
            }

            return unregisterSuccess;
        }
    }
}
