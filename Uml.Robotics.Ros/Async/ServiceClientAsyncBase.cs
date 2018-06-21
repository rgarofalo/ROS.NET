using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    public abstract class ServiceClientAsyncBase
        : IDisposable
    {
        private bool disposed;

        protected ILogger Logger { get; } = ApplicationLogging.CreateLogger<ServiceClientAsyncBase>();
        protected string serviceName;
        protected bool persistent;
        protected IDictionary<string, string> headerValues;
        protected string md5sum;
        protected IServiceServerLink serverLink;

        public ServiceClientAsyncBase(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
        {
            this.serviceName = serviceName;
            this.persistent = persistent;
            this.headerValues = headerValues;
            this.md5sum = md5sum;
        }

        protected abstract IServiceServerLink CreateLink();

        public void Dispose()
        {
            if (!disposed)
                return;
            disposed = true;

            if (!persistent && serverLink != null)
            {
                ServiceManager.Instance.RemoveServiceServerLink(serverLink);
                serverLink = null;
            }
        }

        public bool IsValid => !persistent || (!disposed && (serverLink?.IsValid ?? false));
        public string ServiceName => serviceName;

        protected bool PreCall(string service_md5sum)
        {
            if (service_md5sum != md5sum)
            {
                Logger.LogError("Call to service [{0} with md5sum [{1} does not match md5sum when the handle was created([{2}])", serviceName, service_md5sum, md5sum);
                return false;
            }

            if (serverLink != null && serverLink.connection.dropped)
            {
                if (persistent)
                    Logger.LogWarning("Persistent service client's server link has been dropped. Trying to reconnect to proceed with this call");
                serverLink = null;
            }

            if (disposed && persistent)
                Logger.LogWarning("Persistent service client is self-resurrecting");

            disposed = false;
            if (persistent && serverLink == null || !persistent)
            {
                serverLink = CreateLink();
            }

            return true;
        }
    }
}
