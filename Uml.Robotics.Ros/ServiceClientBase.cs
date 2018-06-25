using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace Uml.Robotics.Ros
{
    public abstract class ServiceClientBase
        : IDisposable
    {
        private bool disposed;

        protected readonly object gate = new object();
        protected readonly ILogger logger = ApplicationLogging.CreateLogger<ServiceClientBase>();

        protected string serviceName;
        protected bool persistent;
        protected string md5sum;
        protected IDictionary<string, string> headerValues;

        protected IServiceServerLinkAsync serverLink;
        protected bool busy;

        public ServiceClientBase(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
        {
            this.serviceName = serviceName;
            this.persistent = persistent;
            this.headerValues = headerValues;
            this.md5sum = md5sum;
        }

        protected async Task Init()
        {
            if (persistent)
            {
                serverLink = await CreateLink();
            }
        }

        protected abstract Task<IServiceServerLinkAsync> CreateLink();

        public void Dispose()
        {
            if (disposed)
                return;
            disposed = true;

            if (serverLink != null)
            {
                ServiceManager.Instance.RemoveServiceServerLinkAsync(serverLink);
                serverLink.Dispose();
                serverLink = null;
            }
        }

        public bool IsValid => !persistent || (!disposed && (serverLink?.IsValid ?? false));
        public string ServiceName => serviceName;

        protected void EnterCall()
        {
            lock (gate)
            {
                if (busy)
                {
                    throw new Exception("Concurrent calls on a service client are not allowed.");
                }

                busy = true;
            }
        }

        protected void ExitCall()
        {
            lock (gate)
            {
                busy = false;
            }
        }

        protected async Task<bool> PreCall(string serviceMd5Sum)
        {
            if (disposed)
                throw new ObjectDisposedException("ServiceClient instance was disposed");

            if (serviceMd5Sum != md5sum)
            {
                throw new Exception($"Call to service '{serviceName}' with md5sum '{serviceMd5Sum}' does not match the md5sum that was specified when the handle was created ('{md5sum}').");
            }

            if (serverLink != null && !serverLink.IsValid)
            {
                if (persistent)
                {
                    logger.LogWarning("Persistent service client's server link has been dropped. Trying to reconnect to proceed with this call.");
                }
                ServiceManager.Instance.RemoveServiceServerLinkAsync(serverLink);
                serverLink.Dispose();
                serverLink = null;
            }

            if (serverLink == null)
            {
                serverLink = await CreateLink();
            }

            return true;
        }
    }
}
