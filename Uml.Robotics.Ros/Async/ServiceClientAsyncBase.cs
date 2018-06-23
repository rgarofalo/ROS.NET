﻿using Microsoft.Extensions.Logging;
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

        protected object gate = new object();
        protected ILogger logger = ApplicationLogging.CreateLogger<ServiceClientAsyncBase>();
        protected string serviceName;
        protected bool persistent;
        protected IDictionary<string, string> headerValues;
        protected string md5sum;
        protected IServiceServerLinkAsync serverLink;
        protected bool busy;

        public ServiceClientAsyncBase(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
        {
            this.serviceName = serviceName;
            this.persistent = persistent;
            this.headerValues = headerValues;
            this.md5sum = md5sum;
        }

        public async Task Init()
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

        protected async Task<bool> PreCall(string service_md5sum)
        {
            if (disposed)
                throw new ObjectDisposedException("ServiceClient instance was disposed");

            if (service_md5sum != md5sum)
            {
                throw new Exception($"Call to service '{serviceName}' with md5sum '{service_md5sum}' does not match the md5sum that was specified when the handle was created ('{md5sum}').");
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
