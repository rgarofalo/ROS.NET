using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    public class ServiceClient<MReq, MRes>
        : ServiceClientBase
        where MReq : RosMessage, new() where MRes : RosMessage, new()
    {
        internal ServiceClient(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
            if (persistent)
            {
                this.Init().WhenCompleted().Wait();
            }
        }

        protected override async Task<IServiceServerLinkAsync> CreateLink()
        {
            return await ServiceManager.Instance.CreateServiceServerLinkAsync<MReq, MRes>(serviceName, persistent, md5sum, md5sum, headerValues);
        }

        public (bool, MRes) Call(MReq request) =>
            CallAsync(request).Result;

        public async Task<(bool, MRes)> CallAsync(MReq request)
        {
            string md5 = request.MD5Sum();
            return await Call(request, md5);
        }

        public async Task<(bool, MRes)> Call(MReq request, string serviceMd5Sum)
        {
            try
            {
                EnterCall();

                if (!await PreCall(serviceMd5Sum) || serverLink == null || !serverLink.IsValid)
                {
                    return (false, null);
                }

                (bool result, RosMessage response) = await serverLink.Call(request);

                var responseMessage = (MRes)response;
                return (result, responseMessage);
            }
            finally
            {
                ExitCall();
            }
        }
    }

    public class ServiceClient<MSrv>
        : ServiceClientBase
        where MSrv : RosService, new()
    {
        internal ServiceClient(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
            if (persistent)
            {
                this.Init().WhenCompleted().Wait();
            }
        }

        protected override Task<IServiceServerLinkAsync> CreateLink()
        {
            return ServiceManager.Instance.CreateServiceServerLinkAsync<MSrv>(serviceName, persistent, md5sum, md5sum, headerValues);
        }

        public bool Call(MSrv srv) =>
           CallAsync(srv).Result;

        public async Task<bool> CallAsync(MSrv srv)
        {
            string md5 = srv.RequestMessage.MD5Sum();
            return await CallAsync(srv, md5);
        }

        public async Task<bool> CallAsync(MSrv srv, string serviceMd5Sum)
        {
            try
            {
                EnterCall();

                if (!await PreCall(serviceMd5Sum) || serverLink == null || !serverLink.IsValid)
                {
                    return false;
                }

                bool result = await serverLink.Call(srv);
                return result;
            }
            finally
            {
                ExitCall();
            }
        }
    }
}
