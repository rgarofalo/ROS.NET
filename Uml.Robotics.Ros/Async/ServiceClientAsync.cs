using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    public class ServiceClientAsync<MReq, MRes>
        : ServiceClientAsyncBase
        where MReq : RosMessage, new() where MRes : RosMessage, new()
    {
        internal ServiceClientAsync(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
            if (persistent)
            {
                serverLink = CreateLink();
            }
        }

        protected override IServiceServerLink CreateLink()
        {
            return ServiceManager.Instance.CreateServiceServerLinkAsync<MReq, MRes>(serviceName, persistent, md5sum, md5sum, headerValues);
        }

        public async Task<(bool, MRes)> Call(MReq request)
        {
            string md5 = request.MD5Sum();
            return await Call(request, md5);
        }

        public async Task<(bool, MRes)> Call(MReq request, string serviceMd5Sum)
        {
            if (!PreCall(serviceMd5Sum) || serverLink == null)
            {
                Dispose();
                return (false, null);
            }

            var serviceServerLink = serverLink as ServiceServerLinkAsync;
            (bool result, RosMessage response) = await serviceServerLink.Call(request);

            var responseMessage = (MRes)response;
            return (result, responseMessage);
        }
    }

    public class ServiceClientAsync<MSrv>
        : ServiceClientAsyncBase
        where MSrv : RosService, new()
    {
        internal ServiceClientAsync(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
            if (persistent)
            {
                serverLink = CreateLink();
            }
        }

        protected override IServiceServerLink CreateLink()
        {
            return ServiceManager.Instance.CreateServiceServerLinkAsync<MSrv>(serviceName, persistent, md5sum, md5sum, headerValues);
        }

        public async Task<bool> Call(MSrv srv)
        {
            string md5 = srv.RequestMessage.MD5Sum();
            return await Call(srv, md5);
        }

        public async Task<bool> Call(MSrv srv, string serviceMd5Sum)
        {
            if (!PreCall(serviceMd5Sum) || serverLink == null)
            {
                Dispose();
                return false;
            }

            var serviceServerLink = serverLink as ServiceServerLinkAsync;
            bool result = await serviceServerLink.Call(srv);
            return result;
        }
    }
}
