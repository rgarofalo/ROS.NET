using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace Uml.Robotics.Ros
{
    public class ServiceClientAsync<MReq, MRes>
        : ServiceClientBase
        where MReq : RosMessage, new() where MRes : RosMessage, new()
    {
        internal ServiceClientAsync(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
        }

        protected override async Task<IServiceServerLinkAsync> CreateLink()
        {
            return await ServiceManager.Instance.CreateServiceServerLinkAsync<MReq, MRes>(serviceName, persistent, md5sum, md5sum, headerValues);
        }

        public async Task<(bool, MRes)> Call(MReq request)
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

    public class ServiceClientAsync<MSrv>
        : ServiceClientBase
        where MSrv : RosService, new()
    {
        internal ServiceClientAsync(string serviceName, bool persistent, IDictionary<string, string> headerValues, string md5sum)
            : base(serviceName, persistent, headerValues, md5sum)
        {
        }

        protected override Task<IServiceServerLinkAsync> CreateLink()
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
