using System.Collections.Generic;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    public delegate bool ServiceFunction<in MReq, MRes>(MReq req, ref MRes res)
        where MReq : RosMessage, new()
        where MRes : RosMessage, new();

    public class ServiceCallbackHelperParams<MReq, MRes>
        : IServiceCallbackHelperParams
        where MReq : RosMessage
        where MRes : RosMessage
    {
        public IDictionary<string, string> ConnectionHeader { get; set; } = new Dictionary<string, string>();
        public MReq Request { get; set; }
        public MRes Response { get; set; }

        RosMessage IServiceCallbackHelperParams.Request => Request;
        RosMessage IServiceCallbackHelperParams.Response => Response;
    }

    public interface IServiceCallbackHelperParams
    {
        IDictionary<string, string> ConnectionHeader { get; set; }
        RosMessage Request { get; }
        RosMessage Response { get; }
    }

    public class ServiceCallbackHelper<MReq, MRes> : IServiceCallbackHelper
        where MReq : RosMessage, new()
        where MRes : RosMessage, new()
    {
        protected new ServiceFunction<MReq, MRes> callback;

        public ServiceCallbackHelper(ServiceFunction<MReq, MRes> srv_func)
        {
            callback = srv_func;
        }

        internal bool Call(ServiceCallbackHelperParams<MReq, MRes> parms)
        {
            MRes response = parms.Response;
            bool result = callback.Invoke(parms.Request, ref response);
            parms.Response = response;
            return result;
        }
    }

    public class IServiceCallbackHelper
    {
        private ILogger Logger { get; } = ApplicationLogging.CreateLogger<IServiceCallbackHelper>();
        protected ServiceFunction<RosMessage, RosMessage> callback;

        public string type;

        protected IServiceCallbackHelper()
        {
        }

        protected IServiceCallbackHelper(ServiceFunction<RosMessage, RosMessage> callback)
        {
            this.callback = callback;
        }

        public virtual ServiceFunction<RosMessage, RosMessage> Callback()
        {
            return callback;
        }

        public virtual ServiceFunction<RosMessage, RosMessage> Callback(ServiceFunction<RosMessage, RosMessage> cb)
        {
            callback = cb;
            return callback;
        }

        public virtual MReq Deserialize<MReq, MRes>(ServiceCallbackHelperParams<MReq, MRes> parms)
            where MReq : RosMessage
            where MRes : RosMessage
        {
            RosMessage msg = RosMessage.Generate(type);
            msg.connection_header = new Dictionary<string, string>(parms.ConnectionHeader);
            MReq t = (MReq) msg;
            t.Deserialize(parms.Response.Serialized);
            return t;
        }
    }
}
