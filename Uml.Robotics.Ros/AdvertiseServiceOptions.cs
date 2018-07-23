using System;
using Messages;

namespace Uml.Robotics.Ros
{
    public class AdvertiseServiceOptions<MReq, MRes>
        where MReq : RosMessage, new()
        where MRes : RosMessage, new()
    {
        public ICallbackQueue CallbackQueue;
        public string DataType => SrvType;
        public ServiceCallbackHelper<MReq, MRes> Helper;
        public string Md5Sum;
        public int QueueSize;
        public string RequestDataType;
        public string ResponseDataType;
        public string Service = "";
        public ServiceFunction<MReq, MRes> ServiceCallback;
        public string SrvType;

        public AdvertiseServiceOptions(string service, ServiceFunction<MReq, MRes> serviceCallback)
        {
            this.Service = service;
            ServiceCallback = serviceCallback;
            Helper = new ServiceCallbackHelper<MReq, MRes>(serviceCallback);
            RequestDataType = new MReq().MessageType.Replace("/Request", "__Request");
            ResponseDataType = new MRes().MessageType.Replace("/Response", "__Response");
            SrvType = RequestDataType.Replace("__Request", "");
            Md5Sum = RosService.Generate(SrvType).MD5Sum();
        }
    }
}
