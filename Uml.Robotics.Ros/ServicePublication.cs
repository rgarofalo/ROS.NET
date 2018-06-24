using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    public class ServicePublication<MReq, MRes> : IServicePublication
        where MReq : RosMessage, new()
        where MRes : RosMessage, new()
    {
        public ServiceCallbackHelper<MReq, MRes> helper;

        public ServicePublication(string name, string md5Sum, string datatype, string reqDatatype, string resDatatype, ServiceCallbackHelper<MReq, MRes> helper, ICallbackQueue callback, object trackedObject)
        {
            if (string.IsNullOrWhiteSpace(name))
                throw new ArgumentNullException(nameof(name));

            this.name = name;
            this.md5sum = md5Sum;
            this.datatype = datatype;
            this.req_datatype = reqDatatype;
            this.res_datatype = resDatatype;
            this.helper = helper;
            this.callback = callback;
            this.tracked_object = trackedObject;

            if (trackedObject != null)
                has_tracked_object = true;
        }

        public override Task ProcessRequest(byte[] buf, int num_bytes, IServiceClientLink link)
        {
            var cb = new ServiceCallback(this, helper, buf, num_bytes, link, has_tracked_object, tracked_object);
            this.callbackId = cb.Uid;
            callback.AddCallback(cb);
        }

        internal override void AddServiceClientLink(IServiceClientLink iServiceClientLink)
        {
            lock (gate)
            {
                clientLinks.Add(iServiceClientLink);
            }
        }

        internal override void RemoveServiceClientLink(IServiceClientLink iServiceClientLink)
        {
            lock (gate)
            {
                clientLinks.Remove(iServiceClientLink);
            }
        }

        public class ServiceCallback : CallbackInterface
        {
            private ILogger Logger { get; } = ApplicationLogging.CreateLogger<ServiceCallback>();
            private bool _hasTrackedObject;
            private int _numBytes;
            private object _trackedObject;
            private byte[] buffer;
            private ServicePublication<MReq, MRes> isp;
            private IServiceClientLink link;

            public ServiceCallback(ServiceCallbackHelper<MReq, MRes> _helper, byte[] buf, int num_bytes, IServiceClientLink link, bool has_tracked_object, object tracked_object)
                : this(null, _helper, buf, num_bytes, link, has_tracked_object, tracked_object)
            {
            }

            public ServiceCallback(ServicePublication<MReq, MRes> sp, ServiceCallbackHelper<MReq, MRes> _helper, byte[] buf, int num_bytes, IServiceClientLink link, bool has_tracked_object, object tracked_object)
            {
                this.isp = sp;
                if (this.isp != null && _helper != null)
                    this.isp.helper = _helper;
                this.buffer = buf;
                this._numBytes = num_bytes;
                this.link = link;
                this._hasTrackedObject = has_tracked_object;
                this._trackedObject = tracked_object;
            }

            internal override CallResult Call()
            {
                if (!link.Connection.IsValid)
                {
                    return CallResult.Invalid;
                }

                ServiceCallbackHelperParams<MReq, MRes> parms = new ServiceCallbackHelperParams<MReq, MRes>
                {
                    Request = new MReq(),
                    Response = new MRes(),
                    ConnectionHeader = link.Connection.Header.Values
                };
                parms.Request.Deserialize(buffer);

                try
                {
                    bool ok = isp.helper.Call(parms);
                    link.ProcessResponse(parms.Response, ok);
                }
                catch (Exception e)
                {
                    string str = "Exception thrown while processing service call: " + e;
                    ROS.Error()(str);
                    link.ProcessResponse(str, false);
                    return CallResult.Invalid;
                }
                return CallResult.Success;
            }

            public override void AddToCallbackQueue(ISubscriptionCallbackHelper helper, RosMessage msg, bool nonconst_need_copy, ref bool was_full, TimeData receipt_time)
            {
                throw new NotImplementedException();
            }

            public override void Clear()
            {
                throw new NotImplementedException();
            }
        }
    }

    public abstract class IServicePublication
    {
        internal ICallbackQueue callback;
        internal List<IServiceClientLink> clientLinks = new List<IServiceClientLink>();
        protected object gate = new object();
        protected long callbackId = -1;
        internal string datatype;
        internal bool has_tracked_object;
        internal bool isDropped;
        internal string md5sum;
        internal string name;
        internal string req_datatype;
        internal string res_datatype;
        internal object tracked_object;

        internal void Drop()
        {
            lock (gate)
            {
                isDropped = true;
            }
            DropAllConnections();
            if (callbackId >= 0)
            {
                callback.RemoveById(callbackId);
            }
        }

        private void DropAllConnections()
        {
            List<IServiceClientLink> links;
            lock (gate)
            {
                links = new List<IServiceClientLink>(clientLinks);
                clientLinks.Clear();
            }

            foreach (IServiceClientLink iscl in links)
            {
                iscl.Connection.Dispose();
            }
        }

        internal abstract void AddServiceClientLink(IServiceClientLink iServiceClientLink);
        internal abstract void RemoveServiceClientLink(IServiceClientLink iServiceClientLink);
        public abstract Task ProcessRequest(byte[] buffer, int size, IServiceClientLink iServiceClientLink);
    }
}
