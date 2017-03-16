﻿using System;
using System.Diagnostics;
using Messages;
using m = Messages.std_msgs;
using gm = Messages.geometry_msgs;
using nm = Messages.nav_msgs;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    public class SubscriptionCallbackHelper<M> : ISubscriptionCallbackHelper where M : RosMessage, new()
    {
        public SubscriptionCallbackHelper(MsgTypes t, CallbackDelegate<M> cb) : this(new Callback<M>(cb))
        {
            type = t;
        }

        public SubscriptionCallbackHelper(MsgTypes t)
        {
            type = t;
        }

        public SubscriptionCallbackHelper(CallbackInterface q)
            : base(q)
        {
        }

        public override void call(RosMessage msg)
        {
            Callback.func(msg);
        }
    }

    public class ISubscriptionCallbackHelper
    {
        private ILogger Logger { get; } = ApplicationLogging.CreateLogger<ISubscriptionCallbackHelper>();
        public CallbackInterface Callback { protected set; get; }

        public MsgTypes type;

        protected ISubscriptionCallbackHelper()
        {
            // Logger.LogDebug("ISubscriptionCallbackHelper: 0 arg constructor");
        }

        protected ISubscriptionCallbackHelper(CallbackInterface Callback)
        {
            //Logger.LogDebug("ISubscriptionCallbackHelper: 1 arg constructor");
            //throw new NotImplementedException();
            this.Callback = Callback;
        }

        public virtual void call(RosMessage parms)
        {
            // Logger.LogDebug("ISubscriptionCallbackHelper: call");
            throw new NotImplementedException();
        }
    }
}