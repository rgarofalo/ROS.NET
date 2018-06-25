using System;
using System.Collections.Generic;
using System.Threading;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    internal class Callback
        : CallbackInterface
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<Callback>();
        private volatile bool callback_state;

        private readonly bool allow_concurrent_callbacks;
        private readonly Queue<Item> queue = new Queue<Item>();
        private int queueSize;

        public static Callback Create<M>(CallbackDelegate<M> f) where M : RosMessage, new()
        {
            return new Callback(msg => f(msg as M));
        }

        public Callback(CallbackDelegate f, string topic, int queueSize, bool allowConcurrentCallbacks)
            : this(f)
        {
            this.allow_concurrent_callbacks = allowConcurrentCallbacks;
            this.queueSize = queueSize;
        }

        public Callback(CallbackDelegate f)
        {
            base.Event += f;
        }

        public override void AddToCallbackQueue(ISubscriptionCallbackHelper helper, RosMessage message, bool nonconst_need_copy, ref bool was_full, TimeData receipt_time)
        {
            if (was_full)
                was_full = false;

            var i = new Item
            {
                helper = helper,
                message = message,
                nonconst_need_copy = nonconst_need_copy,
                receiptTime = receipt_time
            };

            lock (queue)
            {
                if (this.IsFullNoLock)
                {
                    queue.Dequeue();
                    was_full = true;
                }
                queue.Enqueue(i);
            }
        }

        public override void Clear()
        {
            queue.Clear();
        }

        public virtual bool ready()
        {
            return true;
        }

        private bool IsFullNoLock =>
            queueSize > 0 && queue.Count >= queueSize;

        public bool IsFull
        {
            get
            {
                lock (queue)
                {
                    return this.IsFullNoLock;
                }
            }
        }

        public class Item
        {
            public ISubscriptionCallbackHelper helper;
            public RosMessage message;
            public bool nonconst_need_copy;
            public TimeData receiptTime;
        }

        internal override CallResult Call()
        {
            if (!allow_concurrent_callbacks)
            {
                if (callback_state)
                    return CallResult.TryAgain;
                callback_state = true;
            }
            Item i = null;
            lock (queue)
            {
                if (queue.Count == 0)
                    return CallResult.Invalid;
                i = queue.Dequeue();
            }
            i.helper.call(i.message);
            callback_state = false;
            return CallResult.Success;
        }
    }


    public abstract class CallbackInterface
    {
        private static long nextId = 0;
        private static long NewUniqueId() =>
            Interlocked.Increment(ref nextId);

        public long Uid { get; }
        public delegate void CallbackDelegate(RosMessage msg);
        public event CallbackDelegate Event;

        private readonly ILogger logger = ApplicationLogging.CreateLogger<CallbackInterface>();

        public CallbackInterface()
        {
            this.Uid = NewUniqueId();
        }

        public CallbackInterface(CallbackDelegate f)
            : this()
        {
            Event += f;
        }

        public enum CallResult
        {
            Success,
            TryAgain,
            Invalid
        }

        public void SendEvent(RosMessage msg)
        {
            if (Event != null)
            {
                Event(msg);
            }
            else
            {
                logger.LogError($"{nameof(Event)} is null");
            }
        }

        public abstract void AddToCallbackQueue(ISubscriptionCallbackHelper helper, RosMessage msg, bool nonconst_need_copy, ref bool was_full, TimeData receipt_time);
        public abstract void Clear();
        internal abstract CallResult Call();
    }
}
