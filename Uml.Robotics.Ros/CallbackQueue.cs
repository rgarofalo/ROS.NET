using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Threading;

namespace Uml.Robotics.Ros
{
    public class CallbackQueue : ICallbackQueue
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<CallbackQueue>();
        private readonly object gate = new object();

        private int count;
        private int calling;
        private bool enabled;
        private Dictionary<long, IDInfo> idInfo = new Dictionary<long, IDInfo>();
        private AutoResetEvent sem = new AutoResetEvent(false);
        private List<CallbackInfo> callbacks = new List<CallbackInfo>();
        private TLS tls;

        public CallbackQueue()
        {
            enabled = true;
        }

        public bool IsEmpty
        {
            get { return count == 0; }
        }

        public bool IsEnabled
        {
            get { return enabled; }
        }

        public void AddCallback(CallbackInterface cb, long ownerId)
        {
            lock (gate)
            {
                if (!enabled)
                    return;

                CallbackInfo info = new CallbackInfo { Callback = cb, RemovalId = ownerId };
                callbacks.Add(info);
                count++;

                if (!idInfo.ContainsKey(ownerId))
                {
                    idInfo.Add(ownerId, new IDInfo { calling_rw_mutex = new object(), id = ownerId });
                }
            }

            NotifyOne();
        }

        public void CallAvailable(int timeout = ROS.WallDuration)
        {
            SetupTls();
            int called = 0;
            lock (gate)
            {
                if (!enabled)
                    return;
            }

            if (count == 0 && timeout != 0)
            {
                if (!sem.WaitOne(timeout))
                    return;
            }

            lock (gate)
            {
                if (count == 0)
                    return;
                if (!enabled)
                    return;
                callbacks.ForEach(cbi => tls.Enqueue(cbi));
                callbacks.Clear();
                count = 0;
                calling += tls.Count;
            }

            while (tls.Count > 0 && ROS.OK)
            {
                if (CallOne(tls) != CallOneResult.Empty)
                    ++called;
            }
            lock (gate)
            {
                calling -= called;
            }
            sem.Set();
        }

        public void Clear()
        {
            lock (gate)
            {
                callbacks.Clear();
                count = 0;
            }
        }

        public void Disable()
        {
            lock (gate)
            {
                enabled = false;
            }
            NotifyAll();
        }

        public void Dispose()
        {
            Disable();
        }

        public void Enable()
        {
            lock (gate)
            {
                enabled = true;
            }
            NotifyAll();
        }

        public void RemoveById(long ownerId)
        {
            SetupTls();
            lock (gate)
            {
                if (!idInfo.ContainsKey(ownerId))
                    return;
                RemoveAll(ownerId);
                idInfo.Remove(ownerId);
            }
        }

        private CallOneResult CallOne(TLS tls)
        {
            CallbackInfo info = tls.Head;
            if (info == null)
                return CallOneResult.Empty;

            if (TryGetIdInfo(info.RemovalId, out IDInfo idinfo))
            {
                CallbackInterface cb = info.Callback;
                lock (gate)
                {
                    CallbackInterface.CallResult result = CallbackInterface.CallResult.Invalid;
                    tls.SpliceOut(info);
                    if (!info.MarkedForRemoval)
                    {
                        try
                        {
                            result = cb.Call();
                        }
                        catch (Exception ex)
                        {
                            ROS.Error()("Error during callback. Error: %s, Stacktrace: %s", ex.ToString(), ex.StackTrace);
                        }
                    }
                    if (result == CallbackInterface.CallResult.TryAgain && !info.MarkedForRemoval)
                    {
                        lock (gate)
                        {
                            callbacks.Add(info);
                            count++;
                        }
                        return CallOneResult.TryAgain;
                    }
                }
                return CallOneResult.Called;
            }
            CallbackInfo cbi = tls.SpliceOut(info);
            if (cbi != null)
                cbi.Callback.Call();
            return CallOneResult.Called;
        }

        private void RemoveAll(long ownerId)
        {
            lock (gate)
            {
                callbacks.RemoveAll(ici => ici.RemovalId == ownerId);
                count = callbacks.Count;
            }
        }

        private void SetupTls()
        {
            if (tls == null)
            {
                tls = new TLS()
                {
                    calling_in_this_thread = Thread.CurrentThread.ManagedThreadId
                };
            }
        }

        private void NotifyAll()
        {
            sem.Set();
        }

        private void NotifyOne()
        {
            sem.Set();
        }

        private bool TryGetIdInfo(long id, out IDInfo value)
        {
            lock (gate)
            {
                return idInfo.TryGetValue(id, out value);
            }
        }
    }
}
