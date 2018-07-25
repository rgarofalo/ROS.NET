using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    /// <summary>
    /// A callback queue which runs a background thread for calling callbacks. It does not need a spinner. The queue is disabled
    /// by default. The background thread is created when the queue gets enabled.
    /// </summary>
    /*public class CallbackQueueThreaded : ICallbackQueue
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<CallbackQueueThreaded>();
        private readonly object gate = new object();

        private int count;
        private int calling;
        private Thread callbackThread;
        private bool enabled;
        private Dictionary<long, IDInfo> idInfo = new Dictionary<long, IDInfo>();
        private AutoResetEvent sem = new AutoResetEvent(false);
        private List<CallbackInfo> callbacks = new List<CallbackInfo>();
        private TLS tls;

        public bool IsEmpty
        {
            get { return count == 0; }
        }

        public bool IsEnabled
        {
            get { return enabled; }
        }

        private void SetupTls()
        {
            if (tls == null)
            {
                tls = new TLS
                {
                    calling_in_this_thread = Thread.CurrentThread.ManagedThreadId
                };
            }
        }

        private void NotifyAll()
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

        public void AddCallback(CallbackInterface cb, long ownerId)
        {
            var info = new CallbackInfo { Callback = cb, RemovalId = ownerId };

            lock (gate)
            {
                if (!enabled)
                    return;

                callbacks.Add(info);
                count++;

                if (!idInfo.ContainsKey(ownerId))
                {
                    idInfo.Add(ownerId, new IDInfo { calling_rw_mutex = new object(), id = ownerId });
                }
            }

            NotifyAll();
        }

        public void RemoveById(long ownerId)
        {
            SetupTls();

            IDInfo idinfo;
            lock (gate)
            {
                if (!idInfo.ContainsKey(ownerId))
                    return;
                idinfo = idInfo[ownerId];

                RemoveAll(ownerId);

                idInfo.Remove(ownerId);
            }
        }

        private void RemoveAll(long ownerId)
        {
            lock (gate)
            {
                callbacks.RemoveAll(ici => ici.RemovalId == ownerId);
                count = callbacks.Count;
            }
        }

        private void ThreadFunc()
        {
            TimeSpan wallDuration = new TimeSpan(0, 0, 0, 0, ROS.WallDuration);
            while (ROS.OK)
            {
                DateTime begin = DateTime.UtcNow;
                CallAvailable(ROS.WallDuration);
                DateTime end = DateTime.UtcNow;

                var remainingTime = wallDuration - (end - begin);
                if (remainingTime > TimeSpan.Zero)
                    Thread.Sleep(remainingTime);
            }
            logger.LogDebug("CallbackQueue thread broke out!");
        }

        public void Enable()
        {
            lock (gate)
            {
                enabled = true;
            }
            NotifyAll();
            if (callbackThread == null)
            {
                callbackThread = new Thread(ThreadFunc);
                callbackThread.Start();
            }
        }

        public void Disable()
        {
            lock (gate)
            {
                enabled = false;
            }
            NotifyAll();
            if (callbackThread != null)
            {
                callbackThread.Join();
                callbackThread = null;
            }
        }

        public void Clear()
        {
            lock (gate)
            {
                callbacks.Clear();
                count = 0;
            }
        }

        public CallOneResult CallOne(TLS tls)
        {
            CallbackInfo info = tls.Head;
            if (info == null)
                return CallOneResult.Empty;

            if (TryGetIdInfo(info.RemovalId, out IDInfo idinfo))
            {
                CallbackInterface cb = info.Callback;
                lock (idinfo.calling_rw_mutex)
                {
                    CallbackInterface.CallResult result = CallbackInterface.CallResult.Invalid;
                    tls.SpliceOut(info);
                    if (!info.MarkedForRemoval)
                    {
                        result = cb.Call();
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

        public void CallAvailable(int timeout)
        {
            SetupTls();

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
                if (!enabled || count == 0 )
                    return;

                callbacks.ForEach(cbi => tls.Enqueue(cbi));
                callbacks.Clear();
                count = 0;
                calling += tls.Count;
            }

            int called = 0;
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
    }*/
}
