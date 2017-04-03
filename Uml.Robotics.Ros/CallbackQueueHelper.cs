﻿using System;
using System.Collections.Generic;
using System.Text;

namespace Uml.Robotics.Ros
{
    public class CallbackInfo
    {
        public CallbackInterface Callback { get; set; }
        public bool MarkedForRemoval { get; set; }
        public UInt64 RemovalId { get; set; }
    }


    public class TLS
    {
        private volatile List<CallbackInfo> _queue = new List<CallbackInfo>();
        public UInt64 calling_in_this_thread = 0xffffffffffffffff;

        public int Count
        {
            get
            {
                lock (_queue)
                {
                    return _queue.Count;
                }
            }
        }

        public CallbackInfo Head
        {
            get
            {
                lock (_queue)
                {
                    if (_queue.Count == 0) return null;
                    return _queue[0];
                }
            }
        }

        public CallbackInfo Tail
        {
            get
            {
                lock (_queue)
                {
                    if (_queue.Count == 0) return null;
                    return _queue[_queue.Count - 1];
                }
            }
        }

        public CallbackInfo Dequeue()
        {
            CallbackInfo tmp;
            lock (_queue)
            {
                if (_queue.Count == 0) return null;
                tmp = _queue[0];
                _queue.RemoveAt(0);
            }
            return tmp;
        }

        public void Enqueue(CallbackInfo info)
        {
            if (info.Callback == null)
                return;
            lock (_queue)
                _queue.Add(info);
        }

        public CallbackInfo SpliceOut(CallbackInfo info)
        {
            lock (_queue)
            {
                if (!_queue.Contains(info))
                    return null;
                _queue.RemoveAt(_queue.IndexOf(info));
                return info;
            }
        }
    }


    public class IDInfo
    {
        public object calling_rw_mutex;
        public UInt64 id;
    }


    public enum CallOneResult
    {
        Called,
        TryAgain,
        Disabled,
        Empty
    }
}
