using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using Messages.rosgraph_msgs;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class CallerInfo
    {
        public string MemberName { get; set; }
        public string FilePath { get; set; }
        public int LineNumber { get; set; }
    }

    public class RosOutAppender
        : IDisposable
    {
        internal enum ROSOUT_LEVEL
        {
            DEBUG = 1,
            INFO = 2,
            WARN = 4,
            ERROR = 8,
            FATAL = 16
        }

        private static Lazy<RosOutAppender> instance = new Lazy<RosOutAppender>(LazyThreadSafetyMode.ExecutionAndPublication);

        public static RosOutAppender Instance =>
            instance.Value;

        internal static void Terminate() =>
            Instance.Dispose();

        internal static void Reset() =>
            instance = new Lazy<RosOutAppender>(LazyThreadSafetyMode.ExecutionAndPublication);

        private AsyncQueue<Log> queue = new AsyncQueue<Log>(10000);
        private Task publishLoop;
        private Publisher<Log> publisher;

        public RosOutAppender()
        {
        }

        public void Dispose()
        {
            queue.OnCompleted();
            publishLoop.Wait();
            if (publisher != null)
            {
                publisher.shutdown();
                publisher = null;
            }
        }

        public bool Started
        {
            get
            {
                lock (queue)
                {
                    return publishLoop != null && !publishLoop.IsCompleted && !queue.IsCompleted;
                }
            }
        }

        public void Start()
        {
            lock (queue)
            {
                if (!queue.IsCompleted && publishLoop == null)
                {
                    if (publisher == null)
                        publisher = ROS.GlobalNodeHandle.Advertise<Log>("/rosout", 0);
                    publishLoop = PublishLoopAsync();
                }
            }
        }

        internal void Append(string message, ROSOUT_LEVEL level, CallerInfo callerInfo)
        {
            var logMessage = new Log
            {
                msg = message,
                name = ThisNode.Name,
                file = callerInfo.FilePath,
                function = callerInfo.MemberName,
                line = (uint)callerInfo.LineNumber,
                level = (byte)level,
                header = new Messages.std_msgs.Header { stamp = ROS.GetTime() }
            };
            TopicManager.Instance.getAdvertisedTopics(out logMessage.topics);
            queue.TryOnNext(logMessage);
        }

        private async Task PublishLoopAsync()
        {
            while (!await queue.MoveNext(default(CancellationToken)))
            {
                Log entry = queue.Current;
                publisher.publish(entry);
            }
        }
    }
}
