using Messages.rosgraph_msgs;
using System;
using System.Threading;
using System.Threading.Tasks;
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

        private AsyncQueue<Log> queue = new AsyncQueue<Log>(10000, true);
        private Task publishLoopTask;
        private TopicManager topicManager;

        public RosOutAppender()
        {
            topicManager = TopicManager.Instance;
        }

        public void Dispose()
        {
            queue.OnCompleted();
            publishLoopTask.WhenCompleted().Wait();
        }

        public bool Started
        {
            get
            {
                lock (queue)
                {
                    return publishLoopTask != null;
                }
            }
        }

        public void Start()
        {
            lock (queue)
            {
                if (queue.IsCompleted || publishLoopTask != null)
                    return;

                publishLoopTask = PublishLoopAsync();
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
            logMessage.topics = topicManager.GetAdvertisedTopics();
            queue.TryOnNext(logMessage);
        }

        private async Task PublishLoopAsync()
        {
            using (var publisher = await ROS.GlobalNodeHandle.AdvertiseAsync<Log>("/rosout", 0))
            {
                while (!await queue.MoveNext(default(CancellationToken)))
                {
                    Log entry = queue.Current;
                    publisher.publish(entry);
                }
            }
        }
    }
}
