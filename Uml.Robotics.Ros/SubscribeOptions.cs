namespace Uml.Robotics.Ros
{
    public class SubscribeOptions
    {
        public bool AllowConcurrentCallbacks = true;
        public ICallbackQueue CallbackQueue;
        public string DataType = "";
        public bool HasHeader;
        public ISubscriptionCallbackHelper CallbackHelper;
        public bool Latch;
        public string Md5Sum = "";
        public string MessageDefinition = "";
        public int QueueSize;
        public string Topic = "";

        public SubscribeOptions(string topic, string dataType, string md5sum, int queueSize, ISubscriptionCallbackHelper callbackHelper)
        {
            this.Topic = topic;
            this.QueueSize = queueSize;
            this.CallbackHelper = callbackHelper;
            this.DataType = dataType;
            this.Md5Sum = md5sum;
        }
    }

    public delegate void CallbackDelegate<in T>(T argument) where T : RosMessage, new();

    public class SubscribeOptions<T>
        : SubscribeOptions
        where T : RosMessage, new()
    {
        public SubscribeOptions()
             : this("", 1)
        {
        }

        public SubscribeOptions(string topic, int queueSize, CallbackDelegate<T> callbackHelper = null)
            : base(topic, null, null, queueSize, null)
        {
            var generic = new T();
            if (callbackHelper != null)
                CallbackHelper = new SubscriptionCallbackHelper<T>(generic.MessageType, callbackHelper);
            else
                CallbackHelper = new SubscriptionCallbackHelper<T>(generic.MessageType);

            DataType = generic.MessageType;
            Md5Sum = generic.MD5Sum();
        }
    }
}
