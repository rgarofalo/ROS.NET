namespace Uml.Robotics.Ros
{
    public delegate void SubscriberStatusCallback(SingleSubscriberPublisher publisher);

    public class SubscriberCallbacks
    {
        public ICallbackQueue CallbackQueue { get; }
        public SubscriberStatusCallback OnConnect { get; }
        public SubscriberStatusCallback OnDisconnect { get; }
        public long CallbackId { get; set; } = -1;

        public SubscriberCallbacks(
            SubscriberStatusCallback onConnect,
            SubscriberStatusCallback onDisconnect,
            ICallbackQueue callbackQueue
        )
        {
            this.OnConnect = onConnect;
            this.OnDisconnect = onDisconnect;
            this.CallbackQueue = callbackQueue;
        }
    }
}
