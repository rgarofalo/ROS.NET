namespace Uml.Robotics.Ros
{
    public class SingleSubscriberPublisher
    {
        private SubscriberLink link;

        internal SingleSubscriberPublisher(SubscriberLink link)
        {
            this.link = link;
        }

        public string Topic
        {
            get { return link.topic; }
        }

        public string SubscriberName
        {
            get { return link.DestinationCallerId; }
        }

        public void Publish<M>(M message)
            where M : RosMessage, new()
        {
            link.EnqueueMessage(new MessageAndSerializerFunc(message, message.Serialize, true, true));
        }
    }
}
