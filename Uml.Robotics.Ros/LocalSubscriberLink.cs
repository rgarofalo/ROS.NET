using System;

namespace Uml.Robotics.Ros
{
    internal class LocalSubscriberLink : SubscriberLink
    {
        private object gate = new object();
        private bool dropped;
        private LocalPublisherLink subscriber;

        public LocalSubscriberLink(Publication pub)
        {
            parent = pub;
            topic = parent.Name;
        }

        public override void Dispose()
        {
            lock (gate)
            {
                if (dropped)
                    return;
                dropped = true;
            }

            subscriber?.Dispose();

            lock (parent)
            {
                parent.RemoveSubscriberLink(this);
            }
        }

        public void SetSubscriber(LocalPublisherLink publisherLink)
        {
            subscriber = publisherLink;
            connectionId = ConnectionManager.Instance.GetNewConnectionId();
            DestinationCallerId = ThisNode.Name;
        }

        public override void EnqueueMessage(MessageAndSerializerFunc holder)
        {
            lock (gate)
            {
                if (dropped)
                    return;
            }

            if (subscriber != null)
                subscriber.HandleMessage(holder.msg, holder.serialize, holder.nocopy);
        }

        public override void GetPublishTypes(ref bool ser, ref bool nocopy, string messageType)
        {
            lock (gate)
            {
                if (dropped)
                {
                    ser = false;
                    nocopy = false;
                    return;
                }
            }

            subscriber.GetPublishTypes(ref ser, ref nocopy, messageType);
        }
    }
}
