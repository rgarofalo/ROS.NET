namespace Uml.Robotics.Ros
{
    public class ServiceCallFailedException
        : RosException
    {
        public string ServiceName { get; }

        public ServiceCallFailedException(string serviceName)
            : this(serviceName, $"Service call to {serviceName} failed.")
        {
        }

        public ServiceCallFailedException(string serviceName, string message)
            : base(message)
        {
            this.ServiceName = serviceName;
        }
    }
}
