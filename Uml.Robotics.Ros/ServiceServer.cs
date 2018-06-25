using System;
using System.Diagnostics;

namespace Uml.Robotics.Ros
{
    public class ServiceServer
    {
        private NodeHandle nodeHandle;
        private string service;
        private bool unadvertised;

        internal ServiceServer(string service, NodeHandle nodeHandle)
        {
            this.service = service;
            this.nodeHandle = nodeHandle;
        }

        public bool IsValid =>
            !unadvertised;

        public void Shutdown() =>
            Unadvertise();

        public string ServiceName() =>
            service;

        internal void Unadvertise()
        {
            if (!unadvertised)
            {
                unadvertised = true;
                ServiceManager.Instance.UnadvertiseService(service);
            }
        }
    }
}
