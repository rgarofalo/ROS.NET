using System.Collections.Generic;

namespace Uml.Robotics.Ros
{
    public class ServiceClientOptions
    {
        public IDictionary<string, string> HeaderValues { get; }
        public string md5sum { get; }
        public bool Persistent { get;  }
        public string service { get;}

        public ServiceClientOptions(string service, bool persistent, IDictionary<string, string> headerValues)
            : this(service, persistent, headerValues, "*")
        {
        }

        public ServiceClientOptions(string service, bool persistent, IDictionary<string, string> headerValues, string md5sum)
        {
            this.HeaderValues = headerValues;
            this.md5sum = md5sum;
            this.Persistent = persistent;
            this.service = service;
        }
    }
}
