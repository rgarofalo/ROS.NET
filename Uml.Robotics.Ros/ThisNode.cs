using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    public static class ThisNode
    {
        private static readonly ILogger logger = ApplicationLogging.CreateLogger(nameof(ThisNode));
        public static string Name { get; private set; } = "empty";
        public static string Namespace { get; private set; } = "";

        public static void Init(string n, IDictionary<string, string> remappings)
        {
            Init(n, remappings, InitOptions.None);
        }

        public static void Init(string name, IDictionary<string, string> remappings, InitOptions options)
        {
            Name = name;

            bool disableAnonymous = false;
            if (remappings.ContainsKey("__name"))
            {
                Name = remappings["__name"];
                disableAnonymous = true;
            }

            if (remappings.ContainsKey("__ns"))
            {
                Namespace = remappings["__ns"];
            }

            if (string.IsNullOrEmpty(Namespace))
            {
                Namespace = "/";
            }

            long walltime = DateTime.UtcNow.Ticks;
            Names.Init(remappings);

            if (Name.Contains("/"))
                throw new ArgumentException("Slashes '/' are not allowed in names", nameof(name));
            if (Name.Contains("~"))
                throw new ArgumentException("Tildes '~' are not allowed in names", nameof(name));

            try
            {
                Name = Names.Resolve(Namespace, Name);
            }
            catch (Exception e)
            {
                logger.LogError(e, e.Message);
            }

            if (options.HasFlag(InitOptions.AnonymousName) && !disableAnonymous)
            {
                int oldLength = Name.Length;
                Name += "_" + walltime;
                if (Name.Length > 201)
                {
                    Name = Name.Remove(201);
                }
            }
        }
    }
}
