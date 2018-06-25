using System;
using System.Threading;
using Microsoft.Extensions.Logging;

using Messages.rosgraph_msgs;

namespace Uml.Robotics.Ros
{
    public class SimTime
    {
        public delegate void SimTimeDelegate(TimeSpan ts);
        public event SimTimeDelegate SimTimeEvent;

        private static readonly ILogger logger = ApplicationLogging.CreateLogger<SimTime>();
        private static Lazy<SimTime> instance = new Lazy<SimTime>(() => new SimTime(), LazyThreadSafetyMode.ExecutionAndPublication);

        public static SimTime Instance =>
            instance.Value;

        internal static void Terminate() =>
            Instance.Shutdown();

        internal static void Reset() =>
            instance = new Lazy<SimTime>(() => new SimTime(), LazyThreadSafetyMode.ExecutionAndPublication);

        private bool checkedSimTime;
        private NodeHandle nodeHandle;
        private bool simTime;
        private Subscriber simTimeSubscriber;

        private SimTime()
        {
            new Thread(() =>
            {
                try
                {
                    while (!ROS.IsStarted() && !ROS.shutting_down)
                    {
                        Thread.Sleep(100);
                    }

                    if (!ROS.shutting_down)
                    {
                        nodeHandle = new NodeHandle();
                        simTimeSubscriber = nodeHandle.Subscribe<Clock>("/clock", 1, SimTimeCallback);
                    }
                }
                catch (Exception e)
                {
                    logger.LogError("Caught exception in sim time thread: " + e.Message);
                }
            }).Start();
        }

        public bool IsTimeSimulated
        {
            get { return simTime; }
        }

        public void Shutdown()
        {
            simTimeSubscriber?.Dispose();
            nodeHandle?.Dispose();
        }

        private void SimTimeCallback(Clock time)
        {
            if (!checkedSimTime)
            {
                if (Param.Get("/use_sim_time", out simTime))
                {
                    checkedSimTime = true;
                }
            }
            if (simTime && SimTimeEvent != null)
            {
                SimTimeEvent(TimeSpan.FromMilliseconds(time.clock.data.sec * 1000.0 + (time.clock.data.nsec / 100000000.0)));
            }
        }
    }
}
