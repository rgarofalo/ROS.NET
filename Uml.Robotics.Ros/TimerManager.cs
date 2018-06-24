using System;
using System.Collections.Generic;
using System.Threading;
using Microsoft.Extensions.Logging;

namespace Uml.Robotics.Ros
{
    /// <summary>
    ///     Timer management utility class
    /// </summary>
    public class TimerManager : IDisposable
    {
        /// <summary>
        ///     Holds on to known timer instances
        /// </summary>
        private HashSet<WrappedTimer> timerList = new HashSet<WrappedTimer>();

        /// <summary>
        ///     clean up shop
        /// </summary>
        public void Dispose()
        {
            lock (timerList)
            {
                //be extra super sure they're all dead
                foreach (WrappedTimer t in timerList)
                {
                    t.Dispose();
                }
                timerList.Clear();
            }
        }

        /// <summary>
        ///     Wrap and start timer a with added functionality, and make sure it dies with this TimerManager
        ///     This DOES NOT START IT.
        /// </summary>
        /// <param name="cb">
        ///     The callback of the wrapped timer
        /// </param>
        /// <param name="d">
        ///     The delay it should have
        /// </param>
        /// <param name="p">
        ///     The period it should have
        /// </param>
        public WrappedTimer MakeTimer(TimerCallback cb, int d = Timeout.Infinite, int p = Timeout.Infinite)
        {
            WrappedTimer wt = new WrappedTimer(cb, d, p);
            MakeTimer(wt);
            return wt;
        }

        /// <summary>
        ///     Wrap a timer a with added functionality, and make sure it dies with this TimerManager
        ///     This DOES NOT START IT.
        /// </summary>
        /// <param name="cb">
        ///     The callback of the wrapped timer
        /// </param>
        /// <param name="d">
        ///     The delay it should have
        /// </param>
        /// <param name="p">
        ///     The period it should have
        /// </param>
        public WrappedTimer StartTimer(TimerCallback cb, int d = Timeout.Infinite, int p = Timeout.Infinite)
        {
            WrappedTimer wt = MakeTimer(cb, d, p);
            wt.Start();
            return wt;
        }

        /// <summary>
        ///     Add a wrapped timer to the hashset
        /// </summary>
        /// <param name="t">the wrapped timer</param>
        public void MakeTimer(WrappedTimer t)
        {
            lock (timerList)
            {
                if (timerList.Contains(t))
                    throw new Exception("The same timer cannot be tracked twice");
                timerList.Add(t);
            }
        }

        /// <summary>
        ///     Stop tracking a timer, and kill it
        /// </summary>
        /// <param name="t">The timer to forget and kill</param>
        public void RemoveTimer(ref WrappedTimer t)
        {
            lock (timerList)
            {
                if (timerList.Contains(t))
                    timerList.Remove(t);
            }
            t.Dispose();
            t = null;
        }
    }

    /// <summary>
    ///     Wrap the System.Threading.Timer with useful functions and state information
    /// </summary>
    public class WrappedTimer : IDisposable
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<WrappedTimer>();

        //variable backing for properties
        private int delay = Timeout.Infinite;
        private int period = Timeout.Infinite;
        private bool running;

        private TimerCallback cb;
        private Timer timer;

        /// <summary>
        ///     Instantiate the wrapper
        /// </summary>
        /// <param name="t">A timer</param>
        /// <param name="d">Its delay</param>
        /// <param name="p">Its period</param>
        public WrappedTimer(TimerCallback cb, int d, int p)
        {
            // add a callback between the caller and the timer, so non-periodic timers state becomes false right before the one time their callback happens
            // (If a timer's period is Timeout.Infinite, it will fire once delay ms after Start is called. If start is recalled before then, nothing changes.
            // To reset the time to the next pending callback before the callback happens, use Restart)
            this.cb = o =>
            {
                if (period == Timeout.Infinite)
                    running = false;
                cb(o);
            };
            timer = new Timer(this.cb, null, Timeout.Infinite, Timeout.Infinite);
            delay = d;
            period = p;
        }

        /// <summary>
        ///     This timer's delay
        /// </summary>
        public int Delay
        {
            get { return delay; }
            set
            {
                if (timer == null)
                    throw new ObjectDisposedException("Timer instance has already been disposed");
                if (delay != value && Running)
                    timer.Change(value, period);
                delay = value;
            }
        }

        /// <summary>
        ///     This timer's period
        /// </summary>
        public int Period
        {
            get { return period; }
            set
            {
                if (timer == null)
                    throw new ObjectDisposedException("Timer instance has already been disposed");
                if (period != value && Running)
                    timer.Change(delay, value);
                period = value;
            }
        }

        /// <summary>
        ///     Is it running
        /// </summary>
        public bool Running
        {
            get { return running; }
            set
            {
                if (timer == null)
                    throw new ObjectDisposedException("Timer instance has already been disposed");
                if (value && !running) Start();
                if (!value && running) Stop();
            }
        }

        /// <summary>
        ///     Performs application-defined tasks associated with freeing, releasing, or resetting unmanaged resources.
        /// </summary>
        public void Dispose()
        {
            if (timer == null)
                return;

            timer.Dispose();
            timer = null;
        }

        /// <summary>
        ///     Starts the timer with this wrapper's set delay and period.
        /// </summary>
        public void Start()
        {
            if (timer == null)
                throw new ObjectDisposedException("Timer instance has already been disposed");
            if (Running)
                return;
            try
            {
                timer.Change(delay, period);
                running = true;
            }
            catch (Exception ex)
            {
                logger.LogError("Error starting timer: " + ex);
            }
        }

        /// <summary>
        ///     Sets this timers delay and period, and immediately starts it
        /// </summary>
        /// <param name="d"></param>
        /// <param name="p"></param>
        public void Start(int d, int p)
        {
            if (timer == null)
                throw new ObjectDisposedException("Timer instance has already been disposed");
            delay = d;
            period = p;
            try
            {
                timer.Change(delay, period);
                running = d != Timeout.Infinite && p != Timeout.Infinite;
            }
            catch (Exception ex)
            {
                logger.LogError("Error starting timer: " + ex);
            }
        }

        /// <summary>
        ///     Stops then Resets the timer, causing any time spent waiting for the next callback to be reset
        /// </summary>
        public void Restart()
        {
            Stop();
            Start();
        }

        /// <summary>
        ///     Stops the timer from firing, while remembering its last set state and period
        /// </summary>
        public void Stop()
        {
            if (timer == null)
                throw new ObjectDisposedException("Timer instance has already been disposed");
            if (!Running)
                return;
            try
            {
                timer.Change(Timeout.Infinite, Timeout.Infinite);
                running = false;
            }
            catch (Exception ex)
            {
                logger.LogError("Error starting timer: " + ex);
            }
        }
    }
}
