using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Xamla.Robotics.Ros.Async
{
    public class AsyncAutoResetEvent
    {
        Deque<TaskCompletionSource<object>> queue;
        bool signaled;
        Exception exception;

        public AsyncAutoResetEvent(bool initialState = false)
        {
            this.queue = new Deque<TaskCompletionSource<object>>();
            this.signaled = initialState;
        }

        public Task WaitAsync(CancellationToken cancel)
        {
            lock (queue)
            {
                if (exception != null)
                {
                    return TaskEx.Throw(exception);
                }

                if (signaled)
                {
                    signaled = false;
                    return TaskConstants.Completed;
                }

                if (cancel.IsCancellationRequested)
                    return TaskConstants.Canceled;

                var tcs = new TaskCompletionSource<object>();
                if (cancel.CanBeCanceled)
                {
                    var registration = cancel.Register(() =>
                    {
                        lock (queue)
                        {
                            queue.Remove(tcs);
                        }
                        tcs.TrySetCanceled();
                    });
                    tcs.Task.Finally(registration.Dispose);
                }

                queue.PushBack(tcs);
                return tcs.Task;
            }
        }

        public Task WaitAsync()
        {
            return WaitAsync(CancellationToken.None);
        }

        public void Set(bool useTaskPool = false)
        {
            TaskCompletionSource<object> release = null;

            lock (queue)
            {
                if (queue.Count > 0)
                {
                    release = queue.Front;
                    queue.PopFront();
                }
                else
                {
                    signaled = true;
                }
            }

            if (release != null)
            {
                if (useTaskPool)
                {
                    Task.Factory.StartNew(s => ((TaskCompletionSource<object>)s).TrySetResult(null),
                        release,
                        CancellationToken.None,
                        TaskCreationOptions.PreferFairness,
                        TaskScheduler.Default
                    );
                }
                else
                {
                    release.TrySetResult(null);
                }
            }
        }

        public void SetException(Exception exception)
        {
            List<TaskCompletionSource<object>> waiting;

            lock (queue)
            {
                this.exception = exception;
                waiting = queue.ToList();
                queue.Clear();
            }

            foreach (var release in waiting)
            {
                release.TrySetException(exception);
            }
        }
    }
}
