using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Threading;
using System.Threading.Tasks;

namespace Xamla.Robotics.Ros.Async
{
    public interface IAsyncObserver<T>
    {
        Task OnNext(T value, CancellationToken cancel);
        void OnError(Exception error);
        void OnCompleted();
    }

    public class AsyncQueue<T>
        : IAsyncObserver<T>
        , IDisposable
    {
        AsyncAutoResetEvent enqueSignal;
        AsyncAutoResetEvent dequeueSignal;

        T current;
        ConcurrentQueue<T> queue;
        readonly int maxLength;
        bool completed;
        ExceptionDispatchInfo error;
        readonly bool dropOldestOnOverflow;
        bool disposed;

        public AsyncQueue(int maxLength, bool dropOldestOnOverflow)
        {
            this.maxLength = maxLength;
            this.dropOldestOnOverflow = dropOldestOnOverflow;
            this.queue = new ConcurrentQueue<T>();
            this.enqueSignal = new AsyncAutoResetEvent();
            this.dequeueSignal = new AsyncAutoResetEvent();
        }

        public bool IsCompleted
        {
            get
            {
                lock (queue)
                {
                    return completed;
                }
            }
        }

        public int Count => queue.Count;

        public void OnError(Exception error)
        {
            lock (queue)
            {
                if (completed)
                    return;

                this.error = ExceptionDispatchInfo.Capture(error);
                completed = true;
            }

            enqueSignal.Set(true);
            dequeueSignal.Set(true);        // release waiting OnNext calls
        }

        public void OnCompleted()
        {
            lock (queue)
            {
                if (completed)
                    return;

                completed = true;
            }

            enqueSignal.Set(true);
            dequeueSignal.Set(true);        // release waiting OnNext calls
        }

        public bool TryOnNext(T value)
        {
            lock (queue)
            {
                if (completed)
                    return false;

                if (queue.Count >= maxLength)
                {
                    if (!dropOldestOnOverflow)
                        return false;

                    queue.TryDequeue(out T lost);
                }

                queue.Enqueue(value);
            }

            enqueSignal.Set(true);
            return true;
        }

        public async Task OnNext(T value, CancellationToken cancel)
        {
            for (;;)
            {
                bool enqueued = false;
                lock (queue)
                {
                    if (completed)
                    {
                        dequeueSignal.Set();
                        return;     // ignore OnNext calls after OnCompleted|OnError was called
                    }

                    if (queue.Count < maxLength)
                    {
                        queue.Enqueue(value);
                        enqueued = true;
                    }
                    else if (dropOldestOnOverflow)
                    {
                        if (queue.TryDequeue(out T lost))
                        {
                            continue;
                        }
                    }
                }

                if (enqueued)
                {
                    enqueSignal.Set(true);
                    return;
                }

                await dequeueSignal.WaitAsync(cancel).ConfigureAwait(false);
            }
        }

        public void Dispose()
        {
            lock (queue)
            {
                if (completed)
                    return;

                completed = true;
                disposed = true;
            }

            dequeueSignal.Set(true);
            enqueSignal.Set(true);
        }

        public T Current
        {
            get { return current; }
        }

        public async Task<bool> MoveNext(CancellationToken cancel)
        {
            for (;;)
            {
                if (queue.TryDequeue(out current))
                {
                    dequeueSignal.Set(true);
                    return true;
                }

                lock (queue)
                {
                    if (queue.Count > 0)
                        continue;

                    if (completed)
                    {
                        enqueSignal.Set();
                        Debug.Assert(!queue.TryDequeue(out current));
                        if (error != null)
                            error.Throw();
                        if (disposed)
                            throw new OperationCanceledException("AsyncQueue object has been disposed");
                        return false;
                    }
                }

                await enqueSignal.WaitAsync(cancel).ConfigureAwait(false);
            }
        }

        public IList<T> Flush()
        {
            return queue.ToList();
        }
    }
}
