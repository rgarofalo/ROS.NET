using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Xamla.Robotics.Ros.Async
{
    [DebuggerDisplay("Count = {count}")]
    [DebuggerTypeProxy(typeof(Deque<>.DebugView))]
    public class Deque<T>
        : IList<T>
    {
        const int DEFAULT_INITIAL_CAPACITY = 8;

        public class Enumerator : IEnumerator<T>
        {
            T current;
            Deque<T> deque;
            T[] buffer;
            int index;
            int remaining;
            int version;

            internal Enumerator(Deque<T> deque)
            {
                this.deque = deque;
                this.version = deque.version;
                this.buffer = deque.buffer;
                this.remaining = deque.count;
                this.index = (remaining == 0) ? 0 : deque.IndexToBufferOffset(0);
                this.current = default(T);
            }

            public T Current
            {
                get { return current; }
            }

            public void Dispose()
            {
                current = default(T);
            }

            object System.Collections.IEnumerator.Current
            {
                get { return current; }
            }

            public bool MoveNext()
            {
                if (deque.version != version)
                    throw new InvalidOperationException("The used iterator is no longer valid. The underlying collection has been modified.");

                if (remaining > 0)
                {
                    current = buffer[index++];
                    if (index >= buffer.Length)
                        index = 0;
                    --remaining;
                    return true;
                }

                return false;
            }

            public void Reset()
            {
                this.version = deque.version;
                this.index = deque.IndexToBufferOffset(0);
                this.current = default(T);
            }
        }

        T[] buffer;
        int offset;
        int count;
        int version;

        public Deque()
            : this(DEFAULT_INITIAL_CAPACITY)
        {
        }

        public Deque(int capacity)
        {
            capacity = Math.Max(capacity, DEFAULT_INITIAL_CAPACITY);
            this.buffer = new T[capacity];
        }

        void EnsureNotEmpty()
        {
            if (count == 0)
                throw new InvalidOperationException("Deque is empty.");
        }

        public T Front
        {
            get
            {
                EnsureNotEmpty();
                return buffer[offset];
            }
            set
            {
                EnsureNotEmpty();
                buffer[offset] = value;
            }
        }

        public T Back
        {
            get
            {
                EnsureNotEmpty();
                return this[count - 1];
            }
            set
            {
                EnsureNotEmpty();
                this[count - 1] = value;
            }
        }

        public void PushFront(T value)
        {
            AllocateFront(1);
            this.Front = value;
            ++version;
        }

        public void PushBack(T value)
        {
            AllocateBack(1);
            this.Back = value;
            ++version;
        }

        public void PopFront()
        {
            buffer[offset] = default(T);
            ++offset;
            if (offset >= buffer.Length)
            {
                Debug.Assert(offset == buffer.Length);
                offset = 0;
            }
            --count;
            TrimExcess();
            ++version;
        }

        public void PopBack()
        {
            buffer[IndexToBufferOffset(count - 1)] = default(T);
            --count;
            TrimExcess();
            ++version;
        }

        public void Add(T item)
        {
            PushBack(item);
        }

        public void Clear()
        {
            count = 0;
            offset = 0;
            ++version;
        }

        public int Capacity
        {
            get { return buffer.Length; }
            set { ChangeCapacity(value); }
        }

        public void Insert(int index, T value)
        {
            if (index == 0)
                PushFront(value);
            else if (index == count)
                PushBack(value);
            else
            {
                if (index > count / 2)
                {
                    int oldCount = this.count;
                    AllocateBack(1);
                    Move(index, index + 1, oldCount - index);          // count has been increased by AllocateBack
                }
                else
                {
                    AllocateFront(1);
                    Move(1, 0, index);
                }
                this[index] = value;
                ++version;
            }
        }

        public void InsertRange(int index, IEnumerable<T> values)
        {
            if (values == null)
                throw new ArgumentNullException("value");

            var collection = values as ICollection<T>;       // try to get information about length of enumerable sequence
            if (collection != null)
            {
                int insertCount = collection.Count;
                EnsureFreeSpace(insertCount);
                if (index > this.count / 2)
                {
                    int oldCount = this.Count;
                    AllocateBack(insertCount);
                    Move(index, index + insertCount, oldCount - index);     // count has been increased by AllocateBack
                }
                else
                {
                    AllocateFront(insertCount);
                    Move(insertCount, 0, index);
                }

                T[] inputArray = new T[insertCount];
                collection.CopyTo(inputArray, 0);

                foreach (var x in inputArray)
                {
                    this[index++] = x;
                }
            }
            else
            {
                foreach (var x in values)
                    Insert(index++, x);
            }
            ++version;
        }

        public void RemoveAt(int index)
        {
            RemoveRange(index, 1);
        }

        public void RemoveRange(int index, int count)
        {
            if (count <= 0)
                return;

            int tail = this.count - index - count;
            if (index < tail)      // before < after
            {
                Move(0, count, index);
                ClearSegment(0, count);
                offset = (offset + count) % buffer.Length;
            }
            else
            {
                Move(index + count, index, tail);
                ClearSegment(index + tail, count);
            }

            this.count -= count;
            TrimExcess();
            ++version;
        }

        public int IndexOf(T item)
        {
            int i = offset;
            for (int j = 0; j < count; ++j, ++i)
            {
                if (i >= buffer.Length)
                    i = 0;
                if (object.Equals(buffer[i], item))
                    return j;
            }
            return -1;
        }

        public T this[int index]
        {
            get { return buffer[IndexToBufferOffset(index)]; }
            set
            {
                buffer[IndexToBufferOffset(index)] = value;
                ++version;
            }
        }

        public bool Contains(T item)
        {
            return IndexOf(item) >= 0;
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            foreach (var x in this)
                array[arrayIndex++] = x;
        }

        public int Count
        {
            get { return count; }
        }

        public bool IsReadOnly
        {
            get { return false; }
        }

        public bool Remove(T item)
        {
            int index = IndexOf(item);
            if (index < 0)
                return false;
            RemoveAt(index);
            ++version;
            return true;
        }

        public IEnumerator<T> GetEnumerator()
        {
            return new Enumerator(this);
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public T[] ToArray()
        {
            var a = new T[this.Count];
            this.CopyTo(a, 0);
            return a;
        }

        int IndexToBufferOffset(int index)
        {
            if (index < 0 || index >= count)
                throw new IndexOutOfRangeException();
            return (index + this.offset) % buffer.Length;
        }

        void EnsureFreeSpace(int required)
        {
            int missing = count + required - buffer.Length;
            if (missing <= 0)
                return;

            // grow by half buffer size (new = 150% of old size)
            int newCapacity = buffer.Length + Math.Max(buffer.Length / 2, missing);
            ChangeCapacity(newCapacity);
        }

        void TrimExcess()
        {
            if (buffer.Length > DEFAULT_INITIAL_CAPACITY && count * 3 <= buffer.Length)
                ChangeCapacity(Math.Max(count * 3/2, DEFAULT_INITIAL_CAPACITY));
        }

        void ChangeCapacity(int newCapacity)
        {
            newCapacity = Math.Max(Math.Max(newCapacity, count), DEFAULT_INITIAL_CAPACITY);
            T[] newBuffer = new T[newCapacity];     // copy allocated parts to new buffer
            int backCount = Math.Min(buffer.Length - offset, count);
            Array.Copy(buffer, offset, newBuffer, 0, backCount);
            Array.Copy(buffer, 0, newBuffer, backCount, count - backCount);
            offset = 0;
            buffer = newBuffer;
            ++version;
        }

        void AllocateFront(int count)
        {
            EnsureFreeSpace(count);
            this.count += count;
            offset -= count;
            if (offset < 0)
                offset += buffer.Length;
        }

        void AllocateBack(int count)
        {
            EnsureFreeSpace(count);
            this.count += count;
        }

        void Move(int sourceIndex, int destinationIndex, int count)
        {
            if (count == 0)
                return;

            if (sourceIndex < destinationIndex)
            {
                // backward
                int i = IndexToBufferOffset(sourceIndex + count - 1), j = IndexToBufferOffset(destinationIndex + count - 1);
                while (count > 0)
                {
                    buffer[j--] = buffer[i--];
                    if (j < 0)
                        j = buffer.Length - 1;
                    if (i < 0)
                        i = buffer.Length - 1;
                    --count;
                }
            }
            else
            {
                // forward
                int i = IndexToBufferOffset(sourceIndex), j = IndexToBufferOffset(destinationIndex);
                while (count > 0)
                {
                    buffer[j++] = buffer[i++];
                    if (j >= buffer.Length)
                        j = 0;
                    if (i >= buffer.Length)
                        i = 0;
                    --count;
                }
            }
        }

        void ClearSegment(int index, int count)
        {
            var startIndex = IndexToBufferOffset(index);
            int back = Math.Min(buffer.Length - startIndex, count);
            Array.Clear(buffer, startIndex, back);
            Array.Clear(buffer, 0, count - back);
        }

        [DebuggerNonUserCode]
        private sealed class DebugView
        {
            private readonly Deque<T> deque;
            [DebuggerBrowsable(DebuggerBrowsableState.RootHidden)]
            public T[] Items
            {
                get
                {
                    T[] array = new T[this.deque.Count];
                    this.deque.CopyTo(array, 0);
                    return array;
                }
            }

            public DebugView(Deque<T> deque)
            {
                this.deque = deque;
            }
        }
    }
}
