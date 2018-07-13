using System;
using System.Collections.Generic;
using System.Linq;

namespace Uml.Robotics.Ros.Transforms
{
    public class TimeCache
    {
        private const int MIN_INTERPOLATION_DISTANCE = 5;
        private const uint MAX_LENGTH_LINKED_LIST = 10000000;
        private const Int64 DEFAULT_MAX_STORAGE_TIME = 1000000000;

        private readonly SortedList<ulong, TransformStorage> storage = new SortedList<ulong, TransformStorage>();
        private ulong maxStorageTime;

        public TimeCache()
            : this(DEFAULT_MAX_STORAGE_TIME)
        {
        }

        public TimeCache(ulong maxStorageTime)
        {
            this.maxStorageTime = maxStorageTime;
        }

        public static ulong ToLong(TimeData timeData)
        {
            return (ulong)timeData.Ticks;
        }

        private int FindClosest(ref TransformStorage one, ref TransformStorage two, ulong targetTime, out string errorMessage)
        {
            errorMessage = null;
            lock (storage)
            {
                if (storage.Count == 0)
                {
                    errorMessage = CreateEmptyException();
                    return 0;
                }

                if (targetTime == 0)
                {
                    one = storage.Last().Value;
                    return 1;
                }

                if (storage.Count == 1)
                {
                    TransformStorage ts = storage.First().Value;
                    if (ts.Stamp == targetTime)
                    {
                        one = ts;
                        return 1;
                    }
                    errorMessage = CreateExtrapolationException1(targetTime, ts.Stamp);
                    return 0;
                }

                ulong latestTime = storage.Last().Key;
                ulong earliestTime = storage.First().Key;
                if (targetTime == latestTime)
                {
                    one = storage.Last().Value;
                    return 1;
                }
                if (targetTime == earliestTime)
                {
                    one = storage.First().Value;
                    return 1;
                }
                if (targetTime > latestTime)
                {
                    errorMessage = CreateExtrapolationException2(targetTime, latestTime);
                    return 0;
                }
                if (targetTime < earliestTime)
                {
                    errorMessage = CreateExtrapolationException3(targetTime, earliestTime);
                    return 0;
                }

                ulong i = 0;
                ulong j = storage.Last(kvp =>
                {
                    // look for the first keyvaluepair in the sorted list with a key greater than our target.
                    // if it is the last keyvaluepair's key, aka, the highest stamp
                    if (kvp.Key <= targetTime)
                    {
                        i = kvp.Key;
                        return false;
                    }
                    return true;
                }).Key;
                one = storage[i];
                two = storage[j];
            }
            return 2;
        }

        private void Interpolate(TransformStorage one, TransformStorage two, ulong time, ref TransformStorage output)
        {
            if (one.Stamp == two.Stamp)
            {
                output = two;
                return;
            }

            if (output == null)
                output = new TransformStorage();

            double ratio = (time - one.Stamp) / (two.Stamp - one.Stamp);
            output.Translation = Vector3.Lerp(one.Translation, two.Translation, ratio);
            output.Rotation = Slerp(one.Rotation, two.Rotation, ratio);
            output.Stamp = one.Stamp;
            output.FrameId = one.FrameId;
            output.ChildFrameId = one.ChildFrameId;
        }

        private Quaternion Slerp(Quaternion q1, Quaternion q2, double rt)
        {
            return q1.Slerp(q2, rt);
        }

        private void PruneList()
        {
            ulong latest_time = storage.Last().Key;
            while (storage.Count > 0 && storage.First().Key + maxStorageTime < latest_time || storage.Count > MAX_LENGTH_LINKED_LIST)
                storage.RemoveAt(0);
        }

        public bool GetData(TimeData time_, ref TransformStorage data_out, out string error_str)
        {
            return GetData(ToLong(time_), ref data_out, out error_str);
        }

        public bool GetData(ulong time_, ref TransformStorage data_out, out string error_str)
        {
            TransformStorage temp1 = null, temp2 = null;
            int num_nodes = FindClosest(ref temp1, ref temp2, time_, out error_str);
            switch (num_nodes)
            {
                case 0:
                    return false;
                case 1:
                    data_out = temp1;
                    break;
                case 2:
                    if (temp1.FrameId == temp2.FrameId)
                    {
                        Interpolate(temp1, temp2, time_, ref data_out);
                    }
                    else
                    {
                        data_out = temp1;
                    }
                    break;
                default:
                    throw new Exception("Function getData in TimeCache.cs failed: num_nodes has to be <=2.");
            }
            return true;
        }

        public bool InsertData(TransformStorage newData)
        {
            lock (storage)
            {
                if (storage.Count > 0 && storage.First().Key > newData.Stamp + maxStorageTime)
                {
                    if (!SimTime.Instance.IsTimeSimulated)
                        return false;

                    storage.Clear();
                }
                storage[newData.Stamp] = newData;
                PruneList();
            }
            return true;
        }

        public void ClearList()
        {
            lock (storage)
            {
                storage.Clear();
            }
        }

        public uint GetParent(ulong time, out string errorMessage)
        {
            TransformStorage temp1 = null, temp2 = null;
            int num_nodes = FindClosest(ref temp1, ref temp2, time, out errorMessage);
            if (num_nodes == 0)
                return 0;

            return temp1.FrameId;
        }

        public uint GetParent(TimeData time_, out string errorMessage)
        {
            return GetParent(ToLong(time_), out errorMessage);
        }

        public TimeAndFrameId GetLatestTimeAndParent()
        {
            lock (storage)
            {
                if (storage.Count == 0)
                {
                    return new TimeAndFrameId(0, 0);
                }
                TransformStorage ts = storage.Last().Value;
                return new TimeAndFrameId(ts.Stamp, ts.FrameId);
            }
        }

        public uint GetListLength()
        {
            lock (storage)
            {
                return (uint)storage.Count;
            }
        }

        public ulong GetLatestTimeStamp()
        {
            lock (storage)
            {
                if (storage.Count == 0)
                    return 0;
                return storage.Last().Key;
            }
        }

        public ulong GetOldestTimestamp()
        {
            lock (storage)
            {
                if (storage.Count == 0)
                    return 0;
                return storage.First().Key;
            }
        }

        private string CreateEmptyException()
        {
            return "Cache is empty!";
        }

        private string CreateExtrapolationException1(ulong t0, ulong t1)
        {
            return "Lookup would require extrapolation at time \n" + t0 + ", but only time \n" + t1 + " is in the buffer";
        }

        private string CreateExtrapolationException2(ulong t0, ulong t1)
        {
            return "Lookup would require extrapolation into the future. Requested time \n" + t0 + " but the latest data is at the time \n" + t1;
        }

        private string CreateExtrapolationException3(ulong t0, ulong t1)
        {
            return "Lookup would require extrapolation into the past. Requested time \n" + t0 + " but the earliest data is at the time \n" + t1;
        }
    }
}
