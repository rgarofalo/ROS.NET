using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace Xamla.Robotics.Ros.Async
{
    public static class StreamExtensions
    {
        public static async Task CopyToAsync(this Stream source, Stream destination, int bufferSize, long count, CancellationToken cancel)
        {
            var buffer = new byte[bufferSize];

            long remaining = count;
            while (remaining > 0)
            {
                cancel.ThrowIfCancellationRequested();

                int bytesRead = await source.ReadAsync(buffer, 0, Math.Min((int)remaining, buffer.Length)).ConfigureAwait(false);
                if (bytesRead == 0)
                    throw new EndOfStreamException("Unexpected end of stream");

                await destination.WriteAsync(buffer, 0, bytesRead).ConfigureAwait(false);
                remaining -= bytesRead;
            }
        }

        public static async Task<bool> ReadBlockAsync(this Stream stream, byte[] buffer, int offset, int count, CancellationToken cancel = default(CancellationToken))
        {
            while (count > 0)
            {
                int read = await stream.ReadAsync(buffer, offset, count).ConfigureAwait(false);
                if (read <= 0)
                    return false;

                offset += read;
                count -= read;
            }

            return true;
        }
    }
}
