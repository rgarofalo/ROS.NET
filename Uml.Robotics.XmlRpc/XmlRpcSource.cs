using Microsoft.Extensions.Logging;
using System;
using System.Diagnostics;
using System.Net.Sockets;
using System.Text;

namespace Uml.Robotics.XmlRpc
{
    public abstract class XmlRpcSource : IDisposable
    {
        private const int READ_BUFFER_LENGTH = 4096;

        private readonly ILogger logger = XmlRpcLogging.CreateLogger<XmlRpcSource>();

        /// <summary>
        /// In the client, keep connections open if you intend to make multiple calls.
        /// </summary>
        public bool KeepOpen { get; set; }

        public virtual NetworkStream Stream => null;
        public virtual Socket Socket => null;
        public virtual void Close() { }

        public abstract XmlRpcDispatch.EventType HandleEvent(XmlRpcDispatch.EventType eventType);

        internal virtual bool ReadHeader(ref HttpHeader header)
        {
            // Read available data
            NetworkStream stream = this.Stream;
            if (stream == null)
            {
                throw new Exception("Could not access network stream");
            }

            byte[] data = new byte[READ_BUFFER_LENGTH];
            try
            {
                int dataLen = stream.Read(data, 0, READ_BUFFER_LENGTH);

                if (dataLen == 0)
                {
                    return false;   // zero bytes read -> graceful disconnect
                }

                if (header == null)
                {
                    header = new HttpHeader(Encoding.ASCII.GetString(data, 0, dataLen));
                    Debug.Assert(header.HeaderStatus != HttpHeader.ParseStatus.UNINITIALIZED);
                }
                else if (header.Append(Encoding.ASCII.GetString(data, 0, dataLen)) == HttpHeader.ParseStatus.PARTIAL_HEADER)
                {
                    // if we successfully append a piece of the header, return true, but DO NOT change states
                    return true;
                }
            }
            catch (SocketException ex)
            {
                logger.LogError("XmlRpcServerConnection::readHeader: error while reading header ({0}).", ex.Message);
                return false;
            }
            catch (Exception ex)
            {
                logger.LogError("XmlRpcServerConnection::readHeader: error while reading header ({0}).", ex.Message);
                return false;
            }

            if (header.HeaderStatus != HttpHeader.ParseStatus.COMPLETE_HEADER)
                return false;

            return true;
        }

        public void Dispose()
        {
            Close();
        }

        // In the server, a new source (XmlRpcServerConnection) is created
        // for each connected client. When each connection is closed, the
        // corresponding source object is deleted.
    }
}
