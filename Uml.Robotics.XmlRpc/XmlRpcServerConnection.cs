using Microsoft.Extensions.Logging;
using System;
using System.IO;
using System.Net.Sockets;
using System.Text;

namespace Uml.Robotics.XmlRpc
{
    /// <summary>
    /// Incoming connection to XmlRpcServer
    /// </summary>
    public class XmlRpcServerConnection : XmlRpcSource
    {
        private enum ServerConnectionState
        {
            READ_HEADER,
            READ_REQUEST,
            WRITE_RESPONSE
        };

        private ILogger logger = XmlRpcLogging.CreateLogger<XmlRpcServerConnection>();

        private int bytesWritten;
        private ServerConnectionState connectionState;

        // Whether to keep the current client connection open for further requests
        private bool keepAlive;

        // Request headers
        private HttpHeader header;

        // The XmlRpc server that accepted this connection
        private XmlRpcServer server;

        private Socket socket;
        private NetworkStream stream;

        // The server delegates handling client requests to a serverConnection object.
        public XmlRpcServerConnection(Socket socket, XmlRpcServer server)
        {
            logger.LogInformation("XmlRpcServerConnection: new socket {0}.", socket.RemoteEndPoint.ToString());
            this.server = server;
            this.socket = socket;
            this.stream = new NetworkStream(this.socket, true);
            this.connectionState = ServerConnectionState.READ_HEADER;
            this.KeepOpen = true;
            this.keepAlive = true;
        }

        public override NetworkStream Stream =>
            stream;

        public override Socket Socket =>
            socket;

        // Handle input on the server socket by accepting the connection
        // and reading the rpc request. Return true to continue to monitor
        // the socket for events, false to remove it from the dispatcher.
        public override XmlRpcDispatch.EventType HandleEvent(XmlRpcDispatch.EventType eventType)
        {
            if (eventType.HasFlag(XmlRpcDispatch.EventType.ReadableEvent))
            {
                if (connectionState == ServerConnectionState.READ_HEADER)
                {
                    if (!ReadHeader(ref header))
                        return 0;
                }

                if (connectionState == ServerConnectionState.READ_REQUEST)
                {
                    if (!ReadRequest())
                        return 0;
                }
            }
            else if (eventType.HasFlag(XmlRpcDispatch.EventType.WritableEvent))
            {
                if (connectionState == ServerConnectionState.WRITE_RESPONSE)
                {
                    if (!WriteResponse(header.DataString))
                        return 0;
                }
            }

            return (connectionState == ServerConnectionState.WRITE_RESPONSE)
                ? XmlRpcDispatch.EventType.WritableEvent : XmlRpcDispatch.EventType.ReadableEvent;
        }

        internal override bool ReadHeader(ref HttpHeader header)
        {
            if (base.ReadHeader(ref header))
            {
                if (header.HeaderStatus == HttpHeader.ParseStatus.COMPLETE_HEADER)
                {
                    logger.LogDebug("KeepAlive: {0}", keepAlive);
                    connectionState = ServerConnectionState.READ_REQUEST;
                }

                return true;
            }

            return false;
        }

        public override void Close()
        {
            logger.LogInformation("XmlRpcServerConnection is closing");
            if (socket != null)
            {
                socket.Shutdown(SocketShutdown.Both);
                socket.Close(100);
                socket.Dispose();
                socket = null;
            }
            server.RemoveConnection(this);
        }

        private bool ReadRequest()
        {
            int left = header.ContentLength - header.DataString.Length;
            int dataLen = 0;
            if (left > 0)
            {
                byte[] data = new byte[left];
                try
                {
                    dataLen = stream.Read(data, 0, left);
                    if (dataLen == 0)
                    {
                        logger.LogError("XmlRpcServerConnection::readRequest: Stream was closed");
                        return false;
                    }
                }
                catch (Exception ex)
                {
                    logger.LogError("XmlRpcServerConnection::readRequest: error while reading the rest of data ({0}).", ex.Message);
                    return false;
                }
                header.Append(Encoding.ASCII.GetString(data, 0, dataLen));
            }
            // Otherwise, parse and dispatch the request
            logger.LogDebug("XmlRpcServerConnection::readRequest read {0} bytes.", dataLen);

            if (!header.ContentComplete)
            {
                return false;
            }
            connectionState = ServerConnectionState.WRITE_RESPONSE;

            return true; // Continue monitoring this source
        }

        private bool WriteResponse(string request)
        {
            string response = server.ExecuteRequest(request);
            if (response.Length == 0)
            {
                logger.LogError("XmlRpcServerConnection::WriteResponse: empty response.");
                return false;
            }
            try
            {
                MemoryStream memstream = new MemoryStream();
                using (StreamWriter writer = new StreamWriter(memstream))
                {
                    writer.Write(response);
                    bytesWritten = response.Length;
                }
                try
                {
                    var buffer = new ArraySegment<byte>();
                    memstream.TryGetBuffer(out buffer);
                    stream.Write(buffer.Array, buffer.Offset, buffer.Count);
                }
                catch (Exception ex)
                {
                    logger.LogError(string.Format("Exception while writing response: {0}", ex.Message));
                }
            }
            catch (Exception ex)
            {
                logger.LogError("XmlRpcServerConnection::WriteResponse: write error ({0}).", ex.Message);
                return false;
            }

            logger.LogDebug("XmlRpcServerConnection::WriteResponse: wrote {0} of {0} bytes.", bytesWritten, response.Length);

            // Prepare to read the next request
            if (bytesWritten == response.Length)
            {
                response = "";
                connectionState = ServerConnectionState.READ_HEADER;
            }

            return keepAlive; // Continue monitoring this source if true
        }
    }
}
