using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Uml.Robotics.Ros;

namespace Xamla.Robotics.Ros.Async
{
    class ServiceServerLinkAsync
        : IServiceServerLink
    {
        const int MAX_CALL_QUEUE_LENGTH = 8096;

        class CallInfo
        {
            public TaskCompletionSource<bool> Tcs { get; } = new TaskCompletionSource<bool>();
            public Task<bool> AsyncResult => this.Tcs.Task;

            public RosMessage Request { get; set; }
            public RosMessage Response { get; set; }
        }

        private ILogger Logger { get; } = ApplicationLogging.CreateLogger<IServiceServerLink>();

        private readonly object gate = new object();
        private ConnectionAsync connection;
        private AsyncQueue<CallInfo> callQueue = new AsyncQueue<CallInfo>(MAX_CALL_QUEUE_LENGTH);

        private string name;
        private readonly bool persistent;
        private CallInfo currentCall;

        private CancellationTokenSource cancellationTokenSource;
        private CancellationToken cancel;

        private IDictionary<string, string> headerValues;

        public ServiceServerLinkAsync(
            string name,
            bool persistent,
            string requestMd5Sum,
            string responseMd5Sum,
            IDictionary<string, string> headerValues
        )
        {
            this.name = name;
            this.persistent = persistent;
            this.RequestMd5Sum = requestMd5Sum;
            this.ResponseMd5Sum = responseMd5Sum;
            this.headerValues = headerValues;

            this.cancellationTokenSource = new CancellationTokenSource();
            this.cancel = cancellationTokenSource.Token;
        }

        public bool IsValid { get; private set; }
        public string RequestMd5Sum { get; private set; }
        public string RequestType { get; private set; }
        public string ResponseMd5Sum { get; private set; }
        public string ResponseType { get; private set; }

        public void Dispose()
        {
            connection?.Dispose();
            connection = null;
        }

        public void Initialize<MSrv>()
            where MSrv : RosService, new()
        {
            MSrv srv = new MSrv();
            RequestMd5Sum = srv.RequestMessage.MD5Sum();
            ResponseMd5Sum = srv.ResponseMessage.MD5Sum();
            RequestType = srv.RequestMessage.MessageType;
            ResponseType = srv.ResponseMessage.MessageType;
        }

        public void Initialize<MReq, MRes>()
            where MReq : RosMessage, new() where MRes : RosMessage, new()
        {
            MReq req = new MReq();
            MRes res = new MRes();
            RequestMd5Sum = req.MD5Sum();
            ResponseMd5Sum = res.MD5Sum();
            RequestType = req.MessageType;
            ResponseType = res.MessageType;
        }

        private async Task WriteHeader()
        {
            var header = new Dictionary<string, string>
            {
                ["service"] = name,
                ["md5sum"] = RosService.Generate(RequestType.Replace("__Request", "").Replace("__Response", "")).MD5Sum(),
                ["callerid"] = ThisNode.Name,
                ["persistent"] = persistent ? "1" : "0"
            };

            if (headerValues != null)
            {
                foreach (string o in headerValues.Keys)
                {
                    header[o] = headerValues[o];
                }
            }

            await this.connection.WriteHeader(header, cancel);
        }

        private async Task<IDictionary<string, string>> ReadHeader()
        {
            var remoteHeader = await this.connection.ReadHeader();

            if (!remoteHeader.TryGetValue("md5sum", out string md5sum))
            {
                string errorMessage = "TcpRos header from service server did not have required element: md5sum";
                ROS.Error()(errorMessage);
                throw new ConnectionError(errorMessage);
            }

            // TODO check md5sum

            return remoteHeader;
        }

        private async Task<IDictionary<string, string>> Handshake()
        {
            await WriteHeader();
            return await ReadHeader();
        }

        private async Task ProcessCall(CallInfo call)
        {
            RosMessage request = call.Request;
            request.Serialized = request.Serialize();
            connection.WriteHeader
            byte[] tosend = new byte[request.Serialized.Length + 4];

            Array.Copy(BitConverter.GetBytes(request.Serialized.Length), tosend, 4);
            Array.Copy(request.Serialized, 0, tosend, 4, request.Serialized.Length);
            connection.write(tosend, tosend.Length, onRequestWritten);
        }

        async Task HandleConnection()
        {
            await Handshake();
            IsValid = true;

            while (await callQueue.MoveNext(cancel))
            {
                currentCall = callQueue.Current;

            }
        }

        private void onConnectionDropped(Connection connection, Connection.DropReason reason)
        {
            if (connection != this.connection)
                throw new ArgumentException("Unknown connection", nameof(connection));

            Logger.LogDebug("Service client from [{0}] for [{1}] dropped", connection.RemoteString, name);

            ClearCalls();

            ServiceManager.Instance.RemoveServiceServerLinkAsync(this);

            IsValid = false;
        }

        private bool OnHeaderReceived(Connection conn, Header header)
        {
            string md5sum;
            if (header.Values.ContainsKey("md5sum"))
            {
                md5sum = header.Values["md5sum"];
            }
            else
            {
                var message = "TcpRos header from service server did not have required element: md5sum";
                ROS.Error()(message);
                Logger.LogError(message);
                return false;
            }

            //TODO check md5sum

            bool empty = false;
            lock (gate)
            {
                empty = (callQueue.Count == 0);
                if (empty)
                    headerRead = true;
            }

            IsValid = true;

            if (!empty)
            {
                processNextCall();
                headerRead = true;
            }

            return true;
        }

        private void CallFinished()
        {
            lock (gate)
            {
                currentCall.Tcs.TrySetResult(true);
                currentCall = null;
            }

            processNextCall();
        }

        private void processNextCall()
        {
            bool empty = false;
            lock (gate)
            {
                if (currentCall != null)
                    return;
                if (callQueue.Count > 0)
                {
                    currentCall = callQueue.Dequeue();
                }
                else
                    empty = true;
            }
            if (empty)
            {
                if (!persistent)
                {
                    connection.drop(Connection.DropReason.Destructing);
                }
            }
            else
            {
                RosMessage request;
                lock (call_queue_mutex)
                {
                    request = currentCall.req;
                }

                request.Serialized = request.Serialize();
                byte[] tosend = new byte[request.Serialized.Length + 4];
                Array.Copy(BitConverter.GetBytes(request.Serialized.Length), tosend, 4);
                Array.Copy(request.Serialized, 0, tosend, 4, request.Serialized.Length);
                connection.write(tosend, tosend.Length, onRequestWritten);
            }
        }

        private void ClearCalls()
        {
            CallInfo local_current;
            lock (gate)
            {
                local_current = currentCall;
            }

            if (local_current != null)
            {
                CancelCall(local_current);
            }

            lock (gate)
            {
                while (callQueue.Count > 0)
                {
                    CancelCall(callQueue.Dequeue());
                }
            }
        }

        private void CancelCall(CallInfo info)
        {
            info.Tcs.TrySetCanceled();
        }

        private bool OnHeaderWritten(Connection conn)
        {
            headerWritten = true;
            return true;
        }

        private bool OnRequestWritten(Connection conn)
        {
            Logger.LogInformation("onRequestWritten(Connection conn)");
            connection.read(5, onResponseOkAndLength);
            return true;
        }

        private bool onResponseOkAndLength(Connection conn, byte[] buf, int size, bool success)
        {
            if (conn != connection)
            {
                throw new ArgumentException("Unknown connection", nameof(conn));
            }

            if (size != 5)
            {
                throw new ArgumentException($"Wrong size {size}", nameof(size));
            }

            if (!success)
                return false;

            byte ok = buf[0];
            int len = BitConverter.ToInt32(buf, 1);
            int lengthLimit = 1000000000;
            if (len > lengthLimit)
            {
                ROS.Error()($"Message length exceeds limit of {lengthLimit}. Dropping connection.");
                Logger.LogError($"Message length exceeds limit of {lengthLimit}. Dropping connection.");
                connection.drop(Connection.DropReason.Destructing);
                return false;
            }

            lock (gate)
            {
                if (ok != 0)
                    currentCall.success = true;
                else
                    currentCall.success = false;
            }

            if (len > 0)
            {
                Logger.LogDebug($"Reading message with length of {len}.");
                connection.read(len, onResponse);
            }
            else
            {
                byte[] f = new byte[0];
                onResponse(conn, f, 0, true);
            }
            return true;
        }

        private bool onResponse(Connection conn, byte[] buf, int size, bool success)
        {
            if (conn != connection)
                throw new ArgumentException("Unknown connection", nameof(conn));

            if (!success)
                return false;

            lock (gate)
            {
                if (currentCall.success)
                {
                    if (currentCall.resp == null)
                        throw new NullReferenceException("Service response is null");
                    currentCall.resp.Serialized = buf;
                }
                else if (buf.Length > 0)
                {
                    // call failed with reason
                    currentCall.Tcs.TrySetException(new Exception(Encoding.UTF8.GetString(buf)));
                }
                else
                {
                    // call failed, but no reason is given
                }
            }

            CallFinished();
            return true;
        }

        public async Task<bool> Call(RosService srv)
        {
            (bool result, RosMessage response) = await Call(srv.RequestMessage);
            srv.ResponseMessage = response;
            return result;
        }

        public async Task<(bool, RosMessage)> Call(RosMessage req)
        {
            RosMessage response = RosMessage.Generate(req.MessageType.Replace("Request", "Response"));

            CallInfo info = new CallInfo { Request = req, Response = response };

            lock (gate)
            {
                if (connection.dropped)
                    return (false, null);

                callQueue.Enqueue(info);
            }

            try
            {
                bool success = await info.AsyncResult;
                if (success)
                {
                    // response is only sent on success
                    response.Deserialize(response.Serialized);
                }

                return (success, response);
            }
            catch (Exception e)
            {
                string message = $"Service call failed: service [{name}] responded with an error: {e.Message}";
                ROS.Error()(message);
                Logger.LogError(message);

                return (false, null);
            }
        }
    }
}
