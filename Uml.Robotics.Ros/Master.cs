using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Uml.Robotics.XmlRpc;

namespace Uml.Robotics.Ros
{
    public static class Master
    {
        private static readonly ILogger logger = ApplicationLogging.CreateLogger(nameof(Master));
        private static int port;
        private static string host;
        private static string uri;

        public static TimeSpan RetryTimeout = TimeSpan.FromSeconds(5);

        public static void Init(IDictionary<string, string> remappingArgs)
        {
            uri = string.Empty;
            if (remappingArgs.ContainsKey("__master"))
            {
                uri = (string) remappingArgs["__master"];
                ROS.ROS_MASTER_URI = uri;
            }
            if (string.IsNullOrEmpty(uri))
                uri = ROS.ROS_MASTER_URI;
            if (!Network.SplitUri(uri, out host, out port))
            {
                port = 11311;
            }
        }

        /// <summary>
        ///     Check if ROS master is running by querying the PID of the master process.
        /// </summary>
        /// <returns></returns>
        public static async Task<bool> Check()
        {
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            return await ExecuteAsync("getPid", args, result, payload, false);
        }

        /// <summary>
        ///     Gets all currently published and subscribed topics and adds them to the topic list
        /// </summary>
        /// <param name="topics"> List to store topics</param>
        /// <returns></returns>
        public static async Task<IList<TopicInfo>> GetTopics()
        {
            List<TopicInfo> topics = new List<TopicInfo>();
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);
            args.Set(1, "");

            if (!await ExecuteAsync("getPublishedTopics", args, result, payload, true))
            {
                throw new Exception("getPublishedTopics failed");
            }

            topics.Clear();
            for (int i = 0; i < payload.Count; i++)
                topics.Add(new TopicInfo(payload[i][0].GetString(), payload[i][1].GetString()));
            return topics;
        }

        /// <summary>
        ///     Gets all currently existing nodes and adds them to the nodes list
        /// </summary>
        /// <param name="nodes">List to store nodes</param>
        /// <returns></returns>
        public static async Task<IList<string>> GetNodes()
        {
            List<string> names = new List<string>();
            XmlRpcValue args = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
            args.Set(0, ThisNode.Name);

            if (!await ExecuteAsync("getSystemState", args, result, payload, true))
            {
                throw new Exception("getSystemState failed");
            }

            for (int i = 0; i < payload.Count; i++)
            {
                for (int j = 0; j < payload[i].Count; j++)
                {
                    XmlRpcValue val = payload[i][j][1];
                    for (int k = 0; k < val.Count; k++)
                    {
                        string name = val[k].GetString();
                        names.Add(name);
                    }
                }
            }
            return names;
        }

        internal static async Task<XmlRpcClient> ClientForNode(string nodeName)
        {
            var args = new XmlRpcValue(ThisNode.Name, nodeName);
            var resp = new XmlRpcValue();
            var payl = new XmlRpcValue();

            if (!await ExecuteAsync("lookupNode", args, resp, payl, true))
                return null;

            if (!XmlRpcManager.Instance.ValidateXmlRpcResponse("lookupNode", resp, payl))
                return null;

            string nodeUri = payl.GetString();
            if (!Network.SplitUri(nodeUri, out string nodeHost, out int nodePort) || nodeHost == null || nodePort <= 0)
                return null;

            return new XmlRpcClient(nodeHost, nodePort);
        }

        public static async Task<bool> Kill(string node)
        {
            var cl = await ClientForNode(node);
            if (cl == null)
                return false;

            XmlRpcValue req = new XmlRpcValue(), resp = new XmlRpcValue(), payl = new XmlRpcValue();
            req.Set(0, ThisNode.Name);
            req.Set(1, $"Node '{ThisNode.Name}' requests shutdown.");
            var respose = await cl.ExecuteAsync("shutdown", req);
            if (!respose.Success || !XmlRpcManager.Instance.ValidateXmlRpcResponse("shutdown", respose.Value, payl))
                return false;

            return true;
        }

        /// <summary>
        /// Execute a remote procedure call on the ROS master.
        /// </summary>
        /// <param name="method"></param>
        /// <param name="request">Full request to send to the master </param>
        /// <param name="waitForMaster">If you recieve an unseccessful status code, keep retrying.</param>
        /// <param name="response">Full response including status code and status message. Initially empty.</param>
        /// <param name="payload">Location to store the actual data requested, if any.</param>
        /// <returns></returns>
        public static async Task<bool> ExecuteAsync(string method, XmlRpcValue request, XmlRpcValue response, XmlRpcValue payload, bool waitForMaster)
        {
            bool supprressWarning = false;
            var startTime = DateTime.UtcNow;

            try
            {
                var client = new XmlRpcClient(host, port);

                while (true)
                {
                    // check if we are shutting down
                    if (XmlRpcManager.Instance.IsShuttingDown)
                        return false;

                    try
                    {
                        var result = await client.ExecuteAsync(method, request);           // execute the RPC call
                        response.Set(result.Value);
                        if (result.Success)
                        {
                            // validateXmlrpcResponse logs error in case of validation error (we don't need any logging here.)
                            return XmlRpcManager.Instance.ValidateXmlRpcResponse(method, result.Value, payload);
                        }
                        else
                        {
                            if (response.IsArray && response.Count >= 2)
                                logger.LogError("Execute failed: return={0}, desc={1}", response[0].GetInt(), response[1].GetString());
                            else
                                logger.LogError("response type: " + response.Type.ToString());
                        }
                    }
                    catch (Exception ex)
                    {
                        // no connection to ROS Master
                        if (waitForMaster)
                        {
                            if (!supprressWarning)
                            {
                                logger.LogWarning(
                                    $"[{method}] Could not connect to master at [{host}:{port}]. Retrying for the next {RetryTimeout.TotalSeconds} seconds."
                                );
                                supprressWarning = true;
                            }

                            // timeout expired, throw exception
                            if (RetryTimeout.TotalSeconds > 0 && DateTime.UtcNow.Subtract(startTime) > RetryTimeout)
                            {
                                logger.LogError("[{0}] Timed out trying to connect to the master [{1}:{2}] after [{1}] seconds",
                                                method, host, port, RetryTimeout.TotalSeconds);

                                throw new RosException($"Cannot connect to ROS Master at {host}:{port}", ex);
                            }
                        }
                        else
                        {
                            throw new RosException($"Cannot connect to ROS Master at {host}:{port}", ex);
                        }
                    }

                    await Task.Delay(250);

                    // recreate the client and reinitiate master connection
                    client = new XmlRpcClient(host, port);
                }
            }
            catch (ArgumentNullException e)
            {
                logger.LogError(e, e.Message);
            }

            logger.LogError("Master API call: {0} failed!\n\tRequest:\n{1}", method, request);

            return false;
        }

        /// <summary>
        /// </summary>
        /// <param name="method"></param>
        /// <param name="request">Full request to send to the master </param>
        /// <param name="waitForMaster">If you recieve an unseccessful status code, keep retrying.</param>
        /// <param name="response">Full response including status code and status message. Initially empty.</param>
        /// <param name="payload">Location to store the actual data requested, if any.</param>
        /// <returns></returns>
        public static bool Execute(string method, XmlRpcValue request, XmlRpcValue response, XmlRpcValue payload, bool waitForMaster)
        {
            return ExecuteAsync(method, request, response, payload, waitForMaster).Result;
        }
    }
}
