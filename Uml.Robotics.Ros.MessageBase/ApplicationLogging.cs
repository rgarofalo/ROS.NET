using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Console;

namespace Uml.Robotics.Ros
{
    public static class ApplicationLogging
    {
        private static ILoggerFactory loggerFactory;

        public static LogLevel ConsoleLogLevel { get; set; } = LogLevel.Information;

        public static ILoggerFactory LoggerFactory
        {
            get
            {
                lock (typeof(ApplicationLogging))
                {
                    if (loggerFactory == null)
                    {
                        loggerFactory = new LoggerFactory();
                        loggerFactory.AddProvider(
                            new ConsoleLoggerProvider((string text, LogLevel logLevel) => logLevel >= ConsoleLogLevel, true)
                        );
                    }
                    return loggerFactory;
                }
            }
            set
            {
                lock (typeof(ApplicationLogging))
                {
                    loggerFactory = value;
                }
            }
        }

        public static ILogger CreateLogger<T>() =>
            LoggerFactory.CreateLogger<T>();

        public static ILogger CreateLogger(string category) =>
            LoggerFactory.CreateLogger(category);
    }
}
