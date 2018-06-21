using System.Threading.Tasks;

namespace Xamla.Robotics.Ros.Async
{
    public static class TaskConstants
    {
        public static readonly Task Never = new TaskCompletionSource<object>().Task;
        public static readonly Task Canceled = CanceledTask<object>();
        public static readonly Task Completed = Task.FromResult<object>(null);
        public static readonly Task<bool> True = Task.FromResult<bool>(true);
        public static readonly Task<bool> False = Task.FromResult<bool>(false);

        public static Task<T> CanceledTask<T>()
        {
            var tcs = new TaskCompletionSource<T>();
            tcs.SetCanceled();
            return tcs.Task;
        }
    }
}
