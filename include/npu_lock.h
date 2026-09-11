#ifndef NPU_LOCK_H
#define NPU_LOCK_H

// RV1126 rknn context 非线程安全: 两个线程各自持有独立 rknn_context 同时调用
// rknn_run 会让 NPU 驱动堆内存损坏 (在另一线程后续 Eigen free() 时崩溃)。
// 用一个全局互斥锁串行化所有 rknn_run / 推理会话, 避免并发 NPU 执行。
// 异步 ReID 仍独立于主流水线线程运行, 只是轮流占用 NPU。
#include <pthread.h>

inline pthread_mutex_t &npu_lock()
{
    static pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
    return m;
}

#endif // NPU_LOCK_H
