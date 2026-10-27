#ifndef GENTO_TRAJECTORY_UPLOAD_WAIT_H
#define GENTO_TRAJECTORY_UPLOAD_WAIT_H

#include <chrono>

namespace gento_upload {
enum class WaitResult { confirmed, timeout, interrupted };

// Dependencies are injected so deadline boundaries can be tested without hardware.
template <typename Confirmed, typename Healthy, typename Clock, typename Pause>
WaitResult wait_for_confirmation(Confirmed confirmed, Healthy healthy,
                                 Clock now, Pause pause, std::chrono::milliseconds timeout)
{
    const auto deadline = now() + timeout;
    for (;;) {
        if (!healthy()) return WaitResult::interrupted;
        // Always inspect the latest confirmation before declaring a timeout.
        if (confirmed()) return WaitResult::confirmed;
        if (now() >= deadline) return WaitResult::timeout;
        pause();
    }
}
}
#endif
