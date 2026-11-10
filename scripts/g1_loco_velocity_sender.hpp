#pragma once

#include <cmath>

namespace g1 {

// A failed send revokes this sender for the lifetime of the session.
template <typename LocoClient>
class LocoVelocitySender {
public:
    explicit LocoVelocitySender(LocoClient& client) : client_(client) {}

    bool send(float forward, float lateral, float yaw_rate) {
        if (faulted_) {
            return false;
        }
        if (!std::isfinite(forward) || !std::isfinite(lateral) ||
            !std::isfinite(yaw_rate) || std::abs(forward) > 0.30f ||
            lateral != 0.0f || std::abs(yaw_rate) > 0.25f) {
            faulted_ = true;
            stop();
            return false;
        }

        last_send_result_ = client_.SetVelocity(forward, lateral, yaw_rate, duration_seconds);
        if (last_send_result_ != 0) {
            faulted_ = true;
            stop();
        }
        return !faulted_;
    }

    bool stop() {
        last_stop_result_ = client_.SetVelocity(0.0f, 0.0f, 0.0f, duration_seconds);
        faulted_ = faulted_ || last_stop_result_ != 0;
        return last_stop_result_ == 0;
    }

    int last_send_result() const { return last_send_result_; }
    int last_stop_result() const { return last_stop_result_; }

private:
    static constexpr float duration_seconds = 0.30f;
    LocoClient& client_;
    bool faulted_ = false;
    int last_send_result_ = 0;
    int last_stop_result_ = 0;
};

}  // namespace g1
