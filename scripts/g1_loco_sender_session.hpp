#pragma once

#include <istream>
#include <ostream>
#include <sstream>
#include <string>
#include "g1_loco_velocity_sender.hpp"

namespace g1 {

// First hardware session accepts only explicit zero-speed requests.
template <typename Client>
int run_zero_session(Client& client, std::istream& input, std::ostream& output) {
    LocoVelocitySender<Client> sender(client);
    std::string line;
    while (std::getline(input, line)) {
        if (line == "quit") {
            return 0;
        }
        if (line != "stop") {
            output << "rejected" << std::endl;
            return 2;
        }
        const bool stopped = sender.stop();
        output << "stop_result=" << sender.last_stop_result() << std::endl;
        if (!stopped) {
            return 1;
        }
    }
    return input.eof() ? 0 : 1;
}

// The caller owns the command watchdog and closes input on loss of ownership.
template <typename Client>
int run_velocity_session(Client& client, std::istream& input, std::ostream& output) {
    LocoVelocitySender<Client> sender(client);
    std::string line;
    while (std::getline(input, line)) {
        int fsm_id = -1;
        if (client.GetFsmId(fsm_id) != 0) {
            sender.stop();
            output << "fsm_query_failed" << std::endl;
            return 1;
        }
        // Only the walking state observed in the PC2 bring-up owns SDK velocity.
        if (fsm_id != 802) {
            output << "control_revoked" << std::endl;
            return 1;
        }
        if (line == "stop") {
            const bool stopped = sender.stop();
            output << "stop_result=" << sender.last_stop_result() << std::endl;
            return stopped ? 0 : 1;
        }

        std::istringstream command(line);
        std::string verb;
        float forward = 0.0f;
        float lateral = 0.0f;
        float yaw_rate = 0.0f;
        std::string extra;
        if (!(command >> verb >> forward >> lateral >> yaw_rate) ||
            verb != "velocity" || command >> extra) {
            sender.stop();
            output << "rejected" << std::endl;
            return 2;
        }
        if (!sender.send(forward, lateral, yaw_rate)) {
            output << "send_failed" << std::endl;
            return 1;
        }
        output << "send_result=0" << std::endl;
    }
    int fsm_id = -1;
    if (client.GetFsmId(fsm_id) == 0 && fsm_id != 802) {
        return 1;
    }
    return sender.stop() && input.eof() ? 0 : 1;
}

}  // namespace g1
