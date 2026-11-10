#pragma once

#include <istream>
#include <ostream>
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

}  // namespace g1
