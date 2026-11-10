#include <exception>
#include <iostream>
#include <string>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include "g1_loco_sender_session.hpp"

int main(int argc, char** argv) {
    if (argc != 3 || std::string(argv[1]) != "--zero-only" || !argv[2][0]) {
        std::cerr << "Usage: g1_loco_zero_session --zero-only ROBOT_INTERFACE\n";
        return 2;
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[2]);
        unitree::robot::g1::LocoClient client;
        client.Init();
        client.SetTimeout(0.2f);
        return g1::run_zero_session(client, std::cin, std::cout);
    } catch (const std::exception& error) {
        std::cerr << "SDK zero-speed session failed: " << error.what() << '\n';
        return 1;
    }
}
