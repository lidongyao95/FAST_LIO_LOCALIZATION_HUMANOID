#include <exception>
#include <iostream>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

int main(int argc, char** argv) {
    if (argc != 2 || argv[1][0] == '\0') {
        std::cerr << "Usage: g1_loco_preflight ROBOT_INTERFACE\n";
        return 2;
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::g1::LocoClient client;
        client.Init();
        client.SetTimeout(2.0f);

        int fsm_id = -1;
        int fsm_mode = -1;
        const auto id_result = client.GetFsmId(fsm_id);
        const auto mode_result = client.GetFsmMode(fsm_mode);
        std::cout << "fsm_id_result=" << id_result << " fsm_id=" << fsm_id
                  << " fsm_mode_result=" << mode_result << " fsm_mode=" << fsm_mode << '\n';
        return id_result == 0 && mode_result == 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "SDK preflight failed: " << error.what() << '\n';
        return 1;
    }
}
