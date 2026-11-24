// Read or set the G1 locomotion FSM id/state. Read-only unless 'set' is given.
// Build: see build_navigation_sdk.sh (same includes/libs), output name g1_fsm_tool.
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

int main(int argc, char** argv) {
    if (argc < 3 || argv[1][0] == '\0') {
        std::cerr << "Usage: g1_fsm_tool ROBOT_INTERFACE get | set FSM_ID\n";
        return 2;
    }
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::g1::LocoClient client;
        client.Init();
        client.SetTimeout(2.0f);

        int fsm_id = -1, fsm_mode = -1;
        if (std::strcmp(argv[2], "get") == 0) {
            const auto id_result = client.GetFsmId(fsm_id);
            const auto mode_result = client.GetFsmMode(fsm_mode);
            std::cout << "fsm_id_result=" << id_result << " fsm_id=" << fsm_id
                      << " fsm_mode_result=" << mode_result << " fsm_mode=" << fsm_mode << '\n';
            return id_result == 0 && mode_result == 0 ? 0 : 1;
        }
        if (std::strcmp(argv[2], "set") == 0 && argc == 4) {
            const int target = std::atoi(argv[3]);
            client.GetFsmId(fsm_id);
            const auto mode_result = client.GetFsmMode(fsm_mode);
            std::cout << "before fsm_id=" << fsm_id << " fsm_mode=" << fsm_mode
                      << " (mode_result=" << mode_result << ")\n";
            const auto set_result = client.SetFsmId(target);
            std::cout << "SetFsmId(" << target << ")=" << set_result << '\n';
            for (int attempt = 0; attempt < 10; ++attempt) {
                int current = -1;
                client.GetFsmId(current);
                std::cout << "readback[" << attempt << "] fsm_id=" << current << '\n';
                if (current == target) return 0;
            }
            return set_result == 0 ? 3 : 1;
        }
        std::cerr << "Usage: g1_fsm_tool ROBOT_INTERFACE get | set FSM_ID\n";
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "fsm tool failed: " << error.what() << '\n';
        return 1;
    }
}
