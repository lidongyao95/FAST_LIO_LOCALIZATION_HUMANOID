// Minimal G1 velocity probe: drives the loco client without ROS so SDK issues
// can be separated from the ROS/Nav2 chain.
//
// Usage: g1_loco_velocity_probe ROBOT_INTERFACE [velocity|move|fsm]
//   velocity (default)  SetVelocity(0.3, 0, 0, 0.2) twenty times, then StopMove
//   move                Move(0.3, 0, 0) twenty times, then StopMove
//   fsm                 SetFsmId(802) first, then the velocity sequence
// Each step prints the SDK return code; the body moves for about two seconds.
//
// Build (aarch64, unitree_sdk2 checkout in $HOME/unitree_sdk2):
//   S=$HOME/unitree_sdk2
//   g++ -O2 -std=c++17 -I$S/include -I$S/thirdparty/include/ddscxx \
//     -o /tmp/g1-loco-build/g1_loco_velocity_probe scripts/g1_loco_velocity_probe.cpp \
//     -L$S/lib/aarch64 -lunitree_sdk2 -L$S/thirdparty/lib/aarch64 \
//     -lddscxx -lddsc -lpthread -Wl,-rpath,$S/thirdparty/lib/aarch64
// Run it without sourcing ROS: Foxy's CycloneDDS 0.7 lacks ddsi_sertype_v0.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <unitree/robot/g1/loco/g1_loco_client.hpp>

int main(int argc, char** argv) {
    const char* iface = argc > 1 ? argv[1] : "eth0";
    const char* mode = argc > 2 ? argv[2] : "velocity";
    unitree::robot::ChannelFactory::Instance()->Init(0, iface);
    unitree::robot::g1::LocoClient client;
    client.Init();
    client.SetTimeout(0.5f);

    int fsm = -1;
    const int fsm_result = client.GetFsmId(fsm);
    std::printf("fsm_result=%d fsm_id=%d mode=%s\n", fsm_result, fsm, mode);
    std::fflush(stdout);

    if (std::strcmp(mode, "fsm") == 0) {
        const int set_result = client.SetFsmId(802);
        std::printf("SetFsmId(802)=%d\n", set_result);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    for (int i = 0; i < 20; ++i) {
        const int rc = std::strcmp(mode, "move") == 0
                           ? client.Move(0.3f, 0.0f, 0.0f)
                           : client.SetVelocity(0.3f, 0.0f, 0.0f, 0.2f);
        if (i < 3 || i == 19) {
            std::printf("i=%d rc=%d\n", i, rc);
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const int stop_result = client.StopMove();
    std::printf("StopMove=%d\n", stop_result);
    std::fflush(stdout);
    return 0;
}
