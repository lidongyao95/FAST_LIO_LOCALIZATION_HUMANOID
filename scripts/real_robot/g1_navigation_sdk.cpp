// Persistent SDK transport for an explicitly started web navigation session.
// Never changes locomotion mode. EOF, timeout and signals relinquish velocity.
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <poll.h>
#include <unistd.h>
#include <csignal>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

static volatile std::sig_atomic_t running = 1;
static void stop_signal(int) { running = 0; }
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    bool armed = false, sent = false;
    try {
        std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::g1::LocoClient client; client.Init(); client.SetTimeout(.2f);
        auto permitted = [&]() { int id=-1; return client.GetFsmId(id)==0 && id==501; };
        auto stop = [&]() {
            if (!sent) return true;
            // Do not overwrite a human takeover into another locomotion mode.
            return !permitted() || client.SetVelocity(0.f,0.f,0.f,.25f)==0;
        };
        std::cout << "READY" << std::endl;
        std::string pending;
        auto last=std::chrono::steady_clock::now();
        int result=0;
        while(running) {
            if(armed && std::chrono::steady_clock::now()-last>std::chrono::milliseconds(350)) {
                result=3; break;
            }
            pollfd fd{STDIN_FILENO,POLLIN,0};
            int ready=poll(&fd,1,50);
            if(ready<0) { if(!running) break; continue; }
            if(!ready) continue;
            char buffer[256]; auto count=read(STDIN_FILENO,buffer,sizeof(buffer));
            if(count<=0) break;
            pending.append(buffer,size_t(count));
            if(pending.size()>512) {result=2;break;}
            size_t end;
            while((end=pending.find('\n'))!=std::string::npos) {
                std::string line=pending.substr(0,end);pending.erase(0,end+1);
                if(line=="ARM") {
                    if(armed || !permitted()) {std::cout<<"REJECTED FSM_NOT_501"<<std::endl;return 4;}
                    armed=true;last=std::chrono::steady_clock::now();std::cout<<"OK"<<std::endl;
                } else if(line=="STOP") {
                    bool ok=stop();std::cout<<(ok?"OK":"STOP_FAILED")<<std::endl;return ok?0:5;
                } else {
                    std::istringstream input(line); std::string verb,extra;float x,y,w;
                    if(!armed || !(input>>verb>>x>>y>>w) || verb!="V" || (input>>extra) ||
                       !std::isfinite(x)||!std::isfinite(y)||!std::isfinite(w)||
                       std::abs(x)>.201f||std::abs(y)>.201f||std::abs(w)>.301f) {
                        stop();std::cout<<"REJECTED COMMAND"<<std::endl;return 2;
                    }
                    if(!permitted()) {std::cout<<"REVOKED"<<std::endl;return 4;}
                    sent=true;
                    if(client.SetVelocity(x,y,w,.25f)!=0) {stop();std::cout<<"SDK_FAILED"<<std::endl;return 5;}
                    last=std::chrono::steady_clock::now();std::cout<<"OK"<<std::endl;
                }
            }
        }
        if(!stop()) result=5;
        return result;
    } catch(const std::exception &error) {
        std::cerr<<"Navigation SDK: "<<error.what()<<std::endl;return 1;
    }
}
