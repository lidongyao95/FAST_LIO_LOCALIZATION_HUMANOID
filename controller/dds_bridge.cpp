#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <mutex>
#include <chrono>
using namespace unitree::robot;
static ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> state_pub;
static ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> cmd_sub;
static unitree_hg::msg::dds_::LowCmd_ cmd;
static std::mutex mx;
static std::chrono::steady_clock::time_point received;
extern "C" void bridge_init(int domain){
 ChannelFactory::Instance()->Init(domain,"lo");
 state_pub=std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");state_pub->InitChannel();
 cmd_sub=std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>("rt/lowcmd");
 cmd_sub->InitChannel([](const void* p){std::lock_guard<std::mutex> l(mx);cmd=*static_cast<const unitree_hg::msg::dds_::LowCmd_*>(p);received=std::chrono::steady_clock::now();},1);
}
extern "C" int bridge_step(const double* q,const double* dq,const double* quat,const double* gyro,const double* acc,double* torque,int tick){
 unitree_hg::msg::dds_::LowState_ state{};state.mode_machine()=5;state.tick()=tick;
 for(int i=0;i<29;i++){state.motor_state()[i].q()=q[i];state.motor_state()[i].dq()=dq[i];}
 for(int i=0;i<4;i++)state.imu_state().quaternion()[i]=quat[i];
 for(int i=0;i<3;i++){state.imu_state().gyroscope()[i]=gyro[i];state.imu_state().accelerometer()[i]=acc[i];}
 state_pub->Write(state);
 std::lock_guard<std::mutex> l(mx);
 bool fresh=std::chrono::duration<double>(std::chrono::steady_clock::now()-received).count()<.2;
 for(int i=0;i<29;i++){auto& m=cmd.motor_cmd()[i];torque[i]=fresh ? m.tau()+m.kp()*(m.q()-q[i])+m.kd()*(m.dq()-dq[i]):-3*dq[i];}
 return fresh;
}
