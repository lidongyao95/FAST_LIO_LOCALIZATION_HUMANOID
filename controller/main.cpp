// Thin simulation adapter; policy, inference, actions and FSM are official upstream.
#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include <fstream>
#include <iomanip>
#include <atomic>
#include <cctype>
#include <mutex>
#include <signal.h>
std::unique_ptr<LowCmd_t> FSMState::lowcmd=nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate=nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard=nullptr;
static std::atomic<int> requested{1};
static std::atomic<int> actual{1};
static std::mutex cmd_mutex;
static std::vector<float> ros_velocity(3,0);
static std::vector<float> keyboard_velocity(3,0);
static double ros_cmd_time=0;
static double keyboard_until=0;
static bool ros_nav_active=false;
static std::atomic<bool> running{true};
static double now(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
namespace isaaclab {
REGISTER_OBSERVATION(control_velocity_commands){
 std::lock_guard<std::mutex> l(cmd_mutex);
 const double t=now();
 // Autonomous navigation has exclusive control. Outside navigation, a held
 // keyboard key takes priority for 0.25 s, then ROS/manual commands may resume.
 if(ros_nav_active && t-ros_cmd_time<.5)return ros_velocity;
 if(t<keyboard_until)return keyboard_velocity;
 if(t-ros_cmd_time<.5)return ros_velocity;
 return std::vector<float>(3,0);
}
REGISTER_OBSERVATION(control_gait_phase){
 const float period=params["period"].as<float>();
 env->global_phase=std::fmod(env->global_phase+env->step_dt/period,1.0f);
 auto cmd=control_velocity_commands(env,params);
 if(std::sqrt(cmd[0]*cmd[0]+cmd[1]*cmd[1]+cmd[2]*cmd[2])<.1f)return {0.f,0.f};
 return {std::sin(env->global_phase*2*M_PI),std::cos(env->global_phase*2*M_PI)};
}
}
int main(int argc,char** argv){
 if(argc!=4){std::cerr<<"g1_ctrl CONFIG DOMAIN COMMAND_FILE\n";return 2;}
 param::config=YAML::LoadFile(argv[1]);param::proj_dir=std::filesystem::absolute(argv[1]).parent_path();
 unitree::robot::ChannelFactory::Instance()->Init(std::stoi(argv[2]),"lo");
 FSMState::lowcmd=std::make_unique<LowCmd_t>();FSMState::lowstate=std::make_shared<LowState_t>();
 FSMState::lowstate->wait_for_connection();FSMState::lowcmd->msg_.mode_machine()=5;
 auto fsm=std::make_unique<CtrlFSM>(param::config["FSM"]);
 for(auto& state:fsm->states)for(int mode:{1,2,3})if(!state->isState(mode))state->registered_checks.insert(state->registered_checks.begin(),{[mode]{return requested.load()==mode;},mode});
 for(auto& state:fsm->states){int id=state->getState();state->registered_checks.insert(state->registered_checks.begin(),{[id]{actual=id;return false;},0});}
 signal(SIGINT,[](int){running=false;});signal(SIGTERM,[](int){running=false;});
 const std::string status_name="status-"+std::string(argv[2])+".txt";
 const std::string status_tmp_name=status_name+".tmp-"+std::to_string(getpid());
 if(isatty(STDIN_FILENO)){
  FSMState::keyboard=std::make_shared<Keyboard>();
  std::cout<<"Keyboard control enabled:\n"
           <<"  W/S forward/back, A/D lateral, Q/E turn, Space stop\n"
           <<"  1 Passive, 2 FixStand, 3 Velocity, Ctrl+C exit\n";
 }else{
  std::cout<<"Keyboard control disabled: stdin is not a TTY; ROS velocity control remains active\n";
 }
 fsm->start();
 int previous_mode=0;
 while(running){
  std::ifstream f(argv[3]);double t;int mode,source=0;float x,y,w;
  if(f>>t>>mode>>x>>y>>w){
   f>>source;
   if(mode>=1&&mode<=3){if(mode!=previous_mode){requested=mode;previous_mode=mode;}
    std::lock_guard<std::mutex> l(cmd_mutex);
    ros_velocity={std::clamp(x,-.3f,.3f),std::clamp(y,-.15f,.15f),std::clamp(w,-.4f,.4f)};
    ros_cmd_time=t;ros_nav_active=source==1;}
  }
  if(FSMState::keyboard){
   std::string key=FSMState::keyboard->key();
   if(!key.empty()){
    char c=static_cast<char>(std::tolower(static_cast<unsigned char>(key[0])));
    bool nav_active;
    {std::lock_guard<std::mutex> l(cmd_mutex);nav_active=ros_nav_active;}
    if(!nav_active){
     if(c=='1')requested=1;
     else if(c=='2')requested=2;
     else if(c=='3' && actual.load()==2)requested=3;
     else{
      std::vector<float> v(3,0);
      if(c=='w')v={.15f,0,0};else if(c=='s')v={-.12f,0,0};
      else if(c=='a')v={0,.15f,0};else if(c=='d')v={0,-.15f,0};
      else if(c=='q')v={0,0,.2f};else if(c=='e')v={0,0,-.2f};
      std::lock_guard<std::mutex> l(cmd_mutex);
      keyboard_velocity=v;keyboard_until=now()+.25;
     }
    }
   }
  }
  if(requested.load()==actual.load())requested=0;
  auto status_tmp=param::proj_dir/status_tmp_name;
  {std::ofstream status(status_tmp);status<<std::fixed<<std::setprecision(6)<<now()<<" "<<actual.load()<<"\n";}
  std::filesystem::rename(status_tmp,param::proj_dir/status_name);
  usleep(10000);
 }
 // Upstream FSM owns recurrent threads; process teardown stops them together.
 _Exit(0);
}
