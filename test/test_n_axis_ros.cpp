#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <thread>
#include "controller_interface/controller_interface_params.hpp"
#include "bxi_hardware/n_axis_trajectory_controller.hpp"
namespace bxi_hardware {
struct NAxisTestAccess {
  static void holding(NAxisTrajectoryController & c) {c.elapsed_ = 35.1;}
  static uint64_t sequence(NAxisTrajectoryController & c) {
    std::lock_guard<std::mutex> lock(c.input_mutex_); return c.input_heart_.sequence;
  }
};
namespace {
using C=NAxisTrajectoryController; using CB=controller_interface::CallbackReturn; using Ret=controller_interface::return_type;
class Fake : public C {
public:
  int64_t now=2000000000; size_t n=2; std::array<double,16> command{};
protected:
  int64_t steadyNow() const noexcept override {return now;}
  bool setCommand(size_t i,double v) override {command[i]=v;return true;}
  std::optional<double> getCommand(size_t i) override {return command[i];}
  std::optional<double> getState(size_t i) override {return i==n*3?command[n*5]:0.;}
};
struct Rig {
  std::string dir;
  std::array<double,16> backing{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  Fake c;
  rclcpp::Node::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Client<srv::StartNAxis>::SharedPtr start;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish,stop;
  rclcpp::Publisher<msg::NAxisKeepalive>::SharedPtr heart;
  rclcpp::Subscription<msg::NAxisStatus>::SharedPtr sub;
  msg::NAxisStatus::SharedPtr status;
  CB configured;
  explicit Rig(size_t n,int invalid=0) {
    auto pattern=(std::filesystem::temp_directory_path()/"bxi_ros_XXXXXX").string();
    const auto path=::mkdtemp(pattern.data()); if(!path)throw std::runtime_error("mkdtemp"); dir=path;
    c.n=n; std::vector<std::string> joints;for(size_t a=0;a<n;++a)joints.push_back("joint"+std::to_string(a+1));
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> params={rclcpp::Parameter("joints",joints),rclcpp::Parameter("record_seconds",1.)};
    if(invalid!=1)params.emplace_back("output_dir",invalid==2?"":invalid==3?dir+"/missing":invalid==4?dir+"/file":dir);
    if(invalid==4){std::ofstream(dir+"/file")<<"file";}
    if(invalid==5){std::filesystem::permissions(dir,std::filesystem::perms::owner_read|std::filesystem::perms::owner_exec);}
    options.parameter_overrides(params);
    controller_interface::ControllerInterfaceParams p;p.controller_name="n_axis_ros";p.controller_manager_update_rate=1000;p.update_rate=1000;p.node_options=options;
    EXPECT_EQ(c.init(p),Ret::OK);configured=c.on_configure(rclcpp_lifecycle::State());
    if(invalid)return;
    EXPECT_EQ(configured,CB::SUCCESS);
    commands.reserve(16);states.reserve(10);
    std::vector<hardware_interface::LoanedCommandInterface> cl;std::vector<hardware_interface::LoanedStateInterface> sl;
    for(const auto & key:c.command_interface_configuration().names){const auto j=key.find('/');commands.emplace_back(key.substr(0,j),key.substr(j+1),&backing[commands.size()]);cl.emplace_back(commands.back());}
    for(const auto & key:c.state_interface_configuration().names){const auto j=key.find('/');states.emplace_back(key.substr(0,j),key.substr(j+1),&backing[states.size()]);sl.emplace_back(states.back());}
    c.assign_interfaces(std::move(cl),std::move(sl));
    EXPECT_EQ(c.on_activate(rclcpp_lifecycle::State()),CB::SUCCESS);
    client=rclcpp::Node::make_shared("n_axis_test_client");
    const std::string prefix="/n_axis_ros/n_axis/";
    start=client->create_client<srv::StartNAxis>(prefix+"start");
    finish=client->create_client<std_srvs::srv::Trigger>(prefix+"finish");stop=client->create_client<std_srvs::srv::Trigger>(prefix+"stop");
    heart=client->create_publisher<msg::NAxisKeepalive>(prefix+"keepalive",rclcpp::QoS(1).reliable());
    sub=client->create_subscription<msg::NAxisStatus>(prefix+"status",rclcpp::QoS(1).reliable(),[this](msg::NAxisStatus::SharedPtr m){status=m;});
    executor.add_node(c.get_node()->get_node_base_interface());executor.add_node(client);
    EXPECT_TRUE(until([this]{return start->service_is_ready()&&finish->service_is_ready()&&stop->service_is_ready()&&heart->get_subscription_count()>0;}));
  }
  ~Rig(){
    if(client){executor.remove_node(client);executor.remove_node(c.get_node()->get_node_base_interface());}
    std::filesystem::permissions(dir,std::filesystem::perms::owner_all);std::filesystem::remove_all(dir);
  }
  template<class P> bool until(P p) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    do{executor.spin_some();if(p())return true;std::this_thread::sleep_for(std::chrono::milliseconds(2));}while(std::chrono::steady_clock::now()<end);
    return false;
  }
  void pump(){const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(60);while(std::chrono::steady_clock::now()<end){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));}}
  Ret step(){return c.update(rclcpp::Time(c.now),rclcpp::Duration::from_nanoseconds(1000000));}
  auto request(){auto r=std::make_shared<srv::StartNAxis::Request>();r->nonce=123;r->id=7;r->sent_ns=c.now;r->axes=c.n;r->amplitude=C::kPi/6;r->frequency=.2;r->cycles=5;r->ramp_cycles=1;r->hold=1;r->kp.fill(20);r->kd.fill(1);r->release=false;return r;}
  bool send(std::shared_ptr<srv::StartNAxis::Request> r){auto f=start->async_send_request(r);if(!until([&]{return f.wait_for(std::chrono::seconds(0))==std::future_status::ready;}))throw std::runtime_error("service timeout");return f.get()->accepted;}
  bool trigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr s){auto f=s->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());if(!until([&]{return f.wait_for(std::chrono::seconds(0))==std::future_status::ready;}))throw std::runtime_error("trigger timeout");return f.get()->success;}
  void heartbeat(uint64_t sequence,int64_t sent,uint64_t nonce=123,uint64_t id=7){msg::NAxisKeepalive m;m.sequence=sequence;m.sent_ns=sent;m.nonce=nonce;m.id=id;heart->publish(m);pump();}
  void boot(){ASSERT_EQ(step(),Ret::OK);ASSERT_TRUE(until([&]{return status&&status->state==1;}));}
};
class RosEntry : public ::testing::TestWithParam<size_t> {
public:static void SetUpTestSuite(){rclcpp::init(0,nullptr);}static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_P(RosEntry, OutputDirectoryMandatoryAndWritable){for(int invalid=1;invalid<=5;++invalid){Rig r(GetParam(),invalid);EXPECT_EQ(r.configured,CB::ERROR);}}
TEST_P(RosEntry, RequestValidationAndHeartbeatIdentityOrderTime){
  Rig r(GetParam());r.boot();
  auto bad=r.request();bad->axes=1;EXPECT_FALSE(r.send(bad));
  bad=r.request();bad->frequency=.21;EXPECT_FALSE(r.send(bad));
  bad=r.request();bad->nonce=0;EXPECT_FALSE(r.send(bad));
  bad=r.request();bad->sent_ns=r.c.now+1;EXPECT_FALSE(r.send(bad));
  bad=r.request();bad->sent_ns=r.c.now-C::kLeaseNs;EXPECT_FALSE(r.send(bad));
  ASSERT_TRUE(r.send(r.request()));EXPECT_FALSE(r.send(r.request()));EXPECT_FALSE(r.trigger(r.finish));
  ASSERT_EQ(r.step(),Ret::OK);r.heartbeat(1,r.c.now,124);r.heartbeat(1,r.c.now,123,8);
  EXPECT_EQ(NAxisTestAccess::sequence(r.c),0U);
  r.heartbeat(1,r.c.now);EXPECT_EQ(NAxisTestAccess::sequence(r.c),1U);ASSERT_EQ(r.step(),Ret::OK);
  r.c.now+=100000000;r.heartbeat(2,r.c.now);EXPECT_EQ(NAxisTestAccess::sequence(r.c),2U);
  r.heartbeat(2,r.c.now);r.heartbeat(1,r.c.now);r.heartbeat(3,r.c.now+1);r.heartbeat(3,r.c.now-C::kLeaseNs);r.heartbeat(3,r.c.now-1);
  EXPECT_EQ(NAxisTestAccess::sequence(r.c),2U);
  ASSERT_EQ(r.step(),Ret::OK);ASSERT_TRUE(r.until([&]{return r.status&&r.status->state==3;}));
}
TEST_P(RosEntry, SenderSilenceLeaseBoundaryAndSingleSave){
  for(int64_t offset:{0,1}){
    Rig r(GetParam());r.boot();ASSERT_TRUE(r.send(r.request()));r.heartbeat(1,r.c.now);ASSERT_EQ(r.step(),Ret::OK);
    const auto sent=r.c.now;r.c.now=sent+C::kLeaseNs-1;EXPECT_EQ(r.step(),Ret::OK);
    r.c.now=sent+C::kLeaseNs+offset;EXPECT_EQ(r.step(),Ret::ERROR);
    ASSERT_TRUE(r.until([&]{return r.status&&r.status->state==7&&r.status->record_state==1;}));
    const auto path=r.status->csv_path;EXPECT_TRUE(r.status->terminal);EXPECT_TRUE(r.status->record_complete);EXPECT_TRUE(std::filesystem::is_regular_file(path));
    EXPECT_NE(path.find("nonce_123"),std::string::npos);
    auto stamp=std::filesystem::last_write_time(path);r.pump();EXPECT_EQ(std::filesystem::last_write_time(path),stamp);
    EXPECT_FALSE(r.send(r.request()));EXPECT_EQ(r.step(),Ret::ERROR);
  }
}
TEST_P(RosEntry, FinishStopStatusAndSaveFailureNeverRetries){
  Rig r(GetParam());r.boot();ASSERT_TRUE(r.send(r.request()));r.heartbeat(1,r.c.now);ASSERT_EQ(r.step(),Ret::OK);
  NAxisTestAccess::holding(r.c);ASSERT_EQ(r.step(),Ret::OK);ASSERT_TRUE(r.until([&]{return r.status&&r.status->state==4;}));
  ASSERT_TRUE(r.trigger(r.finish));ASSERT_TRUE(r.trigger(r.stop));ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.until([&]{return r.status&&r.status->state==6;}));EXPECT_FALSE(r.status->record_complete);
  std::filesystem::remove_all(r.dir);EXPECT_EQ(r.c.on_deactivate(rclcpp_lifecycle::State()),CB::SUCCESS);
  ASSERT_TRUE(r.until([&]{return r.status&&r.status->record_state==2;}));const auto path=r.status->csv_path;
  std::filesystem::create_directory(r.dir);r.pump();EXPECT_EQ(r.status->record_state,2U);EXPECT_FALSE(std::filesystem::exists(path));
}
TEST_P(RosEntry, NormalFinishAndDeactivationSavesOutsideLifecycle){
  Rig r(GetParam());r.boot();ASSERT_TRUE(r.send(r.request()));r.heartbeat(1,r.c.now);ASSERT_EQ(r.step(),Ret::OK);
  NAxisTestAccess::holding(r.c);ASSERT_EQ(r.step(),Ret::OK);ASSERT_TRUE(r.trigger(r.finish));ASSERT_EQ(r.step(),Ret::OK);
  ASSERT_TRUE(r.until([&]{return r.status&&r.status->state==5;}));EXPECT_EQ(r.status->record_state,0U);
  EXPECT_EQ(r.c.on_deactivate(rclcpp_lifecycle::State()),CB::SUCCESS);
  ASSERT_TRUE(r.until([&]{return r.status&&r.status->record_state==1;}));EXPECT_TRUE(std::filesystem::exists(r.status->csv_path));
}
INSTANTIATE_TEST_SUITE_P(TwoAndThree,RosEntry,::testing::Values(2U,3U));
}}
