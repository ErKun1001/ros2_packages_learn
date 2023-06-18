#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <cinttypes>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

// which node to handle
static constexpr char const * lifecycle_node = "lc_talker";

// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// In this demo, we use get_state and change_state
// and thus the two service topics are:
// lc_talker/get_state
// lc_talker/change_state
static constexpr char const * node_get_state_topic = "lc_talker/get_state";
static constexpr char const * node_change_state_topic = "lc_talker/change_state";

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do 
  {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } 
  while (rclcpp::ok() && status != std::future_status::ready);

  return status;
}

class LifecycleServiceClient : public rclcpp::Node
{
public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : Node(node_name)
  {}

  void
  init()
  {
    // Every lifecycle node spawns automatically a couple
    // of services which allow an external interaction with
    // these nodes.
    // The two main important ones are GetState and ChangeState.
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
    if(!client_get_state_)
    {
      RCLCPP_ERROR(get_logger(),"client_get_state_ creat failed");
    }
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
  }

  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  unsigned int
  get_state(std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) 
    {
      RCLCPP_ERROR( get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto futureid_result = client_get_state_->async_send_request(request);

      RCLCPP_INFO( get_logger(), "Service %s request id %" PRId64 ".", 
      client_get_state_->get_service_name(), futureid_result.request_id);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(futureid_result.future, time_out);


    if (future_status != std::future_status::ready) 
    {
      RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    auto fut = futureid_result.share();
    // We have an succesful answer. So let's print the current state.
    if (fut.get()) 
    {
      auto sta = fut.get();
      RCLCPP_INFO(get_logger(), "Node %s has current state %s.",
        lifecycle_node, sta->current_state.label.c_str());
      return sta->current_state.id;
    } 
    return 0;
  }

  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        get_logger(),
        "Service %s is not available.",
        client_change_state_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(
        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
        get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
bool
callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
  rclcpp::WallRate time_between_state_changes(0.1);  // 10s

  // configure
  {
    RCLCPP_INFO(rclcpp::get_logger("syk"), "start change to confighre");
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) 
    {
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("syk"), "start get cur state after change to configure");
    if (!lc_client->get_state()) 
    {
      RCLCPP_ERROR(rclcpp::get_logger("syk"), "get cur state filaed after change to configure");
      return false;
    }
  }

  // activate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }

  // deactivate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }

  // activate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }

  // and deactivate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }

  // we cleanup
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }

  // and finally shutdown
  // Note: We have to be precise here on which shutdown transition id to call
  // We are currently in the unconfigured state and thus have to call
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return false;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    {
      return false;
    }
    if (!lc_client->get_state()) {
      return false;
    }
  }
    return true;
}

void
wake_executor(std::shared_future<bool> future, rclcpp::executors::SingleThreadedExecutor & exec)
{
  future.wait();

  RCLCPP_INFO(rclcpp::get_logger("syk"), "%s future result %d",__func__, future.get());
  // Wake the executor when the script is done
  // https://github.com/ros2/rclcpp/issues/1916
  exec.cancel();
}

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();
    
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<bool> script = std::async(
    std::launch::async,
    std::bind(callee_script, lc_client));

  auto wake_exec = std::async(
    std::launch::async,
    std::bind(wake_executor, script, std::ref(exe)));

    //callee_script(lc_client);
  RCLCPP_INFO(rclcpp::get_logger("syk"), "rclcpp ok %d",rclcpp::ok());
  exe.spin_until_future_complete(wake_exec);
   // exe.spin();
  RCLCPP_INFO(rclcpp::get_logger("syk"), "after rclcpp ok %d",rclcpp::ok());
  RCLCPP_INFO(rclcpp::get_logger("syk"), "shutdowm !!!");
  rclcpp::shutdown();

  return 0;
}