#include "coordinator.hpp"

#include <ranges>

#include "lifecycle_msgs/msg/state.hpp"

namespace coordinator
{

ControllerCoordinator::ControllerCoordinator()
: rclcpp::Node("controller_coordinator"),
  activate_hardware_request_(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>()),
  deactivate_hardware_request_(std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>()),
  activate_controllers_request_(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>()),
  deactivate_controllers_request_(std::make_shared<controller_manager_msgs::srv::SwitchController::Request>())
{
  param_listener_ = std::make_shared<controller_coordinator::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);

  auto wait_for_service = [this](const auto & client, const std::string & service_name) {
    RCLCPP_INFO(this->get_logger(), "Waiting for %s service to come up", service_name.c_str());  // NOLINT
    client->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Service available");  // NOLINT
  };

  const std::string list_controllers_name = "controller_manager/list_controllers";
  list_controllers_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
    list_controllers_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(list_controllers_client_, list_controllers_name);

  const std::string unload_controllers_name = "controller_manager/unload_controller";
  unload_controller_client_ = this->create_client<controller_manager_msgs::srv::UnloadController>(
    unload_controllers_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(unload_controller_client_, unload_controllers_name);

  const std::string hardware_service = "controller_manager/set_hardware_component_state";
  hardware_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
    hardware_service, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(hardware_client_, hardware_service);

  const std::string load_controller_name = "controller_manager/load_controller";
  load_controller_client_ = this->create_client<controller_manager_msgs::srv::LoadController>(
    load_controller_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(load_controller_client_, load_controller_name);

  const std::string configure_controller_name = "controller_manager/configure_controller";
  configure_controller_client_ = this->create_client<controller_manager_msgs::srv::ConfigureController>(
    configure_controller_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(configure_controller_client_, configure_controller_name);

  const std::string switch_controller_name = "controller_manager/switch_controller";
  switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
    switch_controller_name, rclcpp::ServicesQoS(), client_callback_group_);
  wait_for_service(switch_controller_client_, switch_controller_name);

  activate_hardware_request_->name = params_.thruster_hardware;
  activate_hardware_request_->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

  deactivate_hardware_request_->name = params_.thruster_hardware;
  deactivate_hardware_request_->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;

  // pre-configure the activate/deactivate service messages
  // these won't dynamically change, so we can set them up once
  activate_controllers_request_->activate_controllers = params_.load_sequence;
  activate_controllers_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  activate_controllers_request_->activate_asap = true;
  activate_controllers_request_->timeout = rclcpp::Duration::from_seconds(params_.activation_timeout);

  deactivate_controllers_request_->deactivate_controllers = params_.load_sequence;
  deactivate_controllers_request_->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  deactivate_controllers_request_->activate_asap = true;
  deactivate_controllers_request_->timeout = rclcpp::Duration::from_seconds(params_.activation_timeout);

  service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
  activate_system_service_ = this->create_service<std_srvs::srv::SetBool>(
    "~/activate",
    [this](
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      response->success = true;
      if (request->data) {
        RCLCPP_INFO(this->get_logger(), "Activating thruster hardware interface and controllers");  // NOLINT

        // activate the hardware interface
        hardware_client_->async_send_request(
          activate_hardware_request_,
          [this, &response](
            rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully activated thruster hardware interface");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to activate thruster hardware interface");  // NOLINT
              response->success = false;
              response->message = "Failed to activate thruster hardware interface";
            }
          });

        // activate the controllers
        switch_controller_client_->async_send_request(
          activate_controllers_request_,
          [this,
           &response](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully activated controllers");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to activate controllers");  // NOLINT
              response->success = false;
              response->message = "Failed to activate controllers";
            }
          });
      } else {
        RCLCPP_INFO(this->get_logger(), "Deactivating controllers and thruster hardware interface");  // NOLINT

        // deactivate the hardware interface
        hardware_client_->async_send_request(
          deactivate_hardware_request_,
          [this, &response](
            rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully deactivated thruster hardware interface");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to deactivate thruster hardware interface");  // NOLINT
              response->success = false;
              response->message = "Failed to deactivate thruster hardware interface";
            }
          });

        // deactivate the controllers
        switch_controller_client_->async_send_request(
          deactivate_controllers_request_,
          [this,
           &response](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture result_response) {
            const auto & result = result_response.get();
            if (result->ok) {
              RCLCPP_INFO(this->get_logger(), "Successfully deactivated controllers");  // NOLINT
            } else {
              RCLCPP_ERROR(this->get_logger(), "Failed to deactivate controllers");  // NOLINT
              response->success = false;
              response->message = "Failed to deactivate controllers";
            }
          });
      }
    },
    rclcpp::ServicesQoS(),
    service_callback_group_);

  // get the list of loaded controllers
  const auto list_request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request(list_request);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), list_future);
  const auto & list_result = list_future.get();

  // lambda used to deactivate a controller
  auto deactivate_controllers = [this](const std::vector<std::string> & controller_names) -> bool {
    const auto deactivate_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    deactivate_request->deactivate_controllers = controller_names;
    deactivate_request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    deactivate_request->activate_asap = true;
    deactivate_request->timeout = rclcpp::Duration::from_seconds(params_.activation_timeout);

    bool controllers_deactivated = false;
    for (int i = 0; i < params_.max_attempts; ++i) {
      auto deactivate_future = switch_controller_client_->async_send_request(deactivate_request);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), deactivate_future);
      if (deactivate_future.get()->ok) {
        controllers_deactivated = true;
        break;
      }
    }
    return controllers_deactivated;
  };

  // lambda used to unload a controller
  auto unload_controller = [this](const std::string & controller_name) -> bool {
    const auto unload_request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
    unload_request->name = controller_name;
    RCLCPP_DEBUG(this->get_logger(), "Unloading %s", controller_name.c_str());  // NOLINT

    bool controller_unloaded = false;
    for (int i = 0; i < params_.max_attempts; ++i) {
      auto unload_future = unload_controller_client_->async_send_request(unload_request);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), unload_future);
      if (unload_future.get()->ok) {
        controller_unloaded = true;
        break;
      }
    }
    return controller_unloaded;
  };

  // deactivate and unload all controllers that are currently active/loaded
  // we could be more intelligent about this by checking if controllers that we want to load are already loaded, but
  // this is an easy solution for an initial version
  auto active_names = list_result->controller | std::views::filter([](const auto & c) { return c.state == "active"; }) |
                      std::views::transform([](const auto & c) { return c.name; });
  const std::vector<std::string> active_controllers(active_names.begin(), active_names.end());

  if (!active_controllers.empty()) {
    RCLCPP_INFO(this->get_logger(), "Deactivating all active controllers");  // NOLINT
    if (!deactivate_controllers(active_controllers)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to deactivate all active controllers. Exiting");  // NOLINT
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully deactivated all active controllers");  // NOLINT
  }

  for (const auto & controller_state : list_result->controller) {
    if (!unload_controller(controller_state.name)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to unload %s. Exiting", controller_state.name.c_str());  // NOLINT
      rclcpp::shutdown();
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Successfully unloaded %s", controller_state.name.c_str());  // NOLINT
  }
  RCLCPP_INFO(this->get_logger(), "Unloaded all previously-loaded controllers");  // NOLINT

  // lambda used to load a controller
  auto load_controller = [this](const std::string & controller_name) -> bool {
    const auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    load_request->name = controller_name;
    RCLCPP_DEBUG(this->get_logger(), "Loading %s", controller_name.c_str());  // NOLINT

    bool controller_loaded = false;
    for (int i = 0; i < params_.max_attempts; ++i) {
      auto load_future = load_controller_client_->async_send_request(load_request);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), load_future);
      if (load_future.get()->ok) {
        controller_loaded = true;
        break;
      }
    }
    return controller_loaded;
  };

  // lambda used to configure a controller
  auto configure_controller = [this](const std::string & controller_name) -> bool {
    auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    configure_request->name = controller_name;
    RCLCPP_DEBUG(this->get_logger(), "Configuring %s", controller_name.c_str());  // NOLINT

    bool controller_configured = false;
    for (int i = 0; i < params_.max_attempts; ++i) {
      auto configure_future = configure_controller_client_->async_send_request(configure_request);
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), configure_future);
      if (configure_future.get()->ok) {
        controller_configured = true;
        break;
      }
    }
    return controller_configured;
  };

  // load the controllers and then configure them
  RCLCPP_INFO(this->get_logger(), "Loading and configuring controllers");  // NOLINT
  for (const auto & controller_name : params_.load_sequence) {
    if (!load_controller(controller_name)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load %s. Exiting", controller_name.c_str());  // NOLINT
      rclcpp::shutdown();
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Successfully loaded %s", controller_name.c_str());  // NOLINT

    if (!configure_controller(controller_name)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure %s. Exiting", controller_name.c_str());  // NOLINT
      rclcpp::shutdown();
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Successfully configured %s", controller_name.c_str());  // NOLINT
  }
  RCLCPP_INFO(this->get_logger(), "Successfully loaded and configured all controllers");  // NOLINT
}

}  // namespace coordinator

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<coordinator::ControllerCoordinator>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
