#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <ppbng_interfaces/srv/confirm_dark_ready.hpp>
#include <ppbng_interfaces/srv/confirm_sample_ready.hpp>
#include <ppbng_interfaces/srv/start_acquisition.hpp>
#include <ppbng_interfaces/srv/stop_acquisition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
using namespace std::chrono_literals;

std::vector<std::string> split_tabs(const std::string & input)
{
  std::vector<std::string> values;
  std::size_t start{};
  while (start <= input.size()) {
    const auto delimiter = input.find('\t', start);
    values.push_back(input.substr(start, delimiter - start));
    if (delimiter == std::string::npos) break;
    start = delimiter + 1U;
  }
  return values;
}

std::string clean(std::string value)
{
  for (auto & character : value) {
    if (character == '\r' || character == '\n') character = ' ';
  }
  return value;
}

void send_all(const SOCKET socket, const std::string & text)
{
  std::size_t offset{};
  while (offset < text.size()) {
    const auto sent = send(socket, text.data() + offset,
      static_cast<int>(text.size() - offset), 0);
    if (sent == SOCKET_ERROR || sent == 0) return;
    offset += static_cast<std::size_t>(sent);
  }
}
}  // namespace

class ProductionControlBridge final : public rclcpp::Node
{
public:
  ProductionControlBridge()
  : Node("production_control_bridge")
  {
    const auto port = declare_parameter<int>("port", 45847);
    if (port < 1024 || port > 65535) throw std::runtime_error("invalid bridge port");
    port_ = static_cast<std::uint16_t>(port);
    start_client_ = create_client<ppbng_interfaces::srv::StartAcquisition>("/acquisition/start");
    dark_client_ = create_client<ppbng_interfaces::srv::ConfirmDarkReady>(
      "/acquisition/confirm_dark_ready");
    sample_client_ = create_client<ppbng_interfaces::srv::ConfirmSampleReady>(
      "/acquisition/confirm_sample_ready");
    stop_client_ = create_client<ppbng_interfaces::srv::StopAcquisition>("/acquisition/stop");
    status_client_ = create_client<std_srvs::srv::Trigger>("/acquisition/status_query");
    server_thread_ = std::thread([this]() {serve();});
  }

  ~ProductionControlBridge() override
  {
    stopping_ = true;
    const auto socket = listen_socket_.exchange(INVALID_SOCKET);
    if (socket != INVALID_SOCKET) closesocket(socket);
    if (server_thread_.joinable()) server_thread_.join();
  }

private:
  template<typename ServiceT>
  typename ServiceT::Response::SharedPtr call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request)
  {
    if (!client->wait_for_service(90s)) throw std::runtime_error("ROS service unavailable");
    auto future = client->async_send_request(request);
    if (future.wait_for(180s) != std::future_status::ready) {
      throw std::runtime_error("ROS service response timeout");
    }
    return future.get();
  }

  static void common_response(
    std::ostringstream & output, const bool accepted, const bool duplicate,
    const std::string & session_id, const std::string & message)
  {
    output << "accepted=" << (accepted ? "true" : "false") << '\n';
    output << "duplicate_request=" << (duplicate ? "true" : "false") << '\n';
    output << "session_id=" << clean(session_id) << '\n';
    output << "message=" << clean(message) << '\n';
  }

  std::string execute(const std::vector<std::string> & fields)
  {
    if (fields.empty()) throw std::runtime_error("empty command");
    std::ostringstream output;
    if (fields[0] == "status") {
      if (fields.size() != 1U) throw std::runtime_error("status takes no arguments");
      const auto response = call<std_srvs::srv::Trigger>(status_client_,
        std::make_shared<std_srvs::srv::Trigger::Request>());
      output << "success=" << (response->success ? "true" : "false") << '\n';
      output << "message=" << clean(response->message) << '\n';
    } else if (fields[0] == "start") {
      if (fields.size() != 3U) throw std::runtime_error("start requires request and dataset");
      auto request = std::make_shared<ppbng_interfaces::srv::StartAcquisition::Request>();
      request->request_id = fields[1];
      request->dataset_name = fields[2];
      request->force_degraded = false;
      const auto response = call<ppbng_interfaces::srv::StartAcquisition>(start_client_, request);
      common_response(output, response->accepted, response->duplicate_request,
        response->session_id, response->message);
      output << "session_directory=" << clean(response->session_directory) << '\n';
    } else if (fields[0] == "dark") {
      if (fields.size() != 2U) throw std::runtime_error("dark requires request id");
      auto request = std::make_shared<ppbng_interfaces::srv::ConfirmDarkReady::Request>();
      request->request_id = fields[1];
      const auto response = call<ppbng_interfaces::srv::ConfirmDarkReady>(dark_client_, request);
      common_response(output, response->accepted, response->duplicate_request,
        response->session_id, response->message);
      output << "resulting_state=" << static_cast<unsigned>(response->resulting_state) << '\n';
    } else if (fields[0] == "sample") {
      if (fields.size() != 2U) throw std::runtime_error("sample requires request id");
      auto request = std::make_shared<ppbng_interfaces::srv::ConfirmSampleReady::Request>();
      request->request_id = fields[1];
      const auto response = call<ppbng_interfaces::srv::ConfirmSampleReady>(sample_client_, request);
      common_response(output, response->accepted, response->duplicate_request,
        response->session_id, response->message);
      output << "resulting_state=" << static_cast<unsigned>(response->resulting_state) << '\n';
    } else if (fields[0] == "stop") {
      if (fields.size() != 3U) throw std::runtime_error("stop requires request id and reason");
      auto request = std::make_shared<ppbng_interfaces::srv::StopAcquisition::Request>();
      request->request_id = fields[1];
      request->reason = fields[2];
      const auto response = call<ppbng_interfaces::srv::StopAcquisition>(stop_client_, request);
      common_response(output, response->accepted, response->duplicate_request,
        response->session_id, response->message);
    } else {
      throw std::runtime_error("unknown command");
    }
    return output.str();
  }

  void serve_client(const SOCKET client)
  {
    std::string input;
    char buffer[512];
    while (input.size() < 4096U && input.find('\n') == std::string::npos) {
      const auto received = recv(client, buffer, sizeof(buffer), 0);
      if (received <= 0) return;
      input.append(buffer, static_cast<std::size_t>(received));
    }
    const auto newline = input.find('\n');
    if (newline != std::string::npos) input.resize(newline);
    if (!input.empty() && input.back() == '\r') input.pop_back();
    try {
      send_all(client, execute(split_tabs(input)) + "END\n");
    } catch (const std::exception & error) {
      send_all(client, "error=" + clean(error.what()) + "\nEND\n");
    }
  }

  void serve()
  {
    WSADATA data{};
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) return;
    const auto listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) {WSACleanup(); return;}
    listen_socket_ = listener;
    BOOL exclusive = TRUE;
    setsockopt(listener, SOL_SOCKET, SO_EXCLUSIVEADDRUSE,
      reinterpret_cast<const char *>(&exclusive), sizeof(exclusive));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port_);
    if (bind(listener, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) ==
      SOCKET_ERROR || listen(listener, 4) == SOCKET_ERROR)
    {
      RCLCPP_ERROR(get_logger(), "failed to bind production control bridge on 127.0.0.1:%u",
        static_cast<unsigned>(port_));
      closesocket(listener);
      listen_socket_ = INVALID_SOCKET;
      WSACleanup();
      return;
    }
    RCLCPP_INFO(get_logger(), "production control bridge ready on 127.0.0.1:%u",
      static_cast<unsigned>(port_));
    while (!stopping_) {
      const auto client = accept(listener, nullptr, nullptr);
      if (client == INVALID_SOCKET) break;
      serve_client(client);
      closesocket(client);
    }
    const auto current = listen_socket_.exchange(INVALID_SOCKET);
    if (current != INVALID_SOCKET) closesocket(current);
    WSACleanup();
  }

  std::uint16_t port_{};
  std::atomic<bool> stopping_{false};
  std::atomic<SOCKET> listen_socket_{INVALID_SOCKET};
  std::thread server_thread_;
  rclcpp::Client<ppbng_interfaces::srv::StartAcquisition>::SharedPtr start_client_;
  rclcpp::Client<ppbng_interfaces::srv::ConfirmDarkReady>::SharedPtr dark_client_;
  rclcpp::Client<ppbng_interfaces::srv::ConfirmSampleReady>::SharedPtr sample_client_;
  rclcpp::Client<ppbng_interfaces::srv::StopAcquisition>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ProductionControlBridge>());
  } catch (const std::exception & error) {
    std::cerr << "production_control_bridge: " << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
