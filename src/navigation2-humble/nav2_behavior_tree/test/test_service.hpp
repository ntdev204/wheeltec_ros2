













#ifndef TEST_SERVICE_HPP_
#define TEST_SERVICE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

template<class ServiceT>
class TestService : public rclcpp::Node
{
public:
  explicit TestService(
    std::string service_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_service", options)
  {
    using namespace std::placeholders;

    server_ = create_service<ServiceT>(
      service_name,
      std::bind(&TestService::handle_service, this, _1, _2, _3));
  }

  std::shared_ptr<typename ServiceT::Request> getCurrentRequest() const
  {
    return current_request_;
  }

protected:
  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<typename ServiceT::Request> request,
    const std::shared_ptr<typename ServiceT::Response> response)
  {
    (void)request_header;
    (void)response;
    current_request_ = request;
  }

private:
  typename rclcpp::Service<ServiceT>::SharedPtr server_;
  std::shared_ptr<typename ServiceT::Request> current_request_;
};

#endif
