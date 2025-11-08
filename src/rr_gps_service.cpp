// #include "rr_state_mgm_srv/rr_gps_service.hpp"

// using namespace rr_state_manager;

// void RrrGpsService::set_gps(const std::shared_ptr<rr_interfaces::srv::Gps::Request> request,
//                             std::shared_ptr<rr_interfaces::srv::Gps::Response> response)
// {
//   auto cb1 = [this](const std::shared_ptr<rr_interfaces::srv::Gps::Request>& request)
//   {
//     buffer_response_->feature_sets.has_gps = true;
//     buffer_response_->gps                  = request->gps_tx;
//   };

//   auto cb2 = [this](std::shared_ptr<rr_interfaces::srv::Gps::Response>& response)
//   { response->gps_rx = buffer_response_->gps; };

//   set_rr_state<rr_interfaces::srv::Gps::Request, rr_interfaces::srv::Gps::Response>(
//       request, response, cb1, cb2, request->override_state);
// }