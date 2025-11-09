#ifndef RR_NAVIGATION_SERVICE_HPP
#define RR_NAVIGATION_SERVICE_HPP

#include "rr_state_mgm_srv/rr_service_base.hpp"
#include "rr_interfaces/srv/navigation.hpp"

namespace rr_state_manager
{
    class RrNavigationService : public RrStateServiceBase
    {
        void set_nav(const std::shared_ptr<rr_interfaces::srv::Navigation::Request> request,
                    std::shared_ptr<rr_interfaces::srv::Navigation::Response> response);
    };
}

#endif // RR_NAVIGATION_SERVICE_HPP