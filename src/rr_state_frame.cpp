// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "rr_state_mgm_srv/rr_state_frame.hpp"

namespace rrobot::state_frame
{

    // set request id
    void RrStateFrameSingleton::set_request_id(const unique_identifier_msgs::msg::UUID &request_id)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.request_id = request_id;
    }

    // set gps
    void RrStateFrameSingleton::set_gps(const sensor_msgs::msg::NavSatFix &gps)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_gps = true;
        state_frame_.gps = gps;
    }

    // set joystick
    void RrStateFrameSingleton::set_joystick(const sensor_msgs::msg::Joy &joy)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_joy = true;
        state_frame_.joystick = joy;
    }

    // set battery state
    void RrStateFrameSingleton::set_batt_state(const sensor_msgs::msg::BatteryState &batt_state)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_batt_state = true;
        state_frame_.batt_state = batt_state;
    }

    void RrStateFrameSingleton::set_img(const sensor_msgs::msg::Image &img)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_img = true;
        state_frame_.img = img;
    }

    void RrStateFrameSingleton::set_imu(const sensor_msgs::msg::Imu &imu)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_imu = true;
        state_frame_.imu = imu;
    }
    
    /** 
     * ranges must be placed in a specific order:
     * 
     */
    void RrStateFrameSingleton::set_range(const sensor_msgs::msg::Range &range) 
    {
        std::lock_guard<std::mutex> lock(mtx_);
        state_frame_.feature_sets.has_ranges = true;

        auto it = range_sensor_map_.find(range.header.frame_id);
        if (it == range_sensor_map_.end()) {
            // unable to add sensor, this can be extended to state that it is not supported.
            return;
        }

        if (state_frame_.ranges.size() < it->second + 1) {
            // resize the map to fit.
            state_frame_.ranges.resize(it->second + 1);
        }

        state_frame_.ranges[it->second] = range;
    }

    /**
     * set GUID and sequence, before calling this request_id should be set by the 
     * calling service.
     */
    const rr_interfaces::msg::StateFrame RrStateFrameSingleton::get_state() {
        std::lock_guard<std::mutex> lock(mtx_);
        rclcpp::Clock clock(RCL_ROS_TIME);
        
        // set stamp before processing, to get better latency control
        state_frame_.header.frame_id = rr_constants::LINK_BUFF_SVR;
        state_frame_.header.stamp = clock.now();
        state_frame_.seq = state_frame_.seq + 1;

        // generate random UUID
        uuid_t uuid;
        uuid_generate_random(uuid);
        for (int i = 0; i < 16; i++) {
            state_frame_.guid.uuid[i] = uuid[i];
        }
        
        return state_frame_;
    }
} // namespace rrobot::state_frame