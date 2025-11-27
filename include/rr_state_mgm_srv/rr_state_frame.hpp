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

#ifndef RR_STATE_FRAME_HPP
#define RR_STATE_FRAME_HPP

#include <uuid/uuid.h>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include "rr_interfaces/msg/state_frame.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_state_mgm_srv/visibility_control.h"

namespace rrobot
{
    namespace state_frame
    {

        /**
         * @class RrStateFrameSingleton
         * @brief provides shared resource for consuming nodes of system state frame.
         * 
         * State Frame contains as much as possible the current state from sensors at the
         * time when a requester retrieves the state_frame.
         */
        class RrStateFrameSingleton
        {
          public:
            /**
             * @fn get_instance
             * @brief returns global instance of RrStateFrameSingleton.
             * 
             * If it has not been access before this will create the instance.
             */
            static RrStateFrameSingleton &get_instance()
            {
                static RrStateFrameSingleton instance;
                return instance;
            }

            /**
             * @fn set_request_id
             * @brief request id is provided by requested client as part of state_frame service request.
             * 
             * Represents the clients unique identifier and is kept for storage within bagging service to train
             * ML algorithms.
             */
            void set_request_id(const unique_identifier_msgs::msg::UUID &request_id);

            /**
             * @fn set_gps
             * @brief sets GPS coordinates
             * 
             * Used by GPS sensor to set GPS coordinates
             */
            void set_gps(const sensor_msgs::msg::NavSatFix &gps);

            /**
             * @fn set_joystick
             * @brief sets joystick/game controller frame
             * 
             * used by joystick sensor, to set current state of joystick.
             */
            void set_joystick(const sensor_msgs::msg::Joy &joy);

            /**
             * @fn set_batt_state
             * @brief sets current battery state
             * 
             * used by battery state sensor to send monitoring information about the battery.
             */
            void set_batt_state(const sensor_msgs::msg::BatteryState &batt_state);

            /**
             * @fn set_img
             * @brief used by camera sensors to set current image
             */
            void set_img(const sensor_msgs::msg::Image &img);

            /**
             * @fn set_imu
             * @brief sets current IMU state
             * 
             * used by IMU to set Gyro, Mangnometer, etc
             */
            void set_imu(const sensor_msgs::msg::Imu &imu);

            /**
             * @fn set_range
             * @brief sets a range sensor
             * 
             * Common interface for all range sensor settings, note that 
             * the header frame_id is crucial for setting range sensors, as it
             * will allow this object to correcly assigned them in the correct element
             * of the ranges vector when state_frame is requested.
             */
            void set_range(const sensor_msgs::msg::Range &range);

            /**
             * @fn get_state
             * @brief returns current state frame.
             * 
             * Sets GUID, header and returns the current frame.
             */
            const rr_interfaces::msg::StateFrame get_state();

          protected:
            // Private constructor to prevent external instantiation
            RrStateFrameSingleton() {}

            // Delete copy constructor and assignment operator to prevent copies
            RrStateFrameSingleton(const RrStateFrameSingleton &) = delete;
            RrStateFrameSingleton &operator=(const RrStateFrameSingleton &) = delete;

          private:
            std::mutex mtx_;
            rr_interfaces::msg::StateFrame state_frame_;

            // mappings for range sensors, note that only the following range sensors are supported
            const std::unordered_map<std::string, uint> range_sensor_map_{
                {rr_constants::LINK_ULTRA_SONIC_LEFT, 0},
                {rr_constants::LINK_ULTRA_SONIC_CENTER, 1},
                {rr_constants::LINK_ULTRA_SONIC_RIGHT, 2}
            };
        };
    } // namespace state_frame
} // namespace rrobot

#endif