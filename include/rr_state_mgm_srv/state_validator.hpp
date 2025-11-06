#ifndef STATE_VALIDATOR_HPP
#define STATE_VALIDATOR_HPP

#include "rr_invalid_attribute_exception.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace rr_state_validator
{

/**
 * @class RrStateValidator
 *
 * @brief validates input
 */
class RrStateValidator
{
 public:
  bool validate_uuid(unique_identifier_msgs::msg::UUID& uuid, const bool required);
};
}  // namespace rr_state_validator

#endif