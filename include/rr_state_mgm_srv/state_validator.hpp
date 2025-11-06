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
 * 
 * This is for internal validation mainly,  validation should be done at the driver level. All input
 * from external sources, such GPS etc, should be appropriatly validatoed but it is better to it at the
 * node level, because it allows for the code be more tightly coupled.
 */
class RrStateValidator
{
 public:

 /**
  * @fn validate_uuid
  * @brief check that input is a valid UUID
  * @param uuid UUID being validated
  * @param required, if set then routine throws exception instead of returning false.
  */
  bool validate_uuid(unique_identifier_msgs::msg::UUID& uuid, const bool required);
};
}  // namespace rr_state_validator

#endif