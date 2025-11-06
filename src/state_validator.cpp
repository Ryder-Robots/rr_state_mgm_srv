#include "rr_state_mgm_srv/state_validator.hpp"

using namespace rr_state_validator;

bool RrStateValidator::validate_uuid(unique_identifier_msgs::msg::UUID& uuid, const bool required)
{
  for (auto byte : uuid.uuid)
  {
    if (byte != 0)
    {
      return true;  // At least one byte is not zero, UUID is defined
    }
  }
  if (required)
  {
    throw RrInvalidException("invalid UUID");
  }
  return false;
}