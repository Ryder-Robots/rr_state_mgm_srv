#ifndef RR_INVALID_ATTIBUTE_EXCEPTION
#define RR_INVALID_ATTIBUTE_EXCEPTION

#include <stdexcept>

namespace rr_state_validator
{
class RrInvalidException : public std::runtime_error
{
 public:
  explicit RrInvalidException(const std::string& message) : std::runtime_error(message) {}
};
}  // namespace rr_state_validator

#endif