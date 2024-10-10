// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_localization:srv/SetUTMZone.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__TRAITS_HPP_
#define ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_localization/srv/detail/set_utm_zone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_localization
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUTMZone_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: utm_zone
  {
    out << "utm_zone: ";
    rosidl_generator_traits::value_to_yaml(msg.utm_zone, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUTMZone_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: utm_zone
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "utm_zone: ";
    rosidl_generator_traits::value_to_yaml(msg.utm_zone, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUTMZone_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_localization

namespace rosidl_generator_traits
{

[[deprecated("use robot_localization::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_localization::srv::SetUTMZone_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_localization::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_localization::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_localization::srv::SetUTMZone_Request & msg)
{
  return robot_localization::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_localization::srv::SetUTMZone_Request>()
{
  return "robot_localization::srv::SetUTMZone_Request";
}

template<>
inline const char * name<robot_localization::srv::SetUTMZone_Request>()
{
  return "robot_localization/srv/SetUTMZone_Request";
}

template<>
struct has_fixed_size<robot_localization::srv::SetUTMZone_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_localization::srv::SetUTMZone_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_localization::srv::SetUTMZone_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace robot_localization
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUTMZone_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUTMZone_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUTMZone_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_localization

namespace rosidl_generator_traits
{

[[deprecated("use robot_localization::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_localization::srv::SetUTMZone_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_localization::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_localization::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_localization::srv::SetUTMZone_Response & msg)
{
  return robot_localization::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_localization::srv::SetUTMZone_Response>()
{
  return "robot_localization::srv::SetUTMZone_Response";
}

template<>
inline const char * name<robot_localization::srv::SetUTMZone_Response>()
{
  return "robot_localization/srv/SetUTMZone_Response";
}

template<>
struct has_fixed_size<robot_localization::srv::SetUTMZone_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_localization::srv::SetUTMZone_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_localization::srv::SetUTMZone_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_localization::srv::SetUTMZone>()
{
  return "robot_localization::srv::SetUTMZone";
}

template<>
inline const char * name<robot_localization::srv::SetUTMZone>()
{
  return "robot_localization/srv/SetUTMZone";
}

template<>
struct has_fixed_size<robot_localization::srv::SetUTMZone>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_localization::srv::SetUTMZone_Request>::value &&
    has_fixed_size<robot_localization::srv::SetUTMZone_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_localization::srv::SetUTMZone>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_localization::srv::SetUTMZone_Request>::value &&
    has_bounded_size<robot_localization::srv::SetUTMZone_Response>::value
  >
{
};

template<>
struct is_service<robot_localization::srv::SetUTMZone>
  : std::true_type
{
};

template<>
struct is_service_request<robot_localization::srv::SetUTMZone_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_localization::srv::SetUTMZone_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_LOCALIZATION__SRV__DETAIL__SET_UTM_ZONE__TRAITS_HPP_
