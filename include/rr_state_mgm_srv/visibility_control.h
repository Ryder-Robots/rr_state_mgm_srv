#ifndef RR_STATE_MGM_SRV__VISIBILITY_CONTROL_H_
#define RR_STATE_MGM_SRV__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RR_STATE_MGM_SRV_EXPORT __attribute__ ((dllexport))
    #define RR_STATE_MGM_SRV_IMPORT __attribute__ ((dllimport))
  #else
    #define RR_STATE_MGM_SRV_EXPORT __declspec(dllexport)
    #define RR_STATE_MGM_SRV_IMPORT __declspec(dllimport)
  #endif
  #ifdef RR_STATE_MGM_SRV_BUILDING_LIBRARY
    #define RR_STATE_MGM_SRV_PUBLIC RR_STATE_MGM_SRV_EXPORT
  #else
    #define RR_STATE_MGM_SRV_PUBLIC RR_STATE_MGM_SRV_IMPORT
  #endif
  #define RR_STATE_MGM_SRV_PUBLIC_TYPE RR_STATE_MGM_SRV_PUBLIC
  #define RR_STATE_MGM_SRV_LOCAL
#else
  #define RR_STATE_MGM_SRV_EXPORT __attribute__ ((visibility("default")))
  #define RR_STATE_MGM_SRV_IMPORT
  #if __GNUC__ >= 4
    #define RR_STATE_MGM_SRV_PUBLIC __attribute__ ((visibility("default")))
    #define RR_STATE_MGM_SRV_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RR_STATE_MGM_SRV_PUBLIC
    #define RR_STATE_MGM_SRV_LOCAL
  #endif
  #define RR_STATE_MGM_SRV_PUBLIC_TYPE
#endif

#endif  // RR_STATE_MGM_SRV__VISIBILITY_CONTROL_H_
