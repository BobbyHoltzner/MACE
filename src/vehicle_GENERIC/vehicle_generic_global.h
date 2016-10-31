#ifndef VEHICLE_GENERIC_GLOBAL_H
#define VEHICLE_GENERIC_GLOBAL_H

#ifdef _MSC_VER
#  if defined(VEHICLE_GENERIC_LIBRARY)
#    define VEHICLE_GENERICSHARED_EXPORT  __declspec(dllexport)
#  else
#    define VEHICLE_GENERICSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define VEHICLE_GENERICSHARED_EXPORT
#endif

#endif // MODULE_VEHICLE_MAVLINK_GLOBAL_H
