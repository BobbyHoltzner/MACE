#ifndef DATA_VEHICLE_GLOBAL_H
#define DATA_VEHICLE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_VEHICLE_LIBRARY)
#    define DATA_VEHICLESHARED_EXPORT  __declspec(dllexport)
#  else
#    define DATA_VEHICLESHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define DATA_VEHICLESHARED_EXPORT
#endif

#endif // MODULE_VEHICLE_MAVLINK_GLOBAL_H
