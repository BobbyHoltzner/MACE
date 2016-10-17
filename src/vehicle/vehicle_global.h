#ifndef VEHICLE_GLOBAL_H
#define VEHICLE_GLOBAL_H

#ifdef _MSC_VER
#  if defined(VEHICLE_LIBRARY)
#    define VEHICLESHARED_EXPORT  __declspec(dllexport)
#  else
#    define VEHICLESHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define VEHICLESHARED_EXPORT
#endif

#endif // VEHICLE_GLOBAL_H

