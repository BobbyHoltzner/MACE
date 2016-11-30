#ifndef DATA_ARDUPILOT_GLOBAL_H
#define DATA_ARDUPILOT_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_ARDUPILOT_LIBRARY)
#    define DATA_ARDUPILOTSHARED_EXPORT  __declspec(dllexport)
#  else
#    define DATA_ARDUPILOTSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define DATA_ARDUPILOTSHARED_EXPORT
#endif

#endif // DATA_ARDUPILOT_GLOBAL_H


