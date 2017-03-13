#ifndef DATA_COMMS_GLOBAL_H
#define DATA_COMMS_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_COMMS_LIBRARY)
#    define DATA_COMMSSHARED_EXPORT __declspec(dllexport)
#  else
#    define DATA_COMMSSHARED_EXPORT __declspec(dllexport)
#  endif
#else
#  define DATA_GENERIC_ITEMSHARED_EXPORT
#endif

#endif // DATA_COMMS_GLOBAL_H
