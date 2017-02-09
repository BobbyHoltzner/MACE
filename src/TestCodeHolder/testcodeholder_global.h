#ifndef TESTCODEHOLDER_GLOBAL_H
#define TESTCODEHOLDER_GLOBAL_H

#ifdef _MSC_VER
#  if defined(TESTCODEHOLDER_LIBRARY)
#    define TESTCODEHOLDERSHARED_EXPORT  __declspec(dllexport)
#  else
#    define TESTCODEHOLDERSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define TESTCODEHOLDERSHARED_EXPORT
#endif
#endif // TESTCODEHOLDER_GLOBAL_H
