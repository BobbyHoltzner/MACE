#ifndef DATA_GENERIC_MISSION_ITEM_TOPIC_GLOBAL_H
#define DATA_GENERIC_MISSION_ITEM_TOPIC_GLOBAL_H

#ifdef _MSC_VER
#  if defined(DATA_GENERIC_MISSION_ITEM_TOPIC_LIBRARY)
#    define DATA_GENERIC_MISSION_ITEM_TOPICSHARED_EXPORT  __declspec(dllexport)
#  else
#    define DATA_GENERIC_MISSION_ITEM_TOPICSHARED_EXPORT  __declspec(dllimport)
#  endif
#else
#  define DATA_GENERIC_MISSION_ITEM_TOPICSHARED_EXPORT
#endif


#endif // DATA_GENERIC_MISSION_ITEM_TOPIC_GLOBAL_H
