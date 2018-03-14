#ifndef MODULE_GENERIC_MAVLINK_GLOBAL_H
#define MODULE_GENERIC_MAVLINK_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(MODULE_GENERIC_MAVLINK_LIBRARY)
#  define MODULE_GENERIC_MAVLINKSHARED_EXPORT Q_DECL_EXPORT
#else
#  define MODULE_GENERIC_MAVLINKSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // MODULE_GENERIC_MAVLINK_GLOBAL_H
