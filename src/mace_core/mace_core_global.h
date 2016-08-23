#ifndef MACE_CORE_GLOBAL_H
#define MACE_CORE_GLOBAL_H

#if defined(MACE_CORE_LIBRARY)
#  define MACE_CORESHARED_EXPORT
#else
#  define MACE_CORESHARED_EXPORT
#endif

#define UNUSED(x) (void)(x)

#endif // MACE_CORE_GLOBAL_H
