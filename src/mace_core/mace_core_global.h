#ifndef MACE_CORE_GLOBAL_H
#define MACE_CORE_GLOBAL_H

#if defined(MACE_CORE_LIBRARY)
#  define MACE_CORESHARED_EXPORT
#else
#  define MACE_CORESHARED_EXPORT
#endif


#ifdef __GNUC__
#define DEPRECATED __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

#define UNUSED(x) (void)(x)

#include <cstddef>

struct EnumClassHash
{
    template <typename ET>
    std::size_t operator()(ET t) const
    {
        return static_cast<std::size_t>(t);
    }
};

#endif // MACE_CORE_GLOBAL_H
