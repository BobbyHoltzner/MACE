#ifndef GENERIC_MAP_ITERATOR_H
#define GENERIC_MAP_ITERATOR_H

#include "common/class_forward.h"
#include "maps/base_grid_map.h"

namespace mace{
namespace maps{

MACE_CLASS_FORWARD(GenericMapIterator);

class GenericMapIterator
{
public:
    GenericMapIterator(const BaseGridMap *map);

    GenericMapIterator(const GenericMapIterator &copy);

public:
    GenericMapIterator begin() const;

    GenericMapIterator end() const;

    bool isPastEnd() const;

public:
    virtual GenericMapIterator& operator ++();
    virtual GenericMapIterator operator ++(int);

    GenericMapIterator& operator =(const GenericMapIterator &rhs);

    bool operator == (const GenericMapIterator &rhs) const;

    bool operator !=(const GenericMapIterator &rhs) const;

    const int operator *() const;

public:
    const BaseGridMap *parentMap;

    size_t currentIndex;
    size_t startIndex;
    size_t endIndex;

    bool pastEnd;
};

} //end of namepsace mace
} //end of namespace maps

#endif // GENERIC_MAP_ITERATOR_H
