#include "generic_map_iterator.h"

namespace mace{
namespace maps{

GenericMapIterator::GenericMapIterator(const BaseGridMap *map)
{
    this->parentMap = map;
    this->startIndex = 0;
    this->currentIndex = 0;
    this->endIndex = map->getNodeCount() - 1;
    this->pastEnd = false;
}

GenericMapIterator::GenericMapIterator(const GenericMapIterator &copy)
{
    this->parentMap = copy.parentMap;
    this->startIndex = copy.startIndex;
    this->currentIndex = copy.currentIndex;
    this->endIndex = copy.endIndex;
    this->pastEnd = copy.pastEnd;
}

GenericMapIterator GenericMapIterator::begin() const
{
    GenericMapIterator newIT(*this);
    newIT.currentIndex = this->startIndex;
    return newIT;
}

GenericMapIterator GenericMapIterator::end() const
{
    GenericMapIterator newIT(*this);
    newIT.currentIndex = this->endIndex + 1;
    return newIT;
}

bool GenericMapIterator::isPastEnd() const
{
    return pastEnd;
}

GenericMapIterator& GenericMapIterator::operator =(const GenericMapIterator& rhs)
{
    this->parentMap = rhs.parentMap;
    this->startIndex = rhs.startIndex;
    this->currentIndex = rhs.currentIndex;
    this->endIndex = rhs.endIndex;
    this->pastEnd = rhs.pastEnd;
    return *this;
}

bool GenericMapIterator::operator == (const GenericMapIterator &rhs) const
{
    if(this->parentMap != rhs.parentMap){
        return false;
    }
    if(this->startIndex != rhs.startIndex){
        return false;
    }
    if(this->currentIndex != rhs.currentIndex){
        return false;
    }
    if(this->endIndex != rhs.endIndex){
        return false;
    }
    if(this->pastEnd != rhs.pastEnd){
        return false;
    }
    return true;
}

bool GenericMapIterator::operator != (const GenericMapIterator &rhs) const
{
    return !(*this == rhs);
}


GenericMapIterator& GenericMapIterator::operator ++()
{
    currentIndex++;
    if(currentIndex <= this->endIndex)
        pastEnd = false;
    else
        pastEnd = true;

    return *this;
}

GenericMapIterator GenericMapIterator::operator ++(int)
{
    GenericMapIterator old = *this;
    ++*this;
    return old;
}

const int GenericMapIterator::operator *() const
{
    return currentIndex;
}


} //end of namepsace mace
} //end of namespace maps


