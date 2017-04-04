#include "data_generic_item_text.h"

namespace DataGenericItem {

DataGenericItem_Text::DataGenericItem_Text() :
    severity(STATUS_INFO), dataString("")
{

}

DataGenericItem_Text::DataGenericItem_Text(const DataGenericItem_Text &copyObj)
{
    this->severity = copyObj.getSeverity();
    this->dataString = copyObj.getText();
}

} //end of namespace DataGenericItem
