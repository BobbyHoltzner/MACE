#include "data_generic_item_text.h"

namespace DataGenericItem {

DataGenericItem_Text::DataGenericItem_Text() :
    severity(Data::StatusSeverityType::STATUS_INFO), dataString("")
{

}

DataGenericItem_Text::DataGenericItem_Text(const DataGenericItem_Text &copyObj)
{
    this->severity = copyObj.getSeverity();
    this->dataString = copyObj.getText();
}


mace_statustext_t DataGenericItem_Text::getMACECommsObject() const
{
    mace_statustext_t rtnObj;

    strcpy(rtnObj.text,this->getText().c_str());
    rtnObj.severity = (uint8_t)this->severity;

    return rtnObj;
}

} //end of namespace DataGenericItem
