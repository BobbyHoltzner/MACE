#ifndef BASE_DATA_ITEM_H
#define BASE_DATA_ITEM_H

#include <vector>
#include <functional>
#include <memory>

#include <unordered_map>

#include "common/optional_parameter.h"
#include "mace_core/module_characteristics.h"

namespace Controllers {

//!
//! \brief Creates an object that can fetch and notify upon reception of data elements
//!
//! Data elements are defined as a Key/Type pair.
//! The Key is a type that identifies the data resource on the entity.
//! The Type is the data element itself
//!
//! For example the Key may be the ModuleCharacteristic itself in data elements such as home position, where there is only one per vehicles.
//! However is Missions the Key may be MissionKey object since a single vehicle may have multiple missions on it.
//!
//! This object creates various set/fetch lambda methods to perform three tasks:
//! - DataReceived : Notifies a resourse that data has been received.
//! - FetchDataFromKey : Given a Key, fetch all data elements that match that key on a given resource.
//! - FetchAll : Fetch all key/values pairs on a given resource.
//!
//! \template Key Key of data elemement that uniquily identifies itself on the module of origin
//! \template Type Type of data object
//!
template< typename Key, typename Type>
class DataItem
{
public:

    typedef std::vector<std::tuple<Key, Type>> FetchKeyReturn;
    typedef std::vector<std::tuple<MaceCore::ModuleCharacteristic, std::vector<std::tuple<Key, Type>>>> FetchModuleReturn;

private:

    std::unordered_map<void*, std::function<void(const Key &, const std::shared_ptr<Type> &)>> m_lambda_DataRecieved;
    OptionalParameter<std::function<FetchKeyReturn(const OptionalParameter<Key> &)>> m_lambda_FetchDataFromKey;
    OptionalParameter<std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)>> m_lambda_FetchAll;
public:


    void setLambda_DataReceived(const std::function<void(const Key &, const std::shared_ptr<Type> &)> &lambda){
        m_lambda_DataRecieved.insert({0, lambda});
    }

    void AddLambda_DataReceived(void* sender, const std::function<void(const Key &, const std::shared_ptr<Type> &)> &lambda){
        m_lambda_DataRecieved.insert({sender, lambda});
    }

    void onDataReceived(const Key &key, const std::shared_ptr<Type> &data){

        for(auto it = m_lambda_DataRecieved.cbegin() ; it != m_lambda_DataRecieved.cend() ; ++it)
        {
            it->second(key, data);
        }
    }




    void setLambda_FetchDataFromKey(const std::function<FetchKeyReturn(const OptionalParameter<Key> &)> &lambda){
        m_lambda_FetchDataFromKey = lambda;
    }

    void FetchDataFromKey(const OptionalParameter<Key> &key, FetchKeyReturn &data) const
    {
        if(m_lambda_FetchDataFromKey.IsSet() == false) {
            throw std::runtime_error("FetchKey Lambda not set!");
        }

        data = m_lambda_FetchDataFromKey()(key);
    }




    void setLambda_FetchAll(const std::function<FetchModuleReturn(const OptionalParameter<MaceCore::ModuleCharacteristic> &)> &lambda)
    {
        m_lambda_FetchAll = lambda;
    }

    void FetchFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &target, FetchModuleReturn &data) const
    {
        if(m_lambda_FetchAll.IsSet() == false) {
            throw std::runtime_error("FetchFromModule Lambda not set!");
        }

        data = m_lambda_FetchAll()(target);
        return;
    }
};

}

#endif // BASE_DATA_ITEM_H
