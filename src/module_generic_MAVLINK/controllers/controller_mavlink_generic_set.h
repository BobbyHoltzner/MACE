#ifndef CONTROLLER_MAVLINK_GENERIC_SET_H
#define CONTROLLER_MAVLINK_GENERIC_SET_H

#include "controllers/generic_controller.h"
#include "controllers/generic_controller_queue_data_with_module.h"

#include "data_generic_command_item/do_items/action_change_mode.h"

#include "controllers/actions/action_send.h"
#include "controllers/actions/action_final_receive_respond.h"
#include "controllers/actions/action_finish.h"

namespace ModuleGenericMavlink {


namespace MAVLINKGenericControllers {

/*
template <typename MESSAGETYPE>
using SystemModeSend = ActionSend<
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;

template <typename MESSAGETYPE>
using SystemMode_FinalReceiveRespond = ActionFinalReceiveRespond<
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    CommandItem::ActionChangeMode,
    mace_command_system_mode_t,
    mace_system_mode_ack_t,
    MACE_MSG_ID_COMMAND_SYSTEM_MODE
>;

template <typename MESSAGETYPE>
using SystemModeFinish = ActionFinish<
    GenericControllerQueueDataWithModule<MESSAGETYPE, CommandItem::ActionChangeMode>,
    MaceCore::ModuleCharacteristic,
    mace_system_mode_ack_t,
    MACE_MSG_ID_SYSTEM_MODE_ACK
>;
*/


template<typename MESSAGETYPE, typename DATA, typename SET_TYPE, typename ACK_TYPE, const int SET_ID, const int ACK_ID>
class GenericControllerSetRequest : public Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,

        public Controllers::ActionSend<
            MESSAGETYPE,
            Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
            MaceCore::ModuleCharacteristic,
            DATA,
            SET_TYPE,
            ACK_ID
        >,
        public Controllers::ActionFinish<
            MESSAGETYPE,
            Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
            MaceCore::ModuleCharacteristic,
            uint8_t,
            ACK_TYPE,
            ACK_ID
        >
{

private:

    std::function<void(const DATA &commandItem, const MaceCore::ModuleCharacteristic &target, SET_TYPE &cmd)> m_FillSetObject;

public:

    void FillSetObject(const std::function<void(const DATA &commandItem, const MaceCore::ModuleCharacteristic &target, SET_TYPE &cmd)> &lambda)
    {
        m_FillSetObject = lambda;
    }

    //!
    //! \brief Query to be given to determin if controller has the given action
    //! \param action Action to ask if controller contains
    //! \return True if contains
    //!
    bool ContainsAction(const Controllers::Actions action)
    {
        if(action == Controllers::Actions::SEND)
        {
            return true;
        }

        return false;
    }


protected:

    virtual void FillObject(const DATA &commandItem, const MaceCore::ModuleCharacteristic &target, SET_TYPE &cmd)
    {
        m_FillSetObject(commandItem, target, cmd);
    }


protected:


    virtual void Construct_Send(const DATA &commandItem, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, SET_TYPE &cmd, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        queueObj = target;

        FillObject(commandItem, target, cmd);
    }

    virtual bool Finish_Receive(const ACK_TYPE &msg, const MaceCore::ModuleCharacteristic &sender, uint8_t& ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);
        queueObj = sender;
        ack = msg.result;
        return true;
    }



public:

    GenericControllerSetRequest(const Controllers::IMessageNotifier<MESSAGETYPE>* cb,
                         Controllers::MessageModuleTransmissionQueue<MESSAGETYPE> * queue,
                         int linkChan,
                         const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGETYPE*, const SET_TYPE*)> &encode_chan,
                         const std::function<void(const MESSAGETYPE*, ACK_TYPE*)> &decode_ack) :
        Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>(cb, queue, linkChan),
        Controllers::ActionSend<
                    MESSAGETYPE,
                    Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
                    MaceCore::ModuleCharacteristic,
                    DATA,
                    SET_TYPE,
                    ACK_ID
                >(this, encode_chan),
        Controllers::ActionFinish<
                    MESSAGETYPE,
                    Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
                    MaceCore::ModuleCharacteristic,
                    uint8_t,
                    ACK_TYPE,
                    ACK_ID
                >(this, decode_ack)
    {

    }

};



template<typename MESSAGETYPE, typename DATA, typename SET_TYPE, typename ACK_TYPE, const int SET_ID, const int ACK_ID>
class GenericControllerSetRequestRespond :
        public GenericControllerSetRequest<MESSAGETYPE, DATA, SET_TYPE, ACK_TYPE, SET_ID, ACK_ID>,

        public Controllers::ActionFinalReceiveRespond<
            MESSAGETYPE,
            Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
            MaceCore::ModuleCharacteristic,
            DATA,
            SET_TYPE,
            ACK_TYPE,
            SET_ID
        >
{

private:

    std::function<void(const SET_TYPE &commandItem, std::shared_ptr<DATA> &data, ACK_TYPE &ack)> m_FillDataAndAck;

public:


    void FillDataAndAck(const std::function<void(const SET_TYPE &commandItem, std::shared_ptr<DATA> &data, ACK_TYPE &ack)> &lambda)
    {
        m_FillDataAndAck = lambda;
    }

    //!
    //! \brief Query to be given to determin if controller has the given action
    //! \param action Action to ask if controller contains
    //! \return True if contains
    //!
    bool ContainsAction(const Controllers::Actions action)
    {
        return GenericControllerSetRequest<MESSAGETYPE, DATA, SET_TYPE, ACK_TYPE, SET_ID, ACK_ID>::ContainsAction(action);
    }


protected:

    virtual void FillObject(const SET_TYPE &commandItem, std::shared_ptr<DATA> &data, ACK_TYPE &ack)
    {
        m_FillDataAndAck(commandItem, data, ack);
    }


protected:



    virtual bool Construct_FinalObjectAndResponse(const SET_TYPE &msg, const MaceCore::ModuleCharacteristic &sender, ACK_TYPE &ack, std::shared_ptr<DATA> &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        vehicleObj.ID = msg.target_system;
        vehicleObj.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

        queueObj = vehicleObj;

        FillObject(msg, data, ack);
        return true;
    }


public:

    GenericControllerSetRequestRespond(const Controllers::IMessageNotifier<MESSAGETYPE>* cb,
                         Controllers::MessageModuleTransmissionQueue<MESSAGETYPE> * queue,
                         int linkChan,
                         const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGETYPE*, const SET_TYPE*)> &encode_chan,
                         const std::function<void(const MESSAGETYPE*, SET_TYPE*)> &decode,
                         const std::function<void(uint8_t system_id, uint8_t, uint8_t, MESSAGETYPE*, const ACK_TYPE*)> &encode_ack_chan,
                         const std::function<void(const MESSAGETYPE*, ACK_TYPE*)> &decode_ack) :
        GenericControllerSetRequest<MESSAGETYPE, DATA, SET_TYPE, ACK_TYPE, SET_ID, ACK_ID>(cb, queue, linkChan, encode_chan, decode_ack),
        Controllers::ActionFinalReceiveRespond<
                    MESSAGETYPE,
                    Controllers::GenericControllerQueueDataWithModule<MESSAGETYPE, DATA>,
                    MaceCore::ModuleCharacteristic,
                    DATA,
                    SET_TYPE,
                    ACK_TYPE,
                    SET_ID
                >(this, decode, encode_ack_chan)
    {

    }

};

}

}

#endif // CONTROLLER_MAVLINK_GENERIC_SET_H
