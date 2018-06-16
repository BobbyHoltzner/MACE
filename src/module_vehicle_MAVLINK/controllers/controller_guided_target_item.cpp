#include "controller_guided_target_item.h"

namespace MAVLINKVehicleControllers {

    template <>
    void ControllerGuidedTargetItem<TargetControllerStruct>::FillTargetItem(const TargetControllerStruct &targetStruct, mavlink_set_position_target_local_ned_t &mavlinkItem)
    {
        TargetItem::DynamicTargetList::DynamicTarget targetItem = targetStruct.target;

        mavlinkItem.x = targetItem.position.getXPosition();
        mavlinkItem.y = targetItem.position.getYPosition();
        mavlinkItem.z = targetItem.position.getZPosition();
        mavlinkItem.vx = targetItem.velocity.getXVelocity();
        mavlinkItem.vy = targetItem.velocity.getYVelocity();
        mavlinkItem.vz = targetItem.velocity.getZVelocity();
        mavlinkItem.yaw = targetItem.yaw;
        mavlinkItem.yaw_rate = targetItem.yawRate;
        //now we need to update the bitmask to the appropriate values
        //If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration.
        /*
         * Mapping: bit 1: x, bit 2: y, bit 3: z
         * bit 4: vx, bit 5: vy, bit 6: vz,
         * bit 7: ax, bit 8: ay, bit 9: az,
         * bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
         */

        uint16_t bitArray = 65535;
        uint16_t mask = 1<<0; // set the mask for a x position
        if(targetItem.position.hasXBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<0);
        mask = 1<<1;
        if(targetItem.position.hasYBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<1);
        mask = 1<<2;
        if(targetItem.position.hasZBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<2);

        mask = 1<<3;
        if(targetItem.velocity.hasXVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<3);
        mask = 1<<4;
        if(targetItem.velocity.hasYVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<4);
        mask = 1<<5;
        if(targetItem.velocity.hasZVelocityBeenSet())
            bitArray = (bitArray & (~mask)) | ((int)0<<5);

        mask = 1<<10;
        if(std::abs(targetItem.yaw) > 0.001) //KEN FIX THIS
            bitArray = (bitArray & (~mask)) | ((int)0<<10);
        mask = 1<<11;
        if(std::abs(targetItem.yawRate) > 0.001) //KEN FIX THIS
            bitArray = (bitArray & (~mask)) | ((int)0<<11);

        mavlinkItem.type_mask = bitArray;
    }

}// end of namespace MAVLINKVehicleControllers
