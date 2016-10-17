#ifndef RADIOINFO_H
#define RADIOINFO_H

#include <stdint.h>

class RadioInfo
{
public:
    RadioInfo();
private:
    uint8_t mRSSI; /**< Local signal strength. */
    uint8_t mRemRSSI; /**< Remote signal strength. */
    uint8_t mTXBuf; /**< Remaining free buffer space in percent. */
    uint8_t mNoise; /**< Background noise level. */
    uint8_t mRemNoise; /**< Remote background noise level. */
    uint16_t mRXErrors; /**< Receive errors. */
    uint16_t mFixedPackets; /**< Count of error corrected packets. */
};

#endif // RADIOINFO_H
