//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// DecoderMFM.h
// 
// Copyright (C) 2024, All Rights Reserved.
//
// Author: Richard Goedeken
// Date:   2024/01/28

#include <stdint.h>

#include "FloppyTester.h"

class DecoderMFM
{
public:
    DecoderMFM(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax);
    ~DecoderMFM();

    // accessors

    // modifiers
    void DecodeTrack(geo_format_t eFormat, bool bDebugPrint);

    // private methods
private:
    uint32_t ReadSectorBytesIBM(uint32_t uiStartIdx);
    uint32_t ReadSectorBytesAmiga(uint32_t uiStartIdx);
    void advance_crc16(const uint8_t* data_p, uint32_t length);

    // member variables
private:
    const uint16_t  **m_pusDeltaBuffers;
    const uint32_t    m_uiDeltaMax;
    uint32_t          m_uiSectorDataLength;
    uint16_t          m_usCurrentCRC;
};
