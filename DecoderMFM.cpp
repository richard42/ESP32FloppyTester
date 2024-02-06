//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// DecoderMFM.cpp
// 
// Copyright (C) 2024, All Rights Reserved.
//
// Author: Richard Goedeken
// Date:   2024/01/28

#include <stdint.h>
#include <math.h>

#include <Arduino.h>

#include <map>

#include "DecoderMFM.h"

#define DELTA_ITEM(X) m_pusDeltaBuffers[(X)>>12][(X)&0x0fff]

//////////////////////////////////////////////////////////////////////////
// constructor and destructor

DecoderMFM::DecoderMFM(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax)
 : m_pusDeltaBuffers(pusDeltaBuffers)
 , m_uiDeltaMax(uiDeltaMax)
{
}

DecoderMFM::~DecoderMFM()
{
}

//////////////////////////////////////////////////////////////////////////
// accessors

//////////////////////////////////////////////////////////////////////////
// modifiers

void DecoderMFM::DecodeTrack(bool bDebugPrint)
{
    // break the track into segments of valid bits
    std::map<uint32_t, uint32_t> mapSegments;
    uint32_t uiStartIdx = 0xffffffff;
    for (uint32_t ui = 0; ui < m_uiDeltaMax; ui++)
    {
        const uint32_t uiOrigIdx = ui;
        // get binned time delta value
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            ui++;
            uiDelta = ((uiDelta & 0x7fff) << 16) + DELTA_ITEM(ui);
        }
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen / 2.0f + 0.5f);
        // handle state
        const bool bValidDelta = (uiWavelen == 2 || uiWavelen == 3 || uiWavelen == 4);
        if (bValidDelta)
        {
            if (uiStartIdx == 0xffffffff)
            {
                uiStartIdx = uiOrigIdx;
            }
            else if (uiOrigIdx + 1 == m_uiDeltaMax)
            {
                mapSegments.insert(std::make_pair(uiStartIdx, m_uiDeltaMax));
                uiStartIdx = 0xffffffff;
            }
        }
        else if (uiStartIdx != 0xffffffff)
        {
            mapSegments.insert(std::make_pair(uiStartIdx, uiOrigIdx));
            uiStartIdx = 0xffffffff;
        }
    }

    // Decode the valid-data segments
    for (std::map<uint32_t,uint32_t>::iterator it = mapSegments.begin(); it != mapSegments.end(); ++it)
    {
        this->DecodeSegment(bDebugPrint, it->first, it->second);
        ++it;
        std::map<uint32_t,uint32_t>::iterator itNext = it;
        --it;
        if (bDebugPrint && itNext != mapSegments.end())
        {
            uint32_t uiStart = it->second;
            uint32_t uiEnd = itNext->first;
            for (uint32_t ui = uiStart; ui < uiEnd; ui++)
            {
                // get binned time delta value
                uint32_t uiDelta = DELTA_ITEM(ui);
                if (uiDelta & 0x8000)
                {
                    ui++;
                    uiDelta = ((uiDelta & 0x7fff) << 16) + DELTA_ITEM(ui);
                }
                const float fWavelen = uiDelta / 80.0f;
                Serial.printf("[%.3f]", fWavelen);
            }
            Serial.write("\r\n");
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// private methods

void DecoderMFM::DecodeSegment(bool bDebugPrint, uint32_t uiStartIdx, uint32_t uiEndIdx)
{
    // figure out the starting bit polarity by counting the number of bit flips
    // before the first "101" pattern
    uint32_t uiFlipCount = 0;
    uint32_t uiFirst101 = uiStartIdx;
    uint32_t uiTotalTime = 0;
    for (; uiFirst101 < uiEndIdx; uiFirst101++)
    {
        // get binned time delta value
        uint32_t uiDelta = DELTA_ITEM(uiFirst101);
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen / 2.0f + 0.5f);
        uiTotalTime += uiWavelen;
        // handle time delta
        if (uiWavelen == 3)
        {
            uiFlipCount++;
        }
        else if (uiWavelen == 4)
        {
            break;
        }
        else if (uiWavelen != 2)
        {
            Serial.write("DecoderMFM::DecodeSegment() internal error: invalid bit time delta.\r\n");
            return;
        }
    }

    // corner case: no synchronization found in data segment
    if (uiFirst101 == uiEndIdx)
    {
        Serial.printf("<No sync found in %i-bit data segment>\r\n", uiTotalTime / 2);
    }

    // now we know the starting bit polarity
    uint32_t uiLastBit = 1 ^ (uiFlipCount & 1);

    // iterate over all the time deltas again, and parse out the bits.
    // also, look for special patterns to indicate byte framing
    uint32_t uiPriorBits = 0;
    uint32_t uiNumPriorBits = 0;
    bool     bByteSync = false;
    for (uint32_t ui = uiStartIdx; ui < uiEndIdx; ui++)
    {
        // get binned time delta value
        uint32_t uiDelta = DELTA_ITEM(ui);
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen / 2.0f + 0.5f);
        // handle time delta
        if (uiWavelen == 2)
        {
            uiPriorBits = (uiPriorBits << 1) + uiLastBit;
            uiNumPriorBits++;
        }
        else if (uiWavelen == 3)
        {
            if (uiLastBit == 0)
            {
                uiLastBit = 1;
                uiPriorBits = (uiPriorBits << 1) + 1;
                uiNumPriorBits++;
            }
            else
            {
                uiLastBit = 0;
                uiPriorBits = (uiPriorBits << 2) + 0;
                uiNumPriorBits += 2;
            }
        }
        else // (uiWavelen == 4)
        {
            if (uiLastBit == 1)
            {
                uiPriorBits = (uiPriorBits << 2) + 1;
                uiNumPriorBits += 2;
            }
            else if (uiNumPriorBits >= 5 && (uiPriorBits & 31) == 20 && ui + 1 < uiEndIdx && floorf(DELTA_ITEM(ui+1) / 160.0f + 0.5f) == 3)
            {
                // found A1 sync byte
                Serial.write("<A1>");
                ui++;
                bByteSync = true;
                uiLastBit = 1;
                uiPriorBits = 0;
                uiNumPriorBits = 0;
            }
            else
            {
                Serial.printf("Z(%.3f)[", fWavelen);
                for (uint32_t ui = uiNumPriorBits; ui > 0; ui--)
                    Serial.printf("%c", ((uiPriorBits >> (ui - 1)) & 1) ? '1' : '0');
                Serial.write("]\r\n");
                bByteSync = false;
                uiPriorBits = 0;
                uiNumPriorBits = 0;
            }
        }
        // print debug data
        if (bDebugPrint)
        {
            if (bByteSync)
            {
                if (uiNumPriorBits >= 8)
                {
                    Serial.printf("%02x ", (uiPriorBits >> (uiNumPriorBits - 8)) & 0xff);
                    uiPriorBits &= ((1 << (uiNumPriorBits - 8)) - 1);
                    uiNumPriorBits -= 8;
                }
            }
            else
            {
                while (uiNumPriorBits > 5)
                {
                    Serial.write(((uiPriorBits >> (uiNumPriorBits-1)) & 1) ? "1" : "0");
                    uiNumPriorBits--;
                }
                uiPriorBits &= 31;
            }
        }
    }
    Serial.write("\r\n");
}
