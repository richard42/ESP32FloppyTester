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
 , m_uiSectorDataLength(0)
 , m_usCurrentCRC(0xffff)
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
    // iterate over all delta samples, looking for markers
    const uint8_t pucSpecialC2[] = { 3,2,3,4,3,  4,2,3,4,3,  4,2,3,4,3, 4,2,2,2,2,2,3,2 };
    const uint32_t uiSpecialC2Len = sizeof(pucSpecialC2);
    const uint8_t pucSpecialA1[] = { 3,4,3,4,3,  2,4,3,4,3,  2,4,3,4,3 };
    const uint32_t uiSpecialA1Len = sizeof(pucSpecialA1);
    const uint8_t *pucSpecMatch = NULL;
    uint32_t uiNumSpecMatch = 0;
    uint32_t uiMaxSpecMatch = 0;
    uint32_t uiFoundZeros = 0;

    uint32_t uiFoundFive = 0;
    for (uint32_t ui = 0; ui < m_uiDeltaMax; ui++)
    {
        // get binned time delta value and reset state if long delay is found
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            uiFoundZeros = 0;
            uiNumSpecMatch = 0;
            ui++;
            continue;
        }
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen / 2.0f + 0.5f);
        // handle state
        if (uiNumSpecMatch == 0)
        {
            if (uiFoundZeros < 80)
            {
                if (uiWavelen == 2)
                    uiFoundZeros++;
                else
                    uiFoundZeros = 0;
                continue;
            }
            if (uiWavelen == 2)
                uiFoundZeros++;
            else if (uiWavelen == 3)
                uiNumSpecMatch = 1;
            else
                uiFoundZeros = 0;
            continue;
        }
        if (uiNumSpecMatch == 1)
        {
            if (uiWavelen == 2)
            {
                pucSpecMatch = pucSpecialC2;
                uiNumSpecMatch = 2;
                uiMaxSpecMatch = uiSpecialC2Len;
            }
            else if (uiWavelen == 4)
            {
                pucSpecMatch = pucSpecialA1;
                uiNumSpecMatch = 2;
                uiMaxSpecMatch = uiSpecialA1Len;
            }
            else
            {
                uiFoundZeros = 0;
                uiNumSpecMatch = 0;
            }
            continue;
        }
        if (uiWavelen != pucSpecMatch[uiNumSpecMatch])
        {
            uiFoundZeros = 0;
            uiNumSpecMatch = 0;
            pucSpecMatch = NULL;
            continue;
        }
        uiNumSpecMatch++;
        if (uiNumSpecMatch == uiMaxSpecMatch)
        {
            // found a marker
            if (pucSpecMatch == pucSpecialC2)
            {
                Serial.printf("Found Index Access Marker (track start).\r\n");
                uiFoundZeros = 0;
                uiNumSpecMatch = 0;
                pucSpecMatch = NULL;
                continue;
            }
            else // (pucSpecMatch == pucSpecialA1)
            {
                ui = ReadSectorBytes(ui + 1) - 1;
                continue;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// private methods

uint32_t DecoderMFM::ReadSectorBytes(uint32_t uiStartIdx)
{
    uint8_t ucBytes[2048];
    uint32_t uiBytesRead = 0;
    uint32_t uiExpectedBytes = 0;
    
    uint32_t uiLastBit = 1;
    uint32_t uiPriorBits = 0;
    uint32_t uiNumPriorBits = 0;
    for (uint32_t ui = uiStartIdx; ui < m_uiDeltaMax; ui++)
    {
        // get binned time delta value and bail out if long delay is found
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            // error - long delay
            Serial.printf("[Sector read error: long delay]\r\n");
            return ui + 2;
        }
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
            else
            {
                Serial.printf("[Sector read error: missing clock pulse]\r\n");
                return ui + 1;
            }
        }
        // handle byte output
        if (uiNumPriorBits < 8)
            continue;
        ucBytes[uiBytesRead] = (uiPriorBits >> (uiNumPriorBits - 8)) & 0xff;
        uiBytesRead++;
        uiPriorBits &= ((1 << (uiNumPriorBits - 8)) - 1);
        uiNumPriorBits -= 8;
        // handle block type
        if (uiBytesRead == 1)
        {
            if (ucBytes[0] == 0xfe)
            {
                // Sector ID record
                uiExpectedBytes = 7;
            }
            else if (ucBytes[0] == 0xfb)
            {
                // Sector data
                if (m_uiSectorDataLength == 0)
                {
                    Serial.printf("[Sector read error: no ID record, unknown sector length]\r\n");
                    return ui + 1;
                }
                uiExpectedBytes = m_uiSectorDataLength + 3;
            }
            else
            {
                Serial.printf("[Sector read error: invalid address mark 0x%02x]\r\n", ucBytes[0]);
                return ui + 1;
            }
            // initialize CRC calculation
            uint8_t ucFirstBytes[3] = { 0xa1, 0xa1, 0xa1 };
            m_usCurrentCRC = 0xffff;
            advance_crc16(ucFirstBytes, 3);
        }
        // handle block completion
        if (uiExpectedBytes > 0 && uiBytesRead == uiExpectedBytes)
        {
            // calculate CRC
            advance_crc16(ucBytes, uiBytesRead);
            if (ucBytes[0] == 0xfe)
            {
                // Sector ID record
                Serial.printf("Sector ID: Cylinder %i, Side %i, Sector %i, CRC=%04x (%s)\r\n", ucBytes[1], ucBytes[2], ucBytes[3],
                              (ucBytes[5] << 8) + ucBytes[6], (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 1 << (7 + ucBytes[4]);
                return ui + 1;
            }
            else if (ucBytes[0] == 0xfb)
            {
                // Sector data
                Serial.printf("Sector data: %i bytes with CRC=%04x (%s)\r\n", m_uiSectorDataLength,
                              (ucBytes[m_uiSectorDataLength+1] << 8) + ucBytes[m_uiSectorDataLength+2], (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 0;
                return ui + 1;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// private helper methods

void DecoderMFM::advance_crc16(const uint8_t* data_p, uint32_t length)
{
    unsigned char x;

    while (length--)
    {
        x = m_usCurrentCRC >> 8 ^ *data_p++;
        x ^= x>>4;
        m_usCurrentCRC = (m_usCurrentCRC << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
}
