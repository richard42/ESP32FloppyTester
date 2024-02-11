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

#include "FloppyTester.h"
#include "DecoderMFM.h"

#define DELTA_ITEM(X) m_pusDeltaBuffers[(X)>>12][(X)&0x0fff]

//////////////////////////////////////////////////////////////////////////
// constructor and destructor

DecoderMFM::DecoderMFM(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax)
 : m_pusDeltaBuffers(pusDeltaBuffers)
 , m_uiDeltaMax(uiDeltaMax)
 , m_uiSectorDataLength(0)
 , m_usCurrentCRC(0xffff)
 , m_uiSectorsRead(0)
{
}

DecoderMFM::~DecoderMFM()
{
}

//////////////////////////////////////////////////////////////////////////
// accessors

//////////////////////////////////////////////////////////////////////////
// modifiers

void DecoderMFM::DecodeTrack(geo_format_t eFormat, bool bDebugPrint)
{
    // clear sector counter
    m_uiSectorsRead = 0;
    
    // iterate over all delta samples, looking for markers
    const uint8_t pucSpecialC2[] = { 3,2,3,4,3,  4,2,3,4,3,  4,2,3,4,3, 4,2,2,2,2,2,3,2 };
    const uint32_t uiSpecialC2Len = sizeof(pucSpecialC2);
    const uint8_t pucSpecialA1[] = { 3,4,3,4,3,  2,4,3,4,3,  2,4,3,4,3 };
    const uint32_t uiSpecialA1Len = sizeof(pucSpecialA1);
    const uint8_t *pucSpecMatch = NULL;
    uint32_t uiNumSpecMatch = 0;
    uint32_t uiMaxSpecMatch = 0;
    uint32_t uiFoundZeros = 0;

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
            if (uiFoundZeros < (eFormat == FMT_AMIGA ? 12 : 80))
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
        
        // special case for Amiga
        if (eFormat == FMT_AMIGA && uiFoundZeros <= 18 && uiFoundZeros >= 12 && pucSpecMatch == pucSpecialA1 && uiNumSpecMatch == 10)
        {
            ui = ReadSectorBytesAmiga(ui + 1) - 1;
            continue;
        }

        // normal cases for IBM
        if (eFormat == FMT_IBM && uiNumSpecMatch == uiMaxSpecMatch)
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
                ui = ReadSectorBytesIBM(ui + 1) - 1;
                continue;
            }
        }
    }

    Serial.printf("%i %s sectors read.\r\n", m_uiSectorsRead, (eFormat == FMT_IBM ? "IBM" : "Amiga"));
}

//////////////////////////////////////////////////////////////////////////
// private methods

uint32_t DecoderMFM::ReadSectorBytesIBM(uint32_t uiStartIdx)
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
            m_uiSectorsRead++;
            // calculate CRC
            advance_crc16(ucBytes, uiBytesRead);
            if (ucBytes[0] == 0xfe)
            {
                // Sector ID record
                Serial.printf("IBM Sector ID: Cylinder %i, Side %i, Sector %i, CRC=%04x (%s)\r\n", ucBytes[1], ucBytes[2], ucBytes[3],
                              (ucBytes[5] << 8) + ucBytes[6], (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 1 << (7 + ucBytes[4]);
                return ui + 1;
            }
            else if (ucBytes[0] == 0xfb)
            {
                // Sector data
                Serial.printf("IBM Sector data: %i bytes with CRC=%04x (%s)\r\n", m_uiSectorDataLength,
                              (ucBytes[m_uiSectorDataLength+1] << 8) + ucBytes[m_uiSectorDataLength+2], (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 0;
                return ui + 1;
            }
        }
    }
}

uint32_t DecoderMFM::ReadSectorBytesAmiga(uint32_t uiStartIdx)
{
    uint8_t ucSwizBytes[540];
    uint8_t ucBytes[540];
    uint32_t uiBytesRead = 0;
    
    uint32_t uiLastBit = 1;
    uint32_t uiPriorBits = 0;
    uint32_t uiNumPriorBits = 0;
    for (volatile uint32_t ui = uiStartIdx; ui < m_uiDeltaMax; ui++)
    {
        if (ui >= m_uiDeltaMax)  // this is to work around some horrible compiler bug
        {
            return ui;
        }
        // get binned time delta value and bail out if long delay is found
        const uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            // error - long delay
            Serial.printf("[Sector read error: long delay] (position %i/%i)\r\n", ui, m_uiDeltaMax);
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
                Serial.printf("[Sector read error: missing clock pulse] (position %i/%i)\r\n", ui, m_uiDeltaMax);
                return ui + 1;
            }
        }
        // handle byte output
        if (uiNumPriorBits < 8)
            continue;
        ucSwizBytes[uiBytesRead] = (uiPriorBits >> (uiNumPriorBits - 8)) & 0xff;
        uiBytesRead++;
        uiPriorBits &= ((1 << (uiNumPriorBits - 8)) - 1);
        uiNumPriorBits -= 8;
        // handle track completion
        if (uiBytesRead < 512 + 28)
            continue;
        // de-multipex (un-swizzle) the bytes
        for (uint32_t ui2 = 0; ui2 < 540; ui2++)
        {
            uint32_t uiBlockStart = 0, uiPairOffset = 0;
            if (ui2 < 4)
                { uiBlockStart = 0;  uiPairOffset = 2; }
            else if (ui2 < 20)
                { uiBlockStart = 4;  uiPairOffset = 8; }
            else if (ui2 < 24)
                { uiBlockStart = 20; uiPairOffset = 2; }
            else if (ui2 < 28)
                { uiBlockStart = 24; uiPairOffset = 2; }
            else
                { uiBlockStart = 28; uiPairOffset = 256; }
            const uint32_t uiOddIdx = (ui2 - uiBlockStart) / 2 + uiBlockStart;
            const uint32_t uiEvenIdx = uiOddIdx + uiPairOffset;
            uint8_t ucOddBits = ucSwizBytes[uiOddIdx];
            uint8_t ucEvenBits = ucSwizBytes[uiEvenIdx];
            if (ui2 & 1)
            {
                ucOddBits &= 0x0f;
                ucEvenBits &= 0x0f;
            }
            else
            {
                ucOddBits >>= 4;
                ucEvenBits >>= 4;
            }
            ucBytes[ui2] = ((ucOddBits & 0x08) << 4) + ((ucEvenBits & 0x08) << 3) + 
                           ((ucOddBits & 0x04) << 3) + ((ucEvenBits & 0x04) << 2) + 
                           ((ucOddBits & 0x02) << 2) + ((ucEvenBits & 0x02) << 1) + 
                           ((ucOddBits & 0x01) << 1) + ((ucEvenBits & 0x01) << 0);
        }
        // calculate these worthless checksums
        uint16_t usHeaderCk = 0;
        for (uint32_t ui3 = 0; ui3 < 10; ui3++)
        {
            uint16_t usWord = (ucSwizBytes[ui3*2] << 8) + ucSwizBytes[ui3*2+1];
            usHeaderCk ^= usWord;
        }            
        uint16_t usDataCk = 0;
        for (uint32_t ui3 = 14; ui3 < 270; ui3++)
        {
            uint16_t usWord = (ucSwizBytes[ui3*2] << 8) + ucSwizBytes[ui3*2+1];
            usDataCk ^= usWord;
        }            
        uint32_t uiCalcHeaderCk = 0, uiCalcDataCk = 0;
        for (uint32_t ui4 = 0; ui4 < 16; ui4++)
        {
            uiCalcHeaderCk |= (usHeaderCk & (1 << ui4)) << ui4;
            uiCalcDataCk   |= (usDataCk   & (1 << ui4)) << ui4;
        }
        // Sector data
        m_uiSectorsRead++;
        uint32_t uiHeaderCk = (ucBytes[20] << 24) + (ucBytes[21] << 16) + (ucBytes[22] << 8) + ucBytes[23];
        uint32_t uiDataCk =   (ucBytes[24] << 24) + (ucBytes[25] << 16) + (ucBytes[26] << 8) + ucBytes[27];
        Serial.printf("Amiga Sector format: %02x  track: %i  Sector: %2i  SectorsToGap: %2i  Header cksum: %08x (%s)  Data cksum: %08x (%s)\r\n",
                      ucBytes[0], ucBytes[1], ucBytes[2], ucBytes[3],
                      uiHeaderCk, (uiHeaderCk == uiCalcHeaderCk ? "GOOD" : "BAD"), uiDataCk, (uiDataCk == uiCalcDataCk ? "GOOD" : "BAD"));
        return ui + 1;
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
