//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// CodecMFM.cpp
// 
// Copyright (C) 2024  Richard Goedeken
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <Arduino.h>

#include <map>

#include "FloppyTester.h"
#include "CodecMFM.h"

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

void DecoderMFM::DecodeTrack(geo_format_t eFormat, bool bDebugPrint, track_metadata_t& rsMeta)
{
    // clear sector counter
    rsMeta.ucSectorsFound = 0;
    
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
            if (uiFoundZeros < (eFormat == FMT_AMIGA ? 12 : 60))
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
        if (eFormat == FMT_AMIGA && uiFoundZeros >= 12 && pucSpecMatch == pucSpecialA1 && uiNumSpecMatch == 10)
        {
            ui = ReadSectorBytesAmiga(ui + 1, bDebugPrint, rsMeta) - 1;
            continue;
        }

        // normal cases for IBM/Atari
        if ((eFormat == FMT_IBM || eFormat == FMT_ATARI) && uiNumSpecMatch == uiMaxSpecMatch)
        {
            // found a marker
            if (pucSpecMatch == pucSpecialC2)
            {
                if (bDebugPrint)
                    Serial.printf("Found Index Access Marker (track start).\r\n");
                uiFoundZeros = 0;
                uiNumSpecMatch = 0;
                pucSpecMatch = NULL;
                continue;
            }
            else // (pucSpecMatch == pucSpecialA1)
            {
                ui = ReadSectorBytesIBM(ui + 1, bDebugPrint, rsMeta) - 1;
                uiFoundZeros = 0;
                uiNumSpecMatch = 0;
                pucSpecMatch = NULL;
                continue;
            }
        }
    }

    if (bDebugPrint)
        Serial.printf("%i %s sectors read.\r\n", rsMeta.ucSectorsFound, (eFormat == FMT_IBM ? "IBM" : (eFormat == FMT_ATARI ? "Atari" : "Amiga")));
}

//////////////////////////////////////////////////////////////////////////
// private methods

uint32_t DecoderMFM::ReadSectorBytesIBM(uint32_t uiStartIdx, bool bDebugPrint, track_metadata_t& rsMeta)
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
            if (bDebugPrint)
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
                if (bDebugPrint)
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
            else if (ucBytes[0] >= 0xf8 && ucBytes[0] <= 0xfb) // F8 F9 and FA were only used on TRS-80 disks
            {
                // Sector data
                if (m_uiSectorDataLength == 0)
                {
                    if (bDebugPrint)
                        Serial.printf("[Sector read error: no ID record, unknown sector length]\r\n");
                    return ui + 1;
                }
                uiExpectedBytes = m_uiSectorDataLength + 3;
            }
            else
            {
                if (bDebugPrint)
                    Serial.printf("[Sector read error: invalid address mark 0x%02x]\r\n", ucBytes[0]);
                return ui + 1;
            }
            // initialize CRC calculation
            uint8_t ucFirstBytes[3] = { 0xa1, 0xa1, 0xa1 };
            m_usCurrentCRC = 0xffff;
            advance_crc16(m_usCurrentCRC, ucFirstBytes, 3);
        }
        // handle block completion
        if (uiExpectedBytes > 0 && uiBytesRead == uiExpectedBytes)
        {
            // calculate CRC
            advance_crc16(m_usCurrentCRC, ucBytes, uiBytesRead);
            if (ucBytes[0] == 0xfe)
            {
                // Sector ID record
                if (bDebugPrint)
                    Serial.printf("Sector ID: Cylinder %i, Side %i, Sector %i, CRC=%04x (%s)\r\n", ucBytes[1], ucBytes[2], ucBytes[3],
                                  (ucBytes[5] << 8) + ucBytes[6], (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 1 << (7 + ucBytes[4]);
                // set metadata
                if (rsMeta.ucSectorsFound < 18)
                {
                    rsMeta.ucSectorNum[rsMeta.ucSectorsFound] = ucBytes[3];
                    rsMeta.ucSectorSide[rsMeta.ucSectorsFound] = ucBytes[2];
                    rsMeta.ucSectorCylinder[rsMeta.ucSectorsFound] = ucBytes[1];
                    rsMeta.ucSectorGoodID[rsMeta.ucSectorsFound] = (m_usCurrentCRC == 0 ? 1 : 0);
                }
                return ui + 1;
            }
            else if (ucBytes[0] == 0xfb)
            {
                // Sector data
                const uint16_t usDataCRC = (ucBytes[m_uiSectorDataLength+1] << 8) + ucBytes[m_uiSectorDataLength+2];
                if (bDebugPrint)
                    Serial.printf("Sector data: %i bytes with CRC=%04x (%s)\r\n", m_uiSectorDataLength,
                                  usDataCRC, (m_usCurrentCRC == 0 ? "GOOD" : "BAD"));
                m_uiSectorDataLength = 0;
                // set metadata
                if (rsMeta.ucSectorsFound < 18)
                {
                    rsMeta.ucSectorGoodData[rsMeta.ucSectorsFound] = (m_usCurrentCRC == 0 ? 1 : 0);
                    rsMeta.uiSectorDataCRC[rsMeta.ucSectorsFound] = usDataCRC;
                }
                rsMeta.ucSectorsFound++;
                return ui + 1;
            }
        }
    }
}

uint32_t DecoderMFM::ReadSectorBytesAmiga(uint32_t uiStartIdx, bool bDebugPrint, track_metadata_t& rsMeta)
{
    uint8_t ucSwizBytes[540];
    uint8_t ucBytes[540];
    uint32_t uiBytesRead = 0;
    
    uint32_t uiLastBit = 1;
    uint32_t uiPriorBits = 0;
    uint32_t uiNumPriorBits = 0;
    for (volatile uint32_t uiDeltaIdx = uiStartIdx; uiDeltaIdx < m_uiDeltaMax; uiDeltaIdx++)
    {
        if (uiDeltaIdx >= m_uiDeltaMax)  // this is to work around some horrible compiler bug
        {
            return uiDeltaIdx;
        }
        // get binned time delta value and bail out if long delay is found
        const uint32_t uiDelta = DELTA_ITEM(uiDeltaIdx);
        if (uiDelta & 0x8000)
        {
            // error - long delay
            if (bDebugPrint)
                Serial.printf("[Sector read error: long delay] (position %i/%i)\r\n", uiDeltaIdx, m_uiDeltaMax);
            return uiDeltaIdx + 2;
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
                if (bDebugPrint)
                    Serial.printf("[Sector read error: missing clock pulse] (position %i/%i)\r\n", uiDeltaIdx, m_uiDeltaMax);
                return uiDeltaIdx + 1;
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
        uint32_t uiHeaderCk = (ucBytes[20] << 24) + (ucBytes[21] << 16) + (ucBytes[22] << 8) + ucBytes[23];
        uint32_t uiDataCk =   (ucBytes[24] << 24) + (ucBytes[25] << 16) + (ucBytes[26] << 8) + ucBytes[27];
        if (bDebugPrint)
            Serial.printf("Amiga Sector format: %02x  track: %i  Sector: %2i  SectorsToGap: %2i  Header cksum: %08x (%s)  Data cksum: %08x (%s)\r\n",
                          ucBytes[0], ucBytes[1], ucBytes[2], ucBytes[3],
                          uiHeaderCk, (uiHeaderCk == uiCalcHeaderCk ? "GOOD" : "BAD"), uiDataCk, (uiDataCk == uiCalcDataCk ? "GOOD" : "BAD"));
        // set metadata
        if (rsMeta.ucSectorsFound < 18)
        {
            rsMeta.ucSectorNum[rsMeta.ucSectorsFound] = ucBytes[2] + 1;
            rsMeta.ucSectorSide[rsMeta.ucSectorsFound] = ucBytes[1] & 1;
            rsMeta.ucSectorCylinder[rsMeta.ucSectorsFound] = ucBytes[1] >> 1;
            rsMeta.ucSectorGoodID[rsMeta.ucSectorsFound] = ((uiHeaderCk == uiCalcHeaderCk && ucBytes[0] == 0xff) ? 1 : 0);
            rsMeta.ucSectorGoodData[rsMeta.ucSectorsFound] = (uiDataCk == uiCalcDataCk ? 1 : 0);
            rsMeta.uiSectorDataCRC[rsMeta.ucSectorsFound] = uiDataCk;
        }
        rsMeta.ucSectorsFound++;
        return uiDeltaIdx + 1;
    }
}

//////////////////////////////////////////////////////////////////////////
// static helper method

void DecoderMFM::advance_crc16(uint16_t& usCurrentCRC, const uint8_t* data_p, uint32_t length)
{
    unsigned char x;

    while (length--)
    {
        x = usCurrentCRC >> 8 ^ *data_p++;
        x ^= x>>4;
        usCurrentCRC = (usCurrentCRC << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
}

//////////////////////////////////////////////////////////////////////////
// constructor/destructor for encoding class

EncoderMFM::EncoderMFM(uint16_t *pusDeltaBuffers[], geo_format_t eFormat, int iSides, int iTracks, int iSectors)
 : EncoderBase(pusDeltaBuffers, eFormat, iSides, iTracks, iSectors)
 , m_iOldBit(0)
 , m_iOldestBit(0)
{
}

EncoderMFM::~EncoderMFM()
{
}

//////////////////////////////////////////////////////////////////////////
// modifiers for encoding class

uint32_t EncoderMFM::EncodeTrack(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide)
{
    if (m_eGeoFormat == FMT_IBM)
    {
        if (m_iGeoTracks == 40)
          return EncodeTrack360kIBM(ePattern, iDriveTrack, iDriveSide);
        else
          return EncodeTrack720kIBM(ePattern, iDriveTrack, iDriveSide);
    }
    else if (m_eGeoFormat == FMT_ATARI)
    {
        return EncodeTrack800kAtari(ePattern, iDriveTrack, iDriveSide);
    }
    else // (m_eGeoFormat == FMT_AMIGA)
    {
        return EncodeTrack880kAmiga(ePattern, iDriveTrack, iDriveSide);
    }
}

uint32_t EncoderMFM::EncodeTrack360kIBM(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide)
{
    // 200ms / 4us = 50,000 bits / 8 = 6250 bytes per track

    // post-index gap
    for (uint32_t ui = 0; ui < 32; ui++)
    {
        WriteByte(0x4e);
    }

    for (uint32_t uiSector = 1; uiSector <= m_iGeoSectors; uiSector++)
    {
        uint8_t ucSectorData[256];
        calc_sector_data(ePattern, 256, uiSector, ucSectorData);
        // ID Record
        for (uint32_t ui = 0; ui < 8; ui++)
        {
            WriteByte(0x00);
        }
        WriteSpecialA1A1A1();
        WriteByte(0xfe);
        WriteByte(iDriveTrack);
        WriteByte(iDriveSide);
        WriteByte(uiSector);
        WriteByte(1);              // Sector Length 1 == 256 bytes
        const uint16_t usIDCRC = calc_id_crc(iDriveSide, iDriveTrack, uiSector, 1);
        WriteByte(usIDCRC >> 8);
        WriteByte(usIDCRC & 0xff);
        // ID Gap
        for (uint32_t ui = 0; ui < 22; ui++)
        {
            WriteByte(0x4e);
        }
        for (uint32_t ui = 0; ui < 12; ui++)
        {
            WriteByte(0x00);
        }
        WriteSpecialA1A1A1();
        // Data field record
        WriteByte(0xfb);
        for (uint32_t uiDataIdx = 0; uiDataIdx < 256; uiDataIdx++)
        {
            WriteByte(ucSectorData[uiDataIdx]);
        }
        const uint16_t usDataCRC = calc_data_crc(ucSectorData, 256);
        WriteByte(usDataCRC >> 8);
        WriteByte(usDataCRC & 0xff);
        // data gap
        for (uint32_t ui = 0; ui < 24; ui++)
        {
            WriteByte(0x4e);
        }
    }

    // gap 4
    for (uint32_t ui = 0; ui < 134; ui++)
    {
        WriteByte(0x4e);
    }

    return m_uiDeltaPos;
}

uint32_t EncoderMFM::EncodeTrack720kIBM(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide)
{
    //const uint32_t uiBytesPreIndexGap = 96;
    //const uint32_t uiBytesIndexGap = 65;
    //const uint32_t uiBytesIDGap = 7 + 37;    // including sector header
    //const uint32_t uiBytesSectorData = 515;
    //const uint32_t uiBytesDataGap = 69;
    //const uint32_t uiBytesGap4 = 652;
    // total = 6396 bytes
    // 200ms / 4us = 50,000 bits / 8 = 6250 bytes per track

    // pre-index gap
    for (uint32_t ui = 0; ui < 80; ui++)
    {
        WriteByte(0x4e);
    }
    for (uint32_t ui = 0; ui < 12; ui++)
    {
        WriteByte(0x00);
    }
    WriteSpecialC2C2C2();
    WriteByte(0xfc);

    // post-index gap
    for (uint32_t ui = 0; ui < 50; ui++)
    {
        WriteByte(0x4e);
    }
    for (uint32_t ui = 0; ui < 12; ui++)
    {
        WriteByte(0x00);
    }
    WriteSpecialA1A1A1();

    for (uint32_t uiSector = 1; uiSector <= m_iGeoSectors; uiSector++)
    {
        uint8_t ucSectorData[512];
        calc_sector_data(ePattern, 512, uiSector, ucSectorData);
        // ID Record
        WriteByte(0xfe);
        WriteByte(iDriveTrack);
        WriteByte(iDriveSide);
        WriteByte(uiSector);
        WriteByte(2);              // Sector Length 2 == 512 bytes
        const uint16_t usIDCRC = calc_id_crc(iDriveSide, iDriveTrack, uiSector, 2);
        WriteByte(usIDCRC >> 8);
        WriteByte(usIDCRC & 0xff);
        // ID Gap
        for (uint32_t ui = 0; ui < 22; ui++)
        {
            WriteByte(0x4e);
        }
        for (uint32_t ui = 0; ui < 12; ui++)
        {
            WriteByte(0x00);
        }
        WriteSpecialA1A1A1();
        // Data field record
        WriteByte(0xfb);
        for (uint32_t uiDataIdx = 0; uiDataIdx < 512; uiDataIdx++)
        {
            WriteByte(ucSectorData[uiDataIdx]);
        }
        const uint16_t usDataCRC = calc_data_crc(ucSectorData, 512);
        WriteByte(usDataCRC >> 8);
        WriteByte(usDataCRC & 0xff);
        // data gap
        if (uiSector < m_iGeoSectors)
        {
            for (uint32_t ui = 0; ui < 54; ui++)
            {
                WriteByte(0x4e);
            }
            for (uint32_t ui = 0; ui < 12; ui++)
            {
                WriteByte(0x00);
            }
            WriteSpecialA1A1A1();
        }
    }

    // gap 4
    for (uint32_t ui = 0; ui < 506; ui++)
    {
        WriteByte(0x4e);
    }

    return m_uiDeltaPos;
}

uint32_t EncoderMFM::EncodeTrack800kAtari(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide)
{
    // starting 0x4e bytes
    const uint32_t uiStartBytes = (m_iGeoSectors == 10) ? 40 : 76;
    for (uint32_t ui = 0; ui < uiStartBytes; ui++)
    {
        WriteByte(0x4e);
    }

    for (uint32_t uiSector = 1; uiSector <= m_iGeoSectors; uiSector++)
    {
        uint8_t ucSectorData[512];
        calc_sector_data(ePattern, 512, uiSector, ucSectorData);
        // pre-ID sync bytes
        const uint32_t uiStartIDBytes = (m_iGeoSectors == 10) ? 36 : 3;
        for (uint32_t ui = 0; ui < uiStartIDBytes; ui++)
        {
            WriteByte(0x4e);
        }
        for (uint32_t ui = 0; ui < 12; ui++)
        {
            WriteByte(0x00);
        }
        WriteSpecialA1A1A1();
        // ID Record
        WriteByte(0xfe);
        WriteByte(iDriveTrack);
        WriteByte(iDriveSide);
        WriteByte(uiSector);
        WriteByte(2);              // Sector Length 2 == 512 bytes
        const uint16_t usIDCRC = calc_id_crc(iDriveSide, iDriveTrack, uiSector, 2);
        WriteByte(usIDCRC >> 8);
        WriteByte(usIDCRC & 0xff);
        // pre-data sync bytes
        for (uint32_t ui = 0; ui < 24; ui++)
        {
            WriteByte(0x4e);
        }
        for (uint32_t ui = 0; ui < 12; ui++)
        {
            WriteByte(0x00);
        }
        WriteSpecialA1A1A1();
        // Data field record
        WriteByte(0xfb);
        for (uint32_t uiDataIdx = 0; uiDataIdx < 512; uiDataIdx++)
        {
            WriteByte(ucSectorData[uiDataIdx]);
        }
        const uint16_t usDataCRC = calc_data_crc(ucSectorData, 512);
        WriteByte(usDataCRC >> 8);
        WriteByte(usDataCRC & 0xff);
    }

    // ending 0x4e bytes
    const uint32_t uiEndBytes = (m_iGeoSectors == 10) ? 90 : 82;
    for (uint32_t ui = 0; ui < uiEndBytes; ui++)
    {
        WriteByte(0x4e);
    }

    return m_uiDeltaPos;
}

uint32_t EncoderMFM::EncodeTrack880kAmiga(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide)
{
    // starting zero bytes
    for (uint32_t ui = 0; ui < 512; ui++)
    {
        WriteByte(0x00);
    }

    // all the sectors, one right after another (should be 11 of them)
    for (uint32_t uiSector = 1; uiSector <= m_iGeoSectors; uiSector++)
    {
        uint8_t ucSectorData[540];
        calc_amiga_sector(ePattern, iDriveTrack, iDriveSide, uiSector, ucSectorData);
        // pre-ID sync bytes
        WriteByte(0x00);
        WriteByte(0x00);
        WriteSpecialA1A1();
        // ID Record plus data bytes
        for (uint32_t ui = 0; ui < 540; ui++)
        {
            WriteByte(ucSectorData[ui]);
        }
    }

    return m_uiDeltaPos;
}

//////////////////////////////////////////////////////////////////////////
// private helper methods

void EncoderMFM::WriteBit(int iBit)
{
    // deal with different speeds - atari 880k disks write at 261khz instead of 250khz
    const uint32_t uiBitTime = (m_eGeoFormat == FMT_ATARI && m_iGeoSectors == 11) ? (80000 / 261) : (80000 / 250);
    
    if (iBit == m_iOldBit)
    {
        if (m_iOldestBit == 1 && m_iOldBit == 0)
            DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;
        else
            DELTA_ITEM(m_uiDeltaPos) = uiBitTime;
        m_uiDeltaPos++;
        m_iOldestBit = m_iOldBit;
        return;
    }
    else if (m_iOldBit == 0 && iBit == 1)
    {
        if (m_iOldestBit == 1)
            DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;
        else
            DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;
        m_uiDeltaPos++;
        m_iOldestBit = 0;
        m_iOldBit = 1;
        return;
    }
    // m_iOldBit == 1 && iBit == 0
    m_iOldestBit = 1;
    m_iOldBit = 0;
    return;
}

void EncoderMFM::WriteByte(int iByte)
{
    for (uint32_t uiBitIdx = 0; uiBitIdx < 8; uiBitIdx++)
    {
        WriteBit((iByte >> (7 - uiBitIdx)) & 1);
    }
}

void EncoderMFM::WriteSpecialC2C2C2(void)
{
    // this is only used on IBM 720k disks
    
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 4 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
  
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 4 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;

    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 4 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;

    m_iOldestBit = 1;
    m_iOldBit = 0;
}

void EncoderMFM::WriteSpecialA1A1A1(void)
{
    // deal with different speeds - atari 880k disks write at 261khz instead of 250khz
    const uint32_t uiBitTime = (m_eGeoFormat == FMT_ATARI && m_iGeoSectors == 11) ? (80000 / 261) : (80000 / 250);

    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;        m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;         m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;
  
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime;            m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;        m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;        m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;

    DELTA_ITEM(m_uiDeltaPos) = uiBitTime;            m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;        m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 2;        m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = uiBitTime * 3 / 2;    m_uiDeltaPos++;

    m_iOldestBit = 0;
    m_iOldBit = 1;
}

void EncoderMFM::WriteSpecialA1A1(void)
{
    // this is only used on Amiga 880k disks, at 250khz
    
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
  
    DELTA_ITEM(m_uiDeltaPos) = 4 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 8 * 80;    m_uiDeltaPos++;
    DELTA_ITEM(m_uiDeltaPos) = 6 * 80;    m_uiDeltaPos++;

    m_iOldestBit = 0;
    m_iOldBit = 1;
}

void EncoderMFM::calc_sector_data(encoding_pattern_t ePattern, uint32_t uiLength, uint32_t uiSectorNum, uint8_t *pucSectorData)
{
    // seed PRNG if necessary
    if (m_eGeoFormat == ENC_RANDOM)
    {
        uint32_t uiCurCycles;
        asm volatile("  rsr %0, ccount                \n"
                     : "=a"(uiCurCycles) );
        srand(uiCurCycles);
    }

    // write bytes
    for (uint32_t uiByteIdx = 0; uiByteIdx < uiLength; uiByteIdx++)
    {
        switch(ePattern)
        {
            case ENC_ZEROS:
                pucSectorData[uiByteIdx] = 0x00;
                break;
            case ENC_ONES:
                pucSectorData[uiByteIdx] = 0xFF;
                break;
            case ENC_SIXES:
                if ((uiByteIdx % 3) == 0)
                    pucSectorData[uiByteIdx] = 0x24;
                else if ((uiByteIdx % 3) == 1)
                    pucSectorData[uiByteIdx] = 0x92;
                else // ((uiByteIdx % 3) == 2)
                    pucSectorData[uiByteIdx] = 0x49;
                break;
            case ENC_EIGHTS:
                if (m_eGeoFormat == FMT_AMIGA)
                {
                    pucSectorData[uiByteIdx] = 0x33;
                }
                else
                {
                    pucSectorData[uiByteIdx] = 0x55;
                }
                break;
            case ENC_RANDOM:
                pucSectorData[uiByteIdx] = (rand() & 0xff);
                break;
        }
    }
}

void EncoderMFM::calc_amiga_sector(encoding_pattern_t ePattern, uint32_t uiDriveTrack, uint32_t uiDriveSide, uint32_t uiSectorNum, uint8_t pucSwizBytes[540])
{
    // start by writing the straight (un-swizzled) bytes for the sector header+data
    uint8_t ucBytes[540];

    // ID Record
    ucBytes[0] = 0xff;
    ucBytes[1] = (uiDriveTrack * 2 + uiDriveSide);
    ucBytes[2] = uiSectorNum - 1;
    ucBytes[3] = m_iGeoSectors + 1 - uiSectorNum;
    for (uint32_t ui = 4; ui < 20; ui++)
    {
        ucBytes[ui] = 0;    // OS recovery info
    }
    for (uint32_t ui = 20; ui < 28; ui++)
    {
        ucBytes[ui] = 0;    // checksums
    }

    // fill in track data
    calc_sector_data(ePattern, 512, uiSectorNum, ucBytes + 28);

    // multiplex (swizzle) the bytes
    memset(pucSwizBytes, 0, 540);
    for (uint32_t ui = 0; ui < 540; ui++)
    {
        uint32_t uiBlockStart = 0, uiPairOffset = 0;
        if (ui < 4)
            { uiBlockStart = 0;  uiPairOffset = 2; }
        else if (ui < 20)
            { uiBlockStart = 4;  uiPairOffset = 8; }
        else if (ui < 24)
            { uiBlockStart = 20; uiPairOffset = 2; }
        else if (ui < 28)
            { uiBlockStart = 24; uiPairOffset = 2; }
        else
            { uiBlockStart = 28; uiPairOffset = 256; }
        const uint32_t uiOddIdx = (ui - uiBlockStart) / 2 + uiBlockStart;
        const uint32_t uiEvenIdx = uiOddIdx + uiPairOffset;
        const uint8_t ucByte = ucBytes[ui];
        const uint8_t ucOddBits =  ((ucByte & 0x80) >> 4) + ((ucByte & 0x20) >> 3) + ((ucByte & 0x08) >> 2) + ((ucByte & 0x02) >> 1);
        const uint8_t ucEvenBits = ((ucByte & 0x40) >> 3) + ((ucByte & 0x10) >> 2) + ((ucByte & 0x04) >> 1) + (ucByte & 0x01);
        if (ui & 1)
        {
            pucSwizBytes[uiOddIdx]  |= ucOddBits;
            pucSwizBytes[uiEvenIdx] |= ucEvenBits;
        }
        else
        {
            pucSwizBytes[uiOddIdx]  |= (ucOddBits << 4);
            pucSwizBytes[uiEvenIdx] |= (ucEvenBits << 4);
        }
    }

    // calculate these worthless checksums
    uint16_t usHeaderCk = 0;
    for (uint32_t ui = 0; ui < 10; ui++)
    {
        uint16_t usWord = (pucSwizBytes[ui*2] << 8) + pucSwizBytes[ui*2+1];
        usHeaderCk ^= usWord;
    }
    uint16_t usDataCk = 0;
    for (uint32_t ui = 14; ui < 270; ui++)
    {
        uint16_t usWord = (pucSwizBytes[ui*2] << 8) + pucSwizBytes[ui*2+1];
        usDataCk ^= usWord;
    }
    uint32_t uiHeaderCk = 0, uiDataCk = 0;
    for (uint32_t ui = 0; ui < 16; ui++)
    {
        uiHeaderCk |= (usHeaderCk & (1 << ui)) << ui;
        uiDataCk   |= (usDataCk   & (1 << ui)) << ui;
    }

    // write the straight checksums into the straight data buffer
    ucBytes[20] = ((uiHeaderCk >> 24) & 0xff);
    ucBytes[21] = ((uiHeaderCk >> 16) & 0xff);
    ucBytes[22] = ((uiHeaderCk >>  8) & 0xff);
    ucBytes[23] = ((uiHeaderCk >>  0) & 0xff);
    ucBytes[24] = ((uiDataCk >> 24) & 0xff);
    ucBytes[25] = ((uiDataCk >> 16) & 0xff);
    ucBytes[26] = ((uiDataCk >>  8) & 0xff);
    ucBytes[27] = ((uiDataCk >>  0) & 0xff);

    // and finally, swizzle the checksums
    for (uint32_t ui = 20; ui < 28; ui++)
    {
        const uint32_t uiBlockStart = (ui < 24) ? 20 : 24;
        const uint32_t uiPairOffset = 2;
        const uint32_t uiOddIdx = (ui - uiBlockStart) / 2 + uiBlockStart;
        const uint32_t uiEvenIdx = uiOddIdx + uiPairOffset;
        const uint8_t ucByte = ucBytes[ui];
        const uint8_t ucOddBits =  ((ucByte & 0x80) >> 4) + ((ucByte & 0x20) >> 3) + ((ucByte & 0x08) >> 2) + ((ucByte & 0x02) >> 1);
        const uint8_t ucEvenBits = ((ucByte & 0x40) >> 3) + ((ucByte & 0x10) >> 2) + ((ucByte & 0x04) >> 1) + (ucByte & 0x01);
        if (ui & 1)
        {
            pucSwizBytes[uiOddIdx]  |= ucOddBits;
            pucSwizBytes[uiEvenIdx] |= ucEvenBits;
        }
        else
        {
            pucSwizBytes[uiOddIdx]  |= (ucOddBits << 4);
            pucSwizBytes[uiEvenIdx] |= (ucEvenBits << 4);
        }
    }
}

uint16_t EncoderMFM::calc_id_crc(uint32_t uiDriveSide, uint32_t uiDriveTrack, uint32_t uiSectorNum, uint32_t uiSectorLength)
{
    uint8_t ucRecordBytes[8] = { 0xa1, 0xa1, 0xa1, 0xfe, uiDriveTrack, uiDriveSide, uiSectorNum, uiSectorLength };

    uint16_t usCurrentCRC = 0xffff;
    DecoderMFM::advance_crc16(usCurrentCRC, ucRecordBytes, 8);
    return usCurrentCRC;
}

uint16_t EncoderMFM::calc_data_crc(uint8_t *pucSectorData, uint32_t uiDataLength)
{
    uint16_t usCurrentCRC = 0xffff;

    uint8_t ucDataAddrMarkBytes[4] = { 0xa1, 0xa1, 0xa1, 0xfb };
    DecoderMFM::advance_crc16(usCurrentCRC, ucDataAddrMarkBytes, 4);

    DecoderMFM::advance_crc16(usCurrentCRC, pucSectorData, uiDataLength);

    return usCurrentCRC;
}
