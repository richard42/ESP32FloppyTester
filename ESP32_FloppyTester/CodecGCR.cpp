//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// CodecGCR.cpp
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
#include "CodecGCR.h"

#define DELTA_ITEM(X) m_pusDeltaBuffers[(X)>>12][(X)&0x0fff]

//////////////////////////////////////////////////////////////////////////
// constructor and destructor

DecoderGCR::DecoderGCR(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax)
 : m_pusDeltaBuffers(pusDeltaBuffers)
 , m_uiDeltaMax(uiDeltaMax)
 , m_uiSectorDataLength(0)
{
}

DecoderGCR::~DecoderGCR()
{
}

//////////////////////////////////////////////////////////////////////////
// accessors

//////////////////////////////////////////////////////////////////////////
// static methods

bool DecoderGCR::DetectEncoding(const uint32_t auiCountByWavelen[16], const float afAverageByWavelen[16],
                                uint16_t *m_pusDeltaBuffers[], uint32_t uiDeltaMax, float fPeakCenters[3])
{
    // go through the histogram and find isolated peaks in the histogram
    uint32_t uiPeakIdx[4];
    uint32_t uiNumPeaks = 0;
    uint32_t uiThreshold = uiDeltaMax / 30;
    bool bAboveThresh = false;
    for (uint32_t uiIdx = 0; uiIdx < 16; uiIdx++)
    {
        if (auiCountByWavelen[uiIdx] > uiThreshold)
        {
            if (bAboveThresh == false)
            {
                bAboveThresh = true;
                uiPeakIdx[uiNumPeaks] = uiIdx;
            }
            else
            {
                if (auiCountByWavelen[uiIdx] > auiCountByWavelen[uiPeakIdx[uiNumPeaks]])
                {
                    uiPeakIdx[uiNumPeaks] = uiIdx;
                }
            }
        }
        else
        {
            if (bAboveThresh == true)
            {
                bAboveThresh = false;
                uiNumPeaks++;
            }
        }
        if (uiNumPeaks == 4)
            break;
    }
    if (bAboveThresh == true)
        uiNumPeaks++;

    // if this track is GCR encoded, we expect to see 3 peaks, at 1x, 2x, and 3x of the base wavelength
    if (uiNumPeaks != 3)
        return false;

    fPeakCenters[0] = afAverageByWavelen[uiPeakIdx[0]];
    fPeakCenters[1] = afAverageByWavelen[uiPeakIdx[1]];
    fPeakCenters[2] = afAverageByWavelen[uiPeakIdx[2]];

    // use an iterative algorithm to converge on the true centers of each histogram peak
    for (uint32_t uiLoopIter = 0; uiLoopIter < 20; uiLoopIter++)
    {
        // calculate the average position of each delta value within +/- 0.5 microseconds of each peak
        float fNewPeakSums[3] = {0, 0, 0};
        uint32_t uiNewPeakCnts[3] = {0, 0, 0};
        for (uint32_t ui = 0; ui < uiDeltaMax; ui++)
        {
            uint32_t uiDelta = DELTA_ITEM(ui);
            if (uiDelta & 0x8000)
            {
                ui++;
                continue;
            }
            const float fWavelen = uiDelta / 80.0f;
            for (uint32_t uiPeak = 0; uiPeak < 3; uiPeak++)
            {
                if (fabsf(fWavelen - fPeakCenters[uiPeak]) <= 0.5f)
                {
                    fNewPeakSums[uiPeak] += fWavelen;
                    uiNewPeakCnts[uiPeak]++;
                }
            }
        }
        float fNewPeakCenters[3];
        for (uint32_t uiPeak = 0; uiPeak < 3; uiPeak++)
        {
            fNewPeakCenters[uiPeak] = fNewPeakSums[uiPeak] / uiNewPeakCnts[uiPeak];
        }
        // bail out if it has converged
        float fMaxPeakMove = 0.0f;
        for (uint32_t uiPeak = 0; uiPeak < 3; uiPeak++)
        {
            fMaxPeakMove = std::max(fMaxPeakMove, fabsf(fPeakCenters[uiPeak] - fNewPeakCenters[uiPeak]));
        }
        if (fMaxPeakMove < 0.0005)
        {
            //Serial.printf("GCR peak discovery converged after %i iterations.\r\n", uiLoopIter+1);
            break;
        }
        // fail if we have hit our iteration limit
        if (uiLoopIter == 19)
            return false;
        // otherwise, update our centers and try again
        for (uint32_t uiPeak = 0; uiPeak < 3; uiPeak++)
        {
            fPeakCenters[uiPeak] = fNewPeakCenters[uiPeak];
        }
    }

    // final duration spacing check
    //Serial.printf("GCR wavelength peaks at: %.3f, %.3f, %.3f\r\n", fPeakCenters[0], fPeakCenters[1], fPeakCenters[2]);
    if (fabsf(fPeakCenters[1] / fPeakCenters[0] - 2.0f) / 2.0f > 0.15)
        return false;
    if (fabsf(fPeakCenters[2] / fPeakCenters[0] - 3.0f) / 3.0f > 0.15)
        return false;

    // looks like it is GCR
    return true;
}

//////////////////////////////////////////////////////////////////////////
// modifiers

void DecoderGCR::DecodeTrack(geo_format_t eFormat, bool bDebugPrint, track_metadata_t& rsMeta, const float fPeakCenters[3])
{
    // clear sector counter
    rsMeta.ucSectorsFound = 0;

    // calculate thresholds
    const float fThresh[4] = { (3 * fPeakCenters[0] - fPeakCenters[1]) / 2,
                               (fPeakCenters[0] + fPeakCenters[1]) / 2,
                               (fPeakCenters[1] + fPeakCenters[2]) / 2,
                               (3 * fPeakCenters[2] - fPeakCenters[1]) / 2 };
    
    // iterate over all delta samples, looking for markers
    uint32_t uiFoundOnes = 0;
    for (uint32_t ui = 0; ui < m_uiDeltaMax; ui++)
    {
        // get binned time delta value and reset state if long delay is found
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            uiFoundOnes = 0;
            ui++;
            continue;
        }
        const float fWavelen = uiDelta / 80.0f;
        if (fWavelen < fThresh[0] || fWavelen > fThresh[3])
        {
            uiFoundOnes = 0;
            continue;
        }
        const uint32_t uiWavelen = (fWavelen < fThresh[1] ? 1 : (fWavelen < fThresh[2] ? 2 : 3));
        // handle state
        if (uiWavelen == 1)
        {
            uiFoundOnes++;
            continue;
        }
        if (uiFoundOnes < 10)
        {
            uiFoundOnes = 0;
            continue;
        }
        // we have found at least 10 ones (which is a sync pattern) and now a gap, indicating the start of a sector
        //if (bDebugPrint)
        //    Serial.printf("Found sync pattern with %i ones (sector start).\r\n", uiFoundOnes);
        
        ui = ReadSectorBytesC64(ui, fThresh, bDebugPrint, rsMeta);
        uiFoundOnes = 0;
    }

    if (bDebugPrint)
        Serial.printf("%i C64 sectors read.\r\n", rsMeta.ucSectorsFound);
}

//////////////////////////////////////////////////////////////////////////
// private methods

uint32_t DecoderGCR::ReadSectorBytesC64(uint32_t uiStartIdx, const float fThresh[4], bool bDebugPrint, track_metadata_t& rsMeta)
{
    const int8_t chDecodeGCR[32] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, 8,  0,  1, -1, 12,  4,  5,
                                     -1, -1,  2,  3, -1, 15,  6,  7, -1, 9, 10, 11, -1, 13, 14, -1 };
    uint8_t ucBytes[512];
    uint8_t ucHeader[8];
    uint32_t uiBytesRead = 0;

    bool bWaitingForSync = false;
    bool bFoundHeader = false;
    bool bSectorAlreadyFound = false;
    uint32_t uiFoundOnes = 0;
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
        if (fWavelen < fThresh[0] || fWavelen > fThresh[3])
        {
            if (bWaitingForSync)
            {
                uiFoundOnes = 0;
                continue;
            }
            // error - invalid duration
            if (bDebugPrint)
                Serial.printf("[Sector read error: invalid duration %.3fus with uiBytesRead=%i]\r\n", fWavelen, uiBytesRead);
            return ui + 2;
        }
        const uint32_t uiWavelen = (fWavelen < fThresh[1] ? 1 : (fWavelen < fThresh[2] ? 2 : 3));
        // handle sync pattern between header and data
        if (bWaitingForSync)
        {
            if (uiWavelen == 1)
            {
                uiFoundOnes++;
                continue;
            }
            if (uiFoundOnes < 10)
            {
                uiFoundOnes = 0;
                continue;
            }
            bWaitingForSync = false;
            uiPriorBits = 1;
            uiNumPriorBits = uiWavelen;
            continue;
        }
        // accumulate bits
        uiPriorBits = (uiPriorBits << uiWavelen) + 1;
        uiNumPriorBits += uiWavelen;
        if (uiNumPriorBits < 10)
            continue;
        // write out one byte
        const uint32_t uiGCRWord = (uiPriorBits >> (uiNumPriorBits - 10));
        uiNumPriorBits -= 10;
        if (uiNumPriorBits == 0)
            uiPriorBits = 0;
        else
            uiPriorBits &= (1 << uiNumPriorBits) - 1;
        const char chHighNybble = chDecodeGCR[uiGCRWord >> 5];
        const char chLowNybble = chDecodeGCR[uiGCRWord & 31];
        if (chHighNybble == -1)
        {
            if (bDebugPrint)
                Serial.printf("[Sector read error: illegal GCR pattern 0x%02x]\r\n", uiGCRWord >> 5);
            return ui + 1;
        }
        if (chLowNybble == -1)
        {
            if (bDebugPrint)
                Serial.printf("[Sector read error: illegal GCR pattern 0x%02x]\r\n", uiGCRWord & 31);
            return ui + 1;
        }
        const uint8_t ucNewByte = (chHighNybble << 4) + chLowNybble;
        ucBytes[uiBytesRead++] = ucNewByte;
        // handle first byte (block ID)
        if (uiBytesRead == 1)
        {
            if (ucNewByte != 7 && ucNewByte != 8)
            {
                if (bDebugPrint)
                    Serial.printf("[Sector read error: illegal block ID 0x%02x]\r\n", ucNewByte);
                return ui + 1;
            }
            if (ucNewByte == 7 && !bFoundHeader)
            {
                // we found the data block but no header block, so just bail out
                return ui + 1;
            }
        }
        // handle completion of header block
        if (uiBytesRead == 8 && ucBytes[0] == 8)
        {
            // copy it to separate header array and wait for another sync pattern
            bFoundHeader = true;
            memcpy(ucHeader, ucBytes, 8);
            uiBytesRead = 0;
            bWaitingForSync = true;
            uiFoundOnes = 0;
            uiPriorBits = 0;
            uiNumPriorBits = 0;
            // Sector ID record
            const bool bChecksumGood = (ucHeader[1] == (ucHeader[2] ^ ucHeader[3] ^ ucHeader[4] ^ ucHeader[5]));
            if (bDebugPrint)
                Serial.printf("Sector ID: Track %i, Sector %i, Format ID=%02x%02x Checksum=%02x (%s)\r\n", ucHeader[3], ucHeader[2], ucHeader[4],
                              ucHeader[5], ucHeader[1], (bChecksumGood ? "GOOD" : "BAD"));
            // set metadata
            if (rsMeta.ucSectorsFound < 21)
            {
                for (uint32_t ui = 0; ui < rsMeta.ucSectorsFound; ui++)
                {
                    if (rsMeta.ucSectorNum[ui] == ucBytes[2] + 1)
                        bSectorAlreadyFound = true;
                }
                if (!bSectorAlreadyFound)
                {
                    rsMeta.ucSectorNum[rsMeta.ucSectorsFound] = ucBytes[2] + 1;
                    rsMeta.ucSectorSide[rsMeta.ucSectorsFound] = 0;
                    rsMeta.ucSectorCylinder[rsMeta.ucSectorsFound] = ucBytes[3] - 1;
                    rsMeta.ucSectorGoodID[rsMeta.ucSectorsFound] = (bChecksumGood ? 1 : 0);
                }
            }
            continue;
        }
        // handle completion of data block
        if (uiBytesRead == 258 && ucBytes[0] == 7)
        {
            // Sector data
            uint8_t ucDataChecksum = 0;
            for (uint32_t ui = 1; ui <= 0x100; ui++)
            {
                ucDataChecksum ^= ucBytes[ui];
            }
            const bool bDataChecksumGood = (ucBytes[0x101] == ucDataChecksum);
            if (bDebugPrint)
                Serial.printf("Sector data: 256 bytes with Checksum=%s\r\n", (bDataChecksumGood ? "GOOD" : "BAD"));
            // set metadata
            if (rsMeta.ucSectorsFound < 21 && !bSectorAlreadyFound)
            {
                rsMeta.ucSectorGoodData[rsMeta.ucSectorsFound] = (bDataChecksumGood ? 1 : 0);
                rsMeta.uiSectorDataCRC[rsMeta.ucSectorsFound] = ucDataChecksum;
                rsMeta.ucSectorsFound++;
            }
            return ui + 1;
        }
    }
}
