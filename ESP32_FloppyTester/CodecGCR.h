//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// CodecGCR.h
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

#include "FloppyTester.h"

class DecoderGCR
{
public:
    DecoderGCR(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax);
    ~DecoderGCR();

    // accessors

    // modifiers
    void DecodeTrack(geo_format_t eFormat, bool bDebugPrint, track_metadata_t& rsMeta, const float fPeakCenters[3]);

    // static helper
    static bool DetectEncoding(const uint32_t auiCountByWavelen[16], const float afAverageByWavelen[16],
                               uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax, float fPeakCenters[3]);

    
    // private methods
private:
    uint32_t ReadSectorBytesC64(uint32_t uiStartIdx, const float fThresh[4], bool bDebugPrint, track_metadata_t& rsMeta);

    // member variables
private:
    const uint16_t  **m_pusDeltaBuffers;
    const uint32_t    m_uiDeltaMax;
    uint32_t          m_uiSectorDataLength;
};

class EncoderGCR : public EncoderBase
{
public:
    EncoderGCR(uint16_t *pusDeltaBuffers[], geo_format_t eFormat, int iSides, int iTracks, int iSectors);
    ~EncoderGCR();

    // accessors

    // modifiers
    uint32_t EncodeTrack(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);

    // private methods
private:
    uint32_t EncodeTrack180kC64(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);

    void WriteBit(int iBit);
    void WriteNybble(int iNybble);
    void WriteByte(int iByte);
    void calc_sector_data(encoding_pattern_t ePattern, uint32_t uiLength, uint32_t uiSectorNum, uint8_t *pucSectorData);

    // member variables
private:
    // state
    uint32_t            m_uiBaseTime;
    uint32_t            m_uiZeroes;
};
