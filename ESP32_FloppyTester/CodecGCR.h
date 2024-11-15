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
