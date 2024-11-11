//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// CodecMFM.h
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

class DecoderMFM
{
public:
    DecoderMFM(const uint16_t *pusDeltaBuffers[], uint32_t uiDeltaMax);
    ~DecoderMFM();

    // accessors

    // modifiers
    void DecodeTrack(geo_format_t eFormat, bool bDebugPrint, track_metadata_t& rsMeta);

    // static helper
    static void advance_crc16(uint16_t& usCurrentCRC, const uint8_t* data_p, uint32_t length);
    
    // private methods
private:
    uint32_t ReadSectorBytesIBM(uint32_t uiStartIdx, bool bDebugPrint, track_metadata_t& rsMeta);
    uint32_t ReadSectorBytesAmiga(uint32_t uiStartIdx, bool bDebugPrint, track_metadata_t& rsMeta);

    // member variables
private:
    const uint16_t  **m_pusDeltaBuffers;
    const uint32_t    m_uiDeltaMax;
    uint32_t          m_uiSectorDataLength;
    uint16_t          m_usCurrentCRC;
};


class EncoderMFM
{
public:
    EncoderMFM(uint16_t *pusDeltaBuffers[], geo_format_t eFormat, int iSides, int iTracks, int iSectors);
    ~EncoderMFM();

    // accessors

    // modifiers
    uint32_t EncodeTrack(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);

    // private methods
private:
    uint32_t EncodeTrack360kIBM(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);
    uint32_t EncodeTrack720kIBM(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);
    uint32_t EncodeTrack800kAtari(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);
    uint32_t EncodeTrack880kAmiga(encoding_pattern_t ePattern, int iDriveTrack, int iDriveSide);
    
    void WriteBit(int iBit);
    void WriteByte(int iByte);
    void WriteSpecialC2C2C2(void);
    void WriteSpecialA1A1A1(void);
    void WriteSpecialA1A1(void);
    void calc_sector_data(encoding_pattern_t ePattern, uint32_t uiLength, uint32_t uiSectorNum, uint8_t *pucSectorData);
    void calc_amiga_sector(encoding_pattern_t ePattern, uint32_t uiDriveTrack, uint32_t uiDriveSide, uint32_t uiSectorNum, uint8_t pucSectorData[540]);
    uint16_t calc_id_crc(uint32_t uiDriveSide, uint32_t uiDriveTrack, uint32_t uiSectorNum, uint32_t uiSectorLength);
    uint16_t calc_data_crc(uint8_t *pucSectorData);

    // member variables
private:
    uint16_t          **m_pusDeltaBuffers;
    uint32_t            m_uiDeltaPos;
    const geo_format_t  m_eGeoFormat;
    const int           m_iGeoSides;
    const int           m_iGeoTracks;
    const int           m_iGeoSectors;

    // state
    int                 m_iOldBit;
    int                 m_iOldestBit;
};
