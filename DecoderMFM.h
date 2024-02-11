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

    // static helper
    static void advance_crc16(uint16_t& usCurrentCRC, const uint8_t* data_p, uint32_t length);
    
    // private methods
private:
    uint32_t ReadSectorBytesIBM(uint32_t uiStartIdx);
    uint32_t ReadSectorBytesAmiga(uint32_t uiStartIdx);

    // member variables
private:
    const uint16_t  **m_pusDeltaBuffers;
    const uint32_t    m_uiDeltaMax;
    uint32_t          m_uiSectorDataLength;
    uint16_t          m_usCurrentCRC;
    uint32_t          m_uiSectorsRead;
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
    void WriteBit(int iBit);
    void WriteByte(int iByte);
    void WriteSpecialC2C2C2(void);
    void WriteSpecialA1A1A1(void);
    void calc_sector_data(encoding_pattern_t ePattern, uint32_t uiSectorNum, uint8_t *pucSectorData);
    uint16_t calc_id_crc(uint32_t uiDriveSide, uint32_t uiDriveTrack, uint32_t uiSectorNum);
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
