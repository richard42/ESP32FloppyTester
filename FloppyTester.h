//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// FloppyTester.h 
//
// Copyright (C) 2024, All Rights Reserved.
//
// Author: Richard Goedeken
// Date:   2024/02/10

#if !defined(FLOPPY_TESTER_H)
#define FLOPPY_TESTER_H

typedef enum _modulation { MOD_INVALID, MOD_MFM, MOD_GCR } geo_modulation_t;
typedef enum _geo_format { FMT_INVALID, FMT_IBM, FMT_AMIGA } geo_format_t;

typedef enum _encoding_pattern
{
  ENC_INVALID,
  ENC_ZEROS,
  ENC_ONES,
  ENC_SIXES,
  ENC_EIGHTS,
  ENC_RANDOM
} encoding_pattern_t;

typedef struct _track_metadata
{
    uint32_t          uiFluxCount;
    float             fPulseIntervalSpread;
    geo_modulation_t  eModulation;
    uint8_t           ucSectorsFound;
    uint8_t           ucSectorNum[12];
    uint8_t           ucSectorSide[12];
    uint8_t           ucSectorCylinder[12];
    uint8_t           ucSectorGoodID[12];
    uint8_t           ucSectorGoodData[12];
    uint32_t          uiSectorDataCRC[12];
} track_metadata_t;


#endif // FLOPPY_TESTER_H
