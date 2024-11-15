//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// FloppyTester.h 
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

#if !defined(FLOPPY_TESTER_H)
#define FLOPPY_TESTER_H

typedef enum _modulation { MOD_INVALID, MOD_MFM, MOD_GCR                       } geo_modulation_t;
typedef enum _geo_format { FMT_INVALID, FMT_IBM, FMT_ATARI, FMT_AMIGA, FMT_C64 } geo_format_t;

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
    uint8_t           ucSectorNum[21];
    uint8_t           ucSectorSide[21];
    uint8_t           ucSectorCylinder[21];
    uint8_t           ucSectorGoodID[21];
    uint8_t           ucSectorGoodData[21];
    uint32_t          uiSectorDataCRC[21];
} track_metadata_t;


#endif // FLOPPY_TESTER_H
