# ESP32_FloppyTester README

## What it is

The ESP32_FloppyTester is an open-source hardware/software project which aims to create a software-based floppy drive controller and tester with very low-cost hardware.  The materials cost for the interface board is under 10 dollars, and aside from a floppy drive cable the only other part necessary is an ESP32 development board, which can also be had for under 10 dollars.

The user communicates with the ESP32_FloppyTester via a serial monitor program on a PC or laptop, connected to the ESP32 development board by a USB cable.

The user gives commands to the ESP32_FloppyTester through the serial terminal, and as the desired operations are performed, statistics and results are sent back to the user.

The focus of this project is creating an easily-hackable tool which can be used in the diagnosis and repair of vintage 3.5" and 5.25" floppy drives.

## What it is NOT

The ESP32_FloppyTester is not designed to read or write disk image files for copying or preservation of digital media.  There are numerous projects already available which serve this function well.  Before creating the ESP32_FloppyTester, I purchased and used the KryoFlux, the FluxEngine, and the GreaseWeazle. These are all very capable products for recording and capturing data from floppy drives.

The focus of this project is to give the user very low-level control over the operation of a floppy drive so that if it is defective, the nature and root cause of the defect(s) may be determined and remedied.

## Contents

* [Hardware Warning](#hardware-warning)
* [Setup](#setup)
* [Basic Usage](#basic-usage)
* [Advanced Usage](#advanced-usage)
* [Caveats](caveats)

### Hardware Warning

There is one important flaw with the current interface board design, and users must be aware of this.

When the ESP32 microcontroller is being flashed by the ESP-IDF development system, all of its pins go into a high-impedance state with pull-up resistors.

As a result, all of the output lines will go active on any connected floppy drive. In particular, the SELECT, MOTOR ON, SIDE 1, and WRITE GATE lines will all be active during the ESP32 flashing process.

Because of this, if there is a disk inserted into the connected floppy drive, and it is not write protected, the track on side 1 which is under the drive head will be erased.

As a workaround, the user should always make sure that the drive has no disk in it when re-flashing the ESP32 software.

### Setup

To build the interface board, you may send the Gerber files in the "hardware" sub-directory to a printed-circuit board manufacturer such as PCBWay or JLCPCB. Order the remaining parts from digi-key and solder the board together once it all arrives. The "doc" sub-folder contains a Bill of Materials and pictures of the finished product.

The next step is to set up the [arduino IDE software](https://www.arduino.cc/en/software) and configure it for your ESP32 board. Then just load up the ESP32_FloppyTest.ino source code in your Arduino IDE and click the "Upload" button to compile the code and flash the ESP32 device. (But make sure there is no floppy disk in the drive, if you have one connected!)

Before connecting a drive the first time, it is good to check your soldering with the `TEST INPUT` and `TEST OUTPUT` commands.

When in the `TEST INPUT` mode, the software will poll for changes on any input pins and print the names and states of the pins when they change. You may use a jumper wire to bridge each input pin to a 5-volt supply, and observe the change via the serial monitor.

In the `TEST OUTPUT` mode, each output pin will oscillate with a binary pattern corresponding to the number of the pin. This pattern may be observed by connecting an oscilloscope to each output pin in turn. This test mode should never be used when a floppy drive is connected to the 34-pin connector, or **bad** things may happen.

Either test mode may be exited by sending a blank line or any characters to the ESP32.

### Basic Usage

Use the `HELP` command to view a list of all supported commands.

The floppy drive hardware interface, though common across many different drives and vintage computer systems, uses 5 different pins for selecting a particular drive and turning on its motor. The software currently defaults to a *shugart*-style interface with drive 0, such as would be configured for an internal drive on an Amiga computer. PC/Atari drives use different pins for drive selection, and so the user must use the `DETECT PINS` command to set the correct pins before anything else will work.

The `DETECT PINS` command activates each pin individually, and requests feedback from the user based upon how it affects the status of the floppy drive. At each step, the user must tell the ESP32_FloppyTester if the drive light has been illuminated, or if the motor has been activated. After answering all 5 prompts, the software will configure itself to properly address your floppy drive, and all of the other commands should then work.  If the detection fails and the drive light or motor never comes on, this indicates a failure in the drive's selection or motor control circuitry.

The next diagnostic step is to use the `DETECT RPM` command. A floppy disk must be inserted for this to work.  This command will enable the motor and begin sampling the INDEX pin, printing out the current drive RPM speed once per second. If this command fails, it indicates a problem with the index hole detection circuitry, or no floppy disk in the drive.

The final basic drive test can be performed with the `SEEK TEST` command. This will cause the drive to seek outwards until the TRACK0 line goes active, and then seek back and forth several times and ensure that track 0 is reached each time after returning there. If this command fails, there is likely to be a problem with the stepper motor drive circuitry or the track 0 detection mechanism.

### Advanced Usage

After all of the basic drive tests have passed, the user can move on to more advanced testing. The full command list is as follows:

```    HELP           - display this help message.
    TEST INPUT     - activate input test mode, and print level changes on input lines.
    TEST OUTPUT    - activate output test mode, and drive output pins with binary pin numbers.
    DETECT PINS    - toggle select lines to determine drive wiring/jumper setup.
    DETECT RPM     - spin up motor and calculate spindle speed from index pulses.
    DETECT STATUS  - show status of Track0, Write Protect, and Disk Change signal.
    MOTOR ON/OFF   - activate or de-activate motor 1
    SEEK HOME      - move to track 0.
    SEEK TEST      - test head seeking and track 0 detection.
    SEEK <X>       - seek head to track X.
    SEEK           - display current track number.
    SIDE <X>       - use side X (0 or 1) for single-sided commands.
    SIDE           - display current side number.
    GEOMETRY ...   - set/show current drive geometry. Examples:
             IBM <SS/DS> <40/80> <9/10/11> - set IBM format, sides, track count, sector count.
             AMIGA                         - set Amiga format, 2 sides, 80 tracks, 11 sectors.
    TRACK READ     - read current track and print decoded data summary (requires formatted disk).
    TRACK ERASE    - erase current track and validate erasure (destroys data on disk).
    TRACK WRITE ...- write track data with current format and given pattern (destroys data on disk).
                ZEROS     - all sector data is binary 0 (MFM 4us intervals).
                ONES      - all sector data is binary 1 (MFM 4us intervals).
                SIXES     - all sector data is alternating bits (MFM 6us intervals).
                EIGHTS    - all sector data is alternating bits (MFM 8us intervals).
                RANDOM    - all sector data is random.
    DISK READ      - read all tracks on disk and display track/sector status.
    DISK ERASE     - erase all tracks on the disk (destroys data on disk).
    DISK WRITE ... - write all tracks on disk with current format and given pattern (destroys data on disk).
    DISK RWTEST    - write and verify all tracks on disks with 4 different patterns (destroys data on disk).
```
#### DETECT STATUS
The `DETECT STATUS` command may be used to read the state of 3 input lines, which can indicate if the write protect switch/LED and the disk change circuitry (if supported by the drive) is working.

#### MOTOR ON
The user may activate the motor using the `MOTOR ON` command, and then the other commands will leave the motor running after they complete. This could be useful if a particular drive takes longer than the specified 0.5 seconds to settle after motor startup.

#### TRACK READ
The `TRACK READ` command captures the disk pulses from one revolution at the current head position, generates some statistics about the pulse timing intervals, and tries to decode any sector data found. This command is affected by the current drive geometry. The drive geometry must be set to "amiga" if reading amiga disks, or the default IBM if reading IBM or Atari ST disks. The output from the `TRACK READ` command looks like this:

```Track read complete; recorded 37967 flux transitions.
90% pulse interval spread is 0.7 microseconds.
Pulse Distance  Count   Average   Variance
     000004 us  18596   3.981 us  0.00324 us
     000005 us      1   4.988 us  0.00000 us
     000006 us  14598   6.025 us  0.00341 us
     000007 us      1   7.012 us  0.00000 us
     000008 us   4769   8.012 us  0.00459 us
     000030 us      1   29.712 us  0.00000 us
Double Density MFM track detected.
Found Index Access Marker (track start).
IBM Sector ID: Cylinder 0, Side 0, Sector 1, CRC=ca6f (GOOD)
IBM Sector data: 512 bytes with CRC=3cde (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 2, CRC=9f3c (GOOD)
IBM Sector data: 512 bytes with CRC=0f18 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 3, CRC=ac0d (GOOD)
IBM Sector data: 512 bytes with CRC=b1b1 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 4, CRC=359a (GOOD)
IBM Sector data: 512 bytes with CRC=0751 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 5, CRC=06ab (GOOD)
IBM Sector data: 512 bytes with CRC=e9f6 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 6, CRC=53f8 (GOOD)
IBM Sector data: 512 bytes with CRC=bb59 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 7, CRC=60c9 (GOOD)
IBM Sector data: 512 bytes with CRC=320e (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 8, CRC=70f7 (GOOD)
IBM Sector data: 512 bytes with CRC=6da6 (GOOD)
IBM Sector ID: Cylinder 0, Side 0, Sector 9, CRC=43c6 (GOOD)
IBM Sector data: 512 bytes with CRC=ff82 (GOOD)
9 IBM sectors read.
```

The first line tells the total number of pulses received (flux transitions detected) during the track read. Normally, with a 720k disk this will be in the range of 15,000 to 40,000.  The data is encoded on the floppy disk by varying the time between adjacent pulses. For an MFM-encoded double-density disks, nearly all of the pulses should be either 4, 6, or 8 microseconds apart.

The "pulse interval spread" is a statistical metric which tells us how much variation exists between the pulse timings. For a formatted disk with data written along the entire track, this value will be in the 0.5-1.5 microsecond range. For a track which has been completely erased, this value will be much higher, in the 9-10 microsecond range.

Next, a histogram tells us how many pulse intervals fell into each 1-microsecond range. Again with an MFM-formatted double-density disk, we expect to see the vast majority of the pulse intervals at 4, 6, and 8 microseconds and very few in the other time ranges.

After the histogram, any decoded sector data will be printed. If you expect to see track information here, but there's nothing, and the histogram looks good, make sure that you have set the correct encoding format (IBM/Amiga) with the `GEOMETRY` command.

#### DISK READ
Using the `DISK READ` command, you can get an overview of all the sectors on the disk.  The output looks like this:

```Command> DISK READ
Found track 0 after stepping 0.0 tracks.
Reading disk sides 0 and 1.
 0 (37964 0.7) MFM  9 Sectors: OOO OOO OOO OOO OOO OOO OOO OOO OOO  |  (37911 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 1 (38001 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37859 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 2 (37856 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37846 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 3 (37896 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (38010 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 4 (37953 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37851 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 5 (37913 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37917 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 6 (37851 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37865 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 7 (37896 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37998 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 8 (37863 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37900 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
 9 (37843 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37952 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
10 (37921 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37926 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
11 (37910 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37894 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
12 (37892 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37912 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
13 (37958 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37955 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
14 (37951 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37939 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
15 (37899 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37882 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
16 (37934 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO  |  (37857 0.7) MFM  8 Sectors: --- OOO OOO OOO OOO OOO OOO OOO OOO 
17 (18787 10.7) MFM  0 Sectors: --- --- --- --- --- --- --- --- ---  |  (37892 2.8) MFM  0 Sectors: --- --- --- --- --- --- --- --- --- 
18 (34117 3.7) MFM  0 Sectors: --- --- --- --- --- --- --- --- ---  |  (36187 2.9) MFM  0 Sectors: --- --- --- --- --- --- --- --- --- 
```

The left half of this shows side 0, while the right half shows side 1.  At the start of each track, in parentheses, are the values for the number of flux transitions in the track, and the 90% pulse interval spread statistic.

Following this is the encoding type (MFM), the number of sectors read in the track, and 3 letters indicating the status of each sector in the track.  Three dashes or hyphens means that the sector was not detected. Otherwise, each letter is either an `O` (meaning good) or an `X` (meaning bad).

The first letter indicates whether or not the positional metadata (track number and side number) in the sector header matches with the sector's position on the disk. The second letter indicates whether the CRC or checksum value in the sector header matches with the header's actual CRC.  And the third letter indicates whether the sector's data CRC or checksum match with the checksum calculated from the data read from disk.

#### DISK RWTEST

The `DISK RWTEST` command is useful for stress testing the floppy drive and a disk to determine if it is susceptible to any particular patterns in the pulse timings.

With this command, each track is written and then read back four times, with a different data pattern each time. The first pattern uses all 4 microsecond pulse timings for the sector data. The second pattern uses all 6 microsecond pulses, and the third uses all 8 microsecond pulses.  The last data pattern starts with random sector data bytes and writes out the pulses corresponding to these random byte values.

The output of this command will look like this:

```Found track 0 after stepping 79.0 tracks.
Testing disk sides 0 and 1.
 0: OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO | OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO
 1: OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO | OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO OOOO
...
```

The leftmost number here is the track number. Then we have 4 letters for each sector, one letter for each data pattern.  If the letter is an `O`, then the data was successfully read back for that sector with that data pattern. If it is an `X`, then the sector data CRC didn't match, indicating likely data corruption. If it is an `x`, then the sector data was good but the sector header's CRC or metadata values didn't match.

### Caveats

There are many.

* This software and hardware comes with no warranty of any kind.
* Read the [Hardware Warning](#hardware-warning) again.
* This software has been written very quickly and is far from feature-complete.
* The Amiga disk format is currently only supported for reading, not for writing.
* The IBM disk format only supports high sector counts (10 and 11) for reading. Only the 9-sector format is supported for writing.
* I have only tested this with one PC 3.5" drive and one Amiga 3.5" drive. In the future other types of drives will be supported as well.
* Only Double Density disks are currently supported. The ESP32 is fast enough to handle HD disks as well, but it is likely that a significant amount of code refactoring would be necessary to support HD drives due to the limited amout of RAM in the ESP32.

