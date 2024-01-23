//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// 
// Copyright (C) 2023-2024, All Rights Reserved.
//
// Author: Richard Goedeken

#include <stdint.h>
#include <string.h>

#include <driver/timer.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//////////////////////////////////////////////////////////////////////////
// Definitions

#define E32FT_VERSION "1.0"

#define FDC_DENSEL   GPIO_NUM_32
#define FDC_SEL06    GPIO_NUM_33
#define FDC_INDEX    GPIO_NUM_4
#define FDC_SEL10    GPIO_NUM_25
#define FDC_SEL12    GPIO_NUM_23
#define FDC_SEL14    GPIO_NUM_22
#define FDC_SEL16    GPIO_NUM_21
#define FDC_DIR      GPIO_NUM_26
#define FDC_STEP     GPIO_NUM_27
#define FDC_WDATA    GPIO_NUM_19
#define FDC_WGATE    GPIO_NUM_18
#define FDC_TRK00    GPIO_NUM_16
#define FDC_WPT      GPIO_NUM_34
#define FDC_RDATA    GPIO_NUM_35
#define FDC_SIDE1    GPIO_NUM_17
#define FDC_DISKCHG  GPIO_NUM_13

#define TIME_MOTOR_SETTLE       500

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

const gpio_num_t l_aiPinsInput[] = { FDC_INDEX, FDC_TRK00, FDC_WPT, FDC_RDATA, FDC_DISKCHG };
const int l_iNumInputs = sizeof(l_aiPinsInput) / sizeof(gpio_num_t);
const gpio_num_t l_aiPinsOutput[] = { FDC_DENSEL, FDC_SEL06, FDC_SEL10, FDC_SEL12, FDC_SEL14, FDC_SEL16, FDC_DIR, FDC_STEP, FDC_WDATA, FDC_WGATE, FDC_SIDE1 };
const int l_iNumOutputs = sizeof(l_aiPinsOutput) / sizeof(gpio_num_t);

int l_iPinFDDbySignal[40];

//////////////////////////////////////////////////////////////////////////
// Forward declarations

static void IRAM_ATTR onSignalEdge(void *pParams);

void gpio_reset(void);
void gpio_init(void);

std::string command_input(void);

//////////////////////////////////////////////////////////////////////////
// Local variables

static bool         l_bGPIOInit = false;
static int          l_iDriveSelect = FDC_SEL10;
static int          l_iDriveMotor = FDC_SEL16;
static int          l_iCurrentTrack = -1;

const uint32_t            cuiSampleBufSize = 49152;
static uint32_t          *l_puiSampleBuffers[12];      // 12 * 4096 == 49152
static volatile uint32_t  l_uiSampleCnt = 0;

#define SAMPLE_ITEM(X) l_puiSampleBuffers[(X)>>12][(X)&0x0fff]

//////////////////////////////////////////////////////////////////////////
// Initialization

void setup()
{
    // setup serial port (which internally uses UART 0)
    Serial.begin(115200);
    do {
      vTaskDelay(10);
    } while(!Serial);
    delay(200);
    Serial.write("\r\n\r\n\r\nESP32_FloppyTester v" E32FT_VERSION " ready.\r\n");

    // set up GPIO pins and set outputs to 0
    gpio_init();

    // allocate sample buffer memory
    for (int i = 0; i < 12; i++)
    {
        l_puiSampleBuffers[i] = (uint32_t *) malloc(16384);
        if (l_puiSampleBuffers[i] == NULL)
        {
           Serial.printf("Error: failed to allocate sample buffer #%i/12.\r\n", i + 1);
           return;
        }
    }
    
    // set up table mapping signal names to FDD connector pins (used for testing)
    memset(l_iPinFDDbySignal, 0, sizeof(l_iPinFDDbySignal));
    l_iPinFDDbySignal[FDC_DENSEL] = 2;
    l_iPinFDDbySignal[FDC_SEL06] = 6;
    l_iPinFDDbySignal[FDC_INDEX] = 8;
    l_iPinFDDbySignal[FDC_SEL10] = 10;
    l_iPinFDDbySignal[FDC_SEL12] = 12;
    l_iPinFDDbySignal[FDC_SEL14] = 14;
    l_iPinFDDbySignal[FDC_SEL16] = 16;
    l_iPinFDDbySignal[FDC_DIR] = 18;
    l_iPinFDDbySignal[FDC_STEP] = 20;
    l_iPinFDDbySignal[FDC_WDATA] = 22;
    l_iPinFDDbySignal[FDC_WGATE] = 24;
    l_iPinFDDbySignal[FDC_TRK00] = 26;
    l_iPinFDDbySignal[FDC_WPT] = 28;
    l_iPinFDDbySignal[FDC_RDATA] = 30;
    l_iPinFDDbySignal[FDC_SIDE1] = 32;
    l_iPinFDDbySignal[FDC_DISKCHG] = 34;

    // disable interrupt watchdog
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // Unlock timer config.
    TIMERG1.wdt_feed = 1; // Reset feed count.
    TIMERG1.wdt_config0.en = 0; // Disable timer.
    TIMERG1.wdt_wprotect = 0; // Lock timer config.
}

void loop()
{
    // put your main code here, to run repeatedly:
    Serial.write("Command> ");
    std::string strInput = command_input();

    // strip leading and trailing whitespace
    while (strInput.size() > 0 && isspace(strInput.front()))
    {
        strInput.erase(0, 1);
    }
    while (strInput.size() > 0 && isspace(strInput.back()))
    {
        strInput.pop_back();
    }
    if (strInput.size() == 0)
        return;
    
    // parse command
    if (strInput == "help")
    {
        Serial.write("\r\nESP32_FloppyTester commands:\r\n");
        Serial.write("    HELP         - display this help message.\r\n");
        Serial.write("    RESET        - de-activate floppy drive interface.\r\n");
        Serial.write("    STATUS       - show current hardware status.\r\n");
        Serial.write("    TEST INPUT   - activate input test mode, and print level changes on input lines.\r\n");
        Serial.write("    TEST OUTPUT  - activate output test mode, and drive output pins with binary pin numbers.\r\n");
        Serial.write("    DETECT       - toggle select lines to determine drive wiring/jumper setup.\r\n");
        Serial.write("    MOTOR ON/OFF - activate or de-activate motor 1\r\n");
        Serial.write("    TEST RPM     - spin up motor and calculate spindle speed from index pulses.\r\n");
        Serial.write("    TEST SEEK    - test head seeking and track 0 detection.\r\n");
        Serial.write("    TEST READ    - read current track and print decoded data summary (requires formatted disk).\r\n");
        Serial.write("    TEST ERASE   - erase current track and validate erasure (destroys data on disk).\r\n");
        Serial.write("    SEEK <X>     - seek head to track X.\r\n");
        Serial.write("\r\n");
    }
    else if (strInput == "reset")
    {
        gpio_init();
        return;
    }
    else if (strInput == "status")
    {
        if (l_bGPIOInit)
            Serial.write("Floppy drive interface is activated.\r\n");
        else
            Serial.write("Floppy drive interface is in high-impedance state.\r\n");
        return;
    }
    else if (strInput == "test input")
    {
        // repeatedly poll the input lines and print messages for any changes
        char chLineState[l_iNumInputs];
        memset(chLineState, -1, l_iNumInputs);
        while (Serial.available() == 0)
        {
            for (int i = 0; i < l_iNumInputs; i++)
            {
                const int iESP32PinIdx = l_aiPinsInput[i];
                const int iNewState = gpio_get_level((gpio_num_t) iESP32PinIdx);
                const int iFDDPinIdx = l_iPinFDDbySignal[iESP32PinIdx];
                if (chLineState[i] != iNewState)
                {
                    Serial.printf("FDD pin %i changed state to %s\r\n", iFDDPinIdx, iNewState ? "HIGH" : "LOW");
                    chLineState[i] = iNewState;
                }
            }
            vTaskDelay(1);
        }
    }
    else if (strInput == "test output")
    {
        // toggle the output lines with a 100khz pattern with 8 bits, outputting the
        // bit pattern of the pin number on each pin, until a new character is received
        // via the serial port
        while (Serial.available() == 0)
        {
            uint64_t clockTime = 0;
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &clockTime);
            const int bitIdx = (clockTime / 100) & 7;
            const int bitTime = (clockTime % 100);
            for (int i = 0; i < l_iNumOutputs; i++)
            {
                const int iESP32PinIdx = l_aiPinsOutput[i];
                const int iFDDPinIdx = l_iPinFDDbySignal[iESP32PinIdx];
                const int iBitValue = (((iFDDPinIdx << bitIdx) & 128) && (bitTime < 90)) ? 1 : 0;
                gpio_set_level((gpio_num_t) iESP32PinIdx, iBitValue);
            }
        }
        // set all output lines low
        for (int i = 0; i < l_iNumOutputs; i++)
        {
            gpio_set_level((gpio_num_t) l_aiPinsOutput[i], 0);
        }
        // here is a much faster way to set and clear GPIO pins
        //portDISABLE_INTERRUPTS();
        //REG_WRITE(GPIO_OUT_W1TS_REG, (1 << FDC_SEL10));
        //REG_WRITE(GPIO_OUT_W1TC_REG, (1 << FDC_SEL10));
        //REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (FDC_SEL06-32)));
        //REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (FDC_SEL06-32)));
    }
    else if (strInput == "motor on")
    {
        gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
    }
    else if (strInput == "motor off")
    {
        gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
    }
    else if (strInput == "detect")
    {
        int iDriveSelect = -1;
        int iDriveMotor = -1;
        const gpio_num_t aiSelectPins[5] = { FDC_SEL06, FDC_SEL10, FDC_SEL12, FDC_SEL14, FDC_SEL16 };
        bool bFinished = true;
        for (int i = 0; i < 5; i++)
        {
            const gpio_num_t iThisPin = aiSelectPins[i];
            gpio_set_level(iThisPin, 1);
            Serial.printf("Select pin %i/5 activated. Press key for drive state: (M)otor on, (L)ight on, (N)othing on, (A)bort?", i+1);
            char chKey = 0;
            do
            {
                if (Serial.available() == 0)
                {
                    vTaskDelay(50);      // wait for 50 milliseconds
                    continue;
                }
                chKey = Serial.read() & ~32;
            } while (chKey != 'M' && chKey != 'L' && chKey != 'N' && chKey != 'A');
            Serial.printf("%c\r\n", chKey);
            gpio_set_level(iThisPin, 0);
            if (chKey == 'M')
                iDriveMotor = iThisPin;
            if (chKey == 'L')
                iDriveSelect = iThisPin;
            if (chKey =='A')
                break;
            bFinished = (i == 4);
        }
        if (bFinished)
        {
            if (iDriveSelect == -1)
                Serial.write("Error: no drive select line found.\r\n");
            else if (iDriveMotor == -1)
                Serial.write("Error: no motor on line found.\r\n");
            else
            {
                Serial.write("Detection successful.\r\n");
                l_iDriveSelect = iDriveSelect;
                l_iDriveMotor = iDriveMotor;
            }
        }
    }
    else if (strInput == "test rpm")
    {
        // turn on the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
        
        // wait 0.5 seconds for speed to settle
        delay(TIME_MOTOR_SETTLE);

        // reset the sample buffer and enable the index pulse edge capture
        l_uiSampleCnt = 0;
        mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
        MCPWM0.int_ena.val = CAP0_INT_EN;
        mcpwm_isr_register(MCPWM_UNIT_0, onSignalEdge, NULL, ESP_INTR_FLAG_IRAM, NULL);
 
        // print out the calculated speed once per second, until interrupted by user
        bool bStarted = false;
        uint32_t uiStartSample = 0;
        uint32_t uiStartTime = 0;
        uint32_t uiIterations = 0;
        while(Serial.available() == 0)
        {
            // delay to avoid burning the CPU
            delay(100);
            uiIterations++;
            // check for starting sample
            if (!bStarted && l_uiSampleCnt > 0)
            {
                bStarted = true;
                uiStartSample = 0;
                uiStartTime = l_puiSampleBuffers[0][0];
            }
            // check for no index pulse
            if (!bStarted && uiIterations >= 20)
            {
                Serial.write("Error: no index pulse found in 2 seconds.\r\n");
                break;
            }
            // check for one-second window passed
            if (bStarted)
            {
                const uint32_t uiLastTime = SAMPLE_ITEM(l_uiSampleCnt-1);
                if (uiLastTime - uiStartTime >= 80 * 1000 * 1000)
                {
                    // find the next starting sample, ie the oldest sample which is >= 1 second ahead of current window start
                    uint32_t uiNextStartSample = uiStartSample + 1;
                    while (SAMPLE_ITEM(uiNextStartSample) - uiStartTime < 80 * 1000 * 1000)
                        uiNextStartSample++;
                    // calculate the speed and print it
                    double dRevTimeTicks = (SAMPLE_ITEM(uiNextStartSample-1) - SAMPLE_ITEM(uiStartSample)) / (uiNextStartSample - uiStartSample - 1);
                    double dRevTimeSecs = dRevTimeTicks / 80000000.0;
                    double dRPM = (1.0 / dRevTimeSecs) * 60.0;
                    Serial.printf("Drive speed: %.4lf rpm\r\n", dRPM);
                    // advance the window to the next second
                    uiStartSample = uiNextStartSample;
                    uiStartTime += 80 * 1000 * 1000;
                }
            }
        }

        // disable interrupt
        mcpwm_capture_disable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);

        // turn off the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 0);

        // discard all remaining serial data
        delay(100);
        while (Serial.available() > 0)
        {
            Serial.read();
        }
    }
    else if (strInput == "test seek")
    {
        // turn on the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
        // set direction to seek out (towards track 0)
        gpio_set_level(FDC_DIR, 0);
        
        // wait 0.5 seconds for speed to settle
        delay(TIME_MOTOR_SETTLE);

        // search for track zero
        l_iCurrentTrack = -1;
        int iMoved = 0;
        portDISABLE_INTERRUPTS();
        for (; iMoved < 86; iMoved++)
        {
            if (gpio_get_level(FDC_TRK00))
                break;
            gpio_set_level(FDC_STEP, 1);
            delay_micros(1);
            gpio_set_level(FDC_STEP, 0);
            delay_micros(4999);
        }
        portENABLE_INTERRUPTS();
        if (iMoved == 86)
        {
            Serial.write("Error: head stepped 86 tracks but TRACK0 signal never became active.\r\n");
            gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
            gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
            return;
        }
        Serial.printf("Found track 0 after stepping %.1f tracks.\r\n", (float) iMoved);

        // try seeking back and forth a few times
        vTaskDelay(200);
        for (int iDistance = 0; iDistance < 5; iDistance++)
        {
            const int iTracks = 5 * (1 << iDistance) - 1;
            Serial.printf("Seeking to track %i and back.\r\n", iTracks);
            gpio_set_level(FDC_DIR, 1);
            vTaskDelay(1);
            portDISABLE_INTERRUPTS();
            for (iMoved = 0; iMoved < iTracks; iMoved++)
            {
                gpio_set_level(FDC_STEP, 1);
                delay_micros(1);
                gpio_set_level(FDC_STEP, 0);
                delay_micros(4999);
                if (gpio_get_level(FDC_TRK00))
                    break;
            }
            portENABLE_INTERRUPTS();
            if (iMoved != iTracks)
            {
                Serial.write("Error: head stepped %.1f tracks in, but TRACK0 signal was still active.\r\n", (float) iMoved);
                gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
                gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
                return;
            }
            gpio_set_level(FDC_DIR, 0);
            vTaskDelay(200);
            portDISABLE_INTERRUPTS();
            for (iMoved = 0; iMoved < iTracks; )
            {
                gpio_set_level(FDC_STEP, 1);
                delay_micros(1);
                gpio_set_level(FDC_STEP, 0);
                delay_micros(4999);
                iMoved++;
                if (gpio_get_level(FDC_TRK00))
                    break;
            }
            portENABLE_INTERRUPTS();
            if (iMoved < iTracks)
            {
                Serial.printf("Error: TRACK0 signal was asserted after stepping out %.1f of %i tracks.\r\n", (float) iMoved, iTracks);
                gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
                gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
                return;
            }
            if (!gpio_get_level(FDC_TRK00))
            {
                Serial.printf("Error: TRACK0 signal was not asserted after stepping inward and back out %i tracks.\r\n", iTracks);
                gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
                gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
                return;
            }
            vTaskDelay(200);
        }

        // all good
        Serial.write("All tests successful.\r\n");
        l_iCurrentTrack = 0;
        
        // turn off the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
    }
    else if (strInput.find("seek ") == 0)
    {
        int iTargetTrack;
        if (sscanf(strInput.c_str() + 5, "%d", &iTargetTrack) != 1)
        {
            Serial.printf("Error: track number not found in command '%s'.\r\n", strInput.c_str());
            return;
        }
        if (iTargetTrack < 0 || iTargetTrack > 85)
        {
            Serial.printf("Error: invalid target track '%i' specified.\r\n", iTargetTrack);
            return;
        }
        if (l_iCurrentTrack == -1)
        {
            Serial.write("Error: track position not known. You must use \"TEST SEEK\" first.\r\n");
            return;
        }
        if (l_iCurrentTrack == iTargetTrack)
        {
            Serial.printf("Error: drive is already at track %i.\r\n", iTargetTrack);
            return;
        }

        // turn on the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
        
        // set direction to seek
        int iTracksToMove = 0;
        if (iTargetTrack < l_iCurrentTrack)
        {
            gpio_set_level(FDC_DIR, 0);
            iTracksToMove = l_iCurrentTrack - iTargetTrack;
        }
        else
        {
            gpio_set_level(FDC_DIR, 1);
            iTracksToMove = iTargetTrack - l_iCurrentTrack;
        }
        
        // wait 0.5 seconds for speed to settle
        delay(TIME_MOTOR_SETTLE);

        // move to the target track
        int iMoved = 0;
        for (; iMoved < iTracksToMove; iMoved++)
        {
            gpio_set_level(FDC_STEP, 1);
            delay_micros(1);
            gpio_set_level(FDC_STEP, 0);
            delay_micros(4999);
        }

        // special case if destination is zero
        if (iTargetTrack == 0 && !gpio_get_level(FDC_TRK00))
        {
            Serial.write("Error: moved to track 0 but TRACK0 signal is not active.\r\n");
            l_iCurrentTrack = -1;
        }
        else
        {
            l_iCurrentTrack = iTargetTrack;
        }

        // turn off the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
    }
    else if (strInput == "test read")
    {
        // turn on the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
        
        // wait 0.5 seconds for speed to settle
        delay(TIME_MOTOR_SETTLE);

        // turn off the motor
        gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
        gpio_set_level((gpio_num_t) l_iDriveMotor, 0);
    }
    else
    {
        Serial.write("Unknown command.\r\n");
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine

static void IRAM_ATTR onSignalEdge(void *pParams)
{
    // read interrupt status
    uint32_t mcpwm_intr_status = MCPWM0.int_st.val;
    
    // get the latched 80MHz time
    uint32_t signalTime = 0;
    if (mcpwm_intr_status & CAP0_INT_EN)
    {
        signalTime = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
    }
    else if (mcpwm_intr_status & CAP1_INT_EN)
    {
        signalTime = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);
    }
    else
    {
        MCPWM0.int_clr.val = mcpwm_intr_status;
        return;
    }

    // record this time in our sample buffer
    SAMPLE_ITEM(l_uiSampleCnt) = signalTime;
    if (l_uiSampleCnt + 1 < cuiSampleBufSize)
        l_uiSampleCnt++;

    MCPWM0.int_clr.val = mcpwm_intr_status;
}

//////////////////////////////////////////////////////////////////////////
// Helper functions

void delay_micros(uint32_t uiMicros)
{
    uint32_t uiCycles = uiMicros * 240 - 25;
    uint32_t uiEnd, uiCur;
    asm volatile("  rsr %0, ccount                \n"
                 "  add %0, %2, %0                \n"
                 "Loop0:                          \n"
                 "  rsr %1, ccount                \n"
                 "  blt %1, %0, Loop0             \n"
                 : "=&a"(uiEnd), "=a"(uiCur)
                 : "a"(uiCycles) );
}

void gpio_reset(void)
{
    for (int i = 0; i < l_iNumInputs; i++)
    {
        gpio_reset_pin(l_aiPinsInput[i]);
    }
    for (int i = 0; i < l_iNumOutputs; i++)
    {
        gpio_reset_pin(l_aiPinsOutput[i]);
    }
    
    l_bGPIOInit = false;
}

void gpio_init(void)
{
    // Set pin I/O modes
    for (int i = 0; i < l_iNumInputs; i++)
    {
        gpio_set_pull_mode(l_aiPinsInput[i], GPIO_FLOATING);
        gpio_set_direction(l_aiPinsInput[i], GPIO_MODE_INPUT);
    }
    for (int i = 0; i < l_iNumOutputs; i++)
    {
        gpio_set_pull_mode(l_aiPinsOutput[i], GPIO_FLOATING);
        gpio_set_level(l_aiPinsOutput[i], 0);
        gpio_set_direction(l_aiPinsOutput[i], GPIO_MODE_OUTPUT);
    }

    // we will use the MCPWM capture system to record timestamps of signal edges for the INDEX and RDATA lines
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, FDC_INDEX);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, FDC_RDATA);

    l_bGPIOInit = true;
}

std::string command_input(void)
{
    std::string strInputLine;

    // wait for one complete line of input on serial port
    while (true)
    {
        // wait until we have a new character
        if (Serial.available() == 0)
        {
            vTaskDelay(50);      // wait for 50 milliseconds
            continue;
        }
        char newChar = Serial.read();
        // handle backspace
        if (newChar == '\b')
        {
            if (strInputLine.size() > 0)
            {
                Serial.write("\b \b");
                strInputLine.pop_back();
            }
            continue;
        }
        // handle carriage return and linefeed
        if (newChar == '\r')
            continue;
        if (newChar == '\n')
        {
            Serial.write("\r\n"); // add a linefeed after each carriage return
            break;
        }
        // disregard non-ascii characters
        if (newChar < 32 || newChar > 126)
            continue;
        // echo the character back to the terminal and add it to our prompt line
        Serial.printf("%c", newChar);
        strInputLine += tolower(newChar);
    }

    return strInputLine;
}
