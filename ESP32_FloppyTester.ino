//////////////////////////////////////////////////////////////////////////
// ESP32_FloppyTester
// 
// Copyright (C) 2023-2024, All Rights Reserved.
//
// Author: Richard Goedeken

#include <stdint.h>
#include <string.h>

#include <map>

#include <driver/timer.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_struct.h>
#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_timer.h"

#include "esp_intr_alloc.h"

#include "DecoderMFM.h"

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

// initialization helpers
void gpio_reset(void);
void gpio_init(void);

// ISR
static void IRAM_ATTR onSignalEdge(void *pParams);

// command functions
void display_help(void);
void test_input(void);
void test_output(void);
void detect(void);
void test_rpm(void);
void test_seek(void);
void seek_track(int iTargetTrack);
void test_read(void);

// generic helpers
void delay_micros(uint32_t uiMicros);
std::string command_input(void);

//////////////////////////////////////////////////////////////////////////
// Local variables

static bool         l_bGPIOInit = false;
static int          l_iDriveSelect = FDC_SEL10;
static int          l_iDriveMotor = FDC_SEL16;
static int          l_iCurrentTrack = -1;

const uint32_t            cuiDeltaBufSize = 65536;
static volatile bool      l_bRecording = false;
static volatile uint32_t  l_uiLastSignalTime = 0;
static uint16_t          *l_pusDeltaBuffers[16];      // 16 * 4096 == 65536
static volatile uint16_t  l_uiDeltaPos = 0;
static volatile uint32_t  l_uiDeltaCnt = 0;

#define DELTA_ITEM(X) l_pusDeltaBuffers[(X)>>12][(X)&0x0fff]

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
    //Serial.printf("Free heap memory: %i bytes\r\n", ESP.getFreeHeap());
    for (int i = 0; i < 16; i++)
    {
        l_pusDeltaBuffers[i] = (uint16_t *) malloc(8192);
        if (l_pusDeltaBuffers[i] == NULL)
        {
           Serial.printf("Error: failed to allocate delta buffer #%i/16.\r\n", i + 1);
           return;
        }
    }
    //Serial.printf("Free heap memory: %i bytes\r\n", ESP.getFreeHeap());
    
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

//////////////////////////////////////////////////////////////////////////
// Main loop - command processor

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
        display_help();
    }
    else if (strInput == "test input")
    {
        test_input();
    }
    else if (strInput == "test output")
    {
        test_output();
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
        detect();
    }
    else if (strInput == "test rpm")
    {
        test_rpm();
    }
    else if (strInput == "test seek")
    {
        test_seek();
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
        seek_track(iTargetTrack);
    }
    else if (strInput == "test read")
    {
        test_read();
    }
    else
    {
        Serial.write("Unknown command.\r\n");
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command functions

void display_help(void)
{
    Serial.write("\r\nESP32_FloppyTester commands:\r\n");
    Serial.write("    HELP         - display this help message.\r\n");
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

void test_input(void)
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

void test_output(void)
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

void detect(void)
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

void test_rpm(void)
{
    // turn on the motor
    gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
    gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
    
    // wait 0.5 seconds for speed to settle
    delay(TIME_MOTOR_SETTLE);

    // reset the sample buffer and enable the index pulse edge capture
    l_bRecording = false;
    l_uiLastSignalTime = 0;
    l_uiDeltaPos = 0;
    l_uiDeltaCnt = 0;
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);
    MCPWM0.int_ena.val = CAP0_INT_EN;
    mcpwm_isr_register(MCPWM_UNIT_0, onSignalEdge, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // print out the calculated speed once per second, until interrupted by user
    bool bStarted = false;
    uint32_t uiSegmentDuration = 0;
    uint32_t uiSegmentPulses = 0;
    uint32_t uiDeltaReadPos = 0;
    uint32_t uiIterations = 0;
    while(Serial.available() == 0)
    {
        // delay to avoid burning the CPU
        delay(100);
        uiIterations++;
        // check for end of buffer condition
        if (l_uiDeltaPos + 4 >= cuiDeltaBufSize)
        {
            break;
        }
        // check for no index pulse
        if (!l_bRecording && uiIterations >= 20)
        {
            Serial.write("Error: no index pulse found in 2 seconds.\r\n");
            break;
        }
        // check for one-second window passed
        while(uiDeltaReadPos < l_uiDeltaPos)
        {
            // get the next time delta from the buffer
            uint32_t uiDelta = DELTA_ITEM(uiDeltaReadPos);
            if (uiDelta & 0x8000)
            {
                uiDeltaReadPos++;
                uiDelta = ((uiDelta & 0x7fff) << 16) + DELTA_ITEM(uiDeltaReadPos);
            }
            uiDeltaReadPos++;
            // add it to our segment duration
            uiSegmentDuration += uiDelta;
            uiSegmentPulses++;
            // print calculated RPM speed if the segment is over one second
            if (uiSegmentDuration >= 80 * 1000 * 1000)
            {
                double dRevTimeTicks = (double) uiSegmentDuration / uiSegmentPulses;
                double dRevTimeSecs = dRevTimeTicks / 80000000.0;
                double dRPM = (1.0 / dRevTimeSecs) * 60.0;
                Serial.printf("Drive speed: %.4lf rpm\r\n", dRPM);
                // advance the window to the next second
                uiSegmentDuration = 0;
                uiSegmentPulses = 0;
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

void test_seek(void)
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

void seek_track(int iTargetTrack)
{
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

void test_read(void)
{
    // turn on the motor
    gpio_set_level((gpio_num_t) l_iDriveSelect, 1);
    gpio_set_level((gpio_num_t) l_iDriveMotor, 1);
    
    // wait 0.5 seconds for speed to settle
    delay(TIME_MOTOR_SETTLE);

    ESP_INTR_DISABLE(XT_TIMER_INTNUM);

    // wait until index pulse arrives
    do {} while (gpio_get_level(FDC_INDEX) == 0);

    // reset the sample buffer and enable the RDATA pulse edge capture
    l_bRecording = false;
    l_uiLastSignalTime = 0;
    l_uiDeltaPos = 0;
    l_uiDeltaCnt = 0;
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0);
    MCPWM0.int_ena.val = CAP1_INT_EN;
    mcpwm_isr_register(MCPWM_UNIT_0, onSignalEdge, NULL, ESP_INTR_FLAG_IRAM, NULL);

    // wait until the index pulse leaves
    do {} while (gpio_get_level(FDC_INDEX) == 1);

    // wait until the next index pulse arrives
    do {} while (gpio_get_level(FDC_INDEX) == 0);

    // disable the signal capture
    mcpwm_capture_disable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);

    ESP_INTR_ENABLE(XT_TIMER_INTNUM);

    // print some statistics
    if (!l_bRecording)
        Serial.printf("Track read complete; recorded no flux transitions.\r\n");
    else
        Serial.printf("Track read complete; recorded %i flux transitions.\r\n", l_uiDeltaCnt + 1);

    // turn off the motor
    gpio_set_level((gpio_num_t) l_iDriveSelect, 0);
    gpio_set_level((gpio_num_t) l_iDriveMotor, 0);

    // calculate histogram
    std::map<uint32_t, uint32_t> mapCountByWavelen;
    std::map<uint32_t, float>    mapSumByWavelen;
    for (uint32_t ui = 0; ui < l_uiDeltaPos; ui++)
    {
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            ui++;
            uiDelta = ((uiDelta & 0x7fff) << 16) + DELTA_ITEM(ui);
        }
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen + 0.5f);
        if (mapCountByWavelen.count(uiWavelen) == 0)
        {
            mapCountByWavelen.insert(std::make_pair(uiWavelen, 1));
            mapSumByWavelen.insert(std::make_pair(uiWavelen, fWavelen));
        }
        else
        {
            mapCountByWavelen[uiWavelen]++;
            mapSumByWavelen[uiWavelen] += fWavelen;
        }
    }
    
    // calculate average of each bin
    std::map<uint32_t, float>    mapAvgByWavelen;
    std::map<uint32_t, float>    mapVarByWavelen;
    for (std::map<uint32_t,uint32_t>::iterator it = mapCountByWavelen.begin(); it != mapCountByWavelen.end(); ++it)
    {
        const uint32_t uiWavelen = it->first;
        mapAvgByWavelen.insert(std::make_pair(uiWavelen, mapSumByWavelen[uiWavelen] / it->second));
        mapVarByWavelen.insert(std::make_pair(uiWavelen, 0.0f));
    }

    // calculate variance of each bin
    for (uint32_t ui = 0; ui < l_uiDeltaPos; ui++)
    {
        uint32_t uiDelta = DELTA_ITEM(ui);
        if (uiDelta & 0x8000)
        {
            ui++;
            uiDelta = ((uiDelta & 0x7fff) << 16) + DELTA_ITEM(ui);
        }
        const float fWavelen = uiDelta / 80.0f;
        const uint32_t uiWavelen = floorf(fWavelen + 0.5f);
        const float fDiff = fWavelen - mapAvgByWavelen[uiWavelen];
        mapVarByWavelen[uiWavelen] += fDiff * fDiff;
    }
    for (std::map<uint32_t,uint32_t>::iterator it = mapCountByWavelen.begin(); it != mapCountByWavelen.end(); ++it)
    {
        const uint32_t uiWavelen = it->first;
        mapVarByWavelen[uiWavelen] /= it->second;
    }

    // print statistics
    Serial.write("Pulse Distance  Count   Average   Variance\r\n");
    for (std::map<uint32_t,uint32_t>::iterator it = mapCountByWavelen.begin(); it != mapCountByWavelen.end(); ++it)
    {
        const uint32_t uiWavelen = it->first;
        const uint32_t uiCount = it->second;
        const float fAverage = mapAvgByWavelen[uiWavelen];
        const float fVariance = mapVarByWavelen[uiWavelen];
        Serial.printf("     %06u us  %5u   %.3f us  %.5f us\r\n", uiWavelen, uiCount, fAverage, fVariance);
    }

    // detect track encoding
    uint32_t auiCountByWavelen[10];
    for (uint32_t ui = 0; ui < 10; ui++)
    {
        if (mapCountByWavelen.count(ui) == 0)
            auiCountByWavelen[ui] = 0;
        else
            auiCountByWavelen[ui] = mapCountByWavelen[ui];
    }
    if (l_uiDeltaCnt > 10)
    {
        if ((float) (auiCountByWavelen[2] + auiCountByWavelen[3] + auiCountByWavelen[4]) / l_uiDeltaCnt > 0.9f)
        {
            Serial.write("High Density MFM track detected.\r\n");
        }
        if ((float) (auiCountByWavelen[4] + auiCountByWavelen[6] + auiCountByWavelen[8]) / l_uiDeltaCnt > 0.9f)
        {
            Serial.write("Double Density MFM track detected.\r\n");
            DecoderMFM decoder((const uint16_t **) l_pusDeltaBuffers, l_uiDeltaPos);
            decoder.DecodeTrack(true);
        }
        else if ((float) (auiCountByWavelen[3] + auiCountByWavelen[5] + auiCountByWavelen[8]) / l_uiDeltaCnt > 0.9f)
        {
            Serial.write("Double Density GCR track detected.\r\n");
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine

static void IRAM_ATTR onSignalEdge(void *pParams)
{
    // read interrupt status
    const uint32_t mcpwm_intr_status = MCPWM0.int_st.val;
    MCPWM0.int_clr.val = mcpwm_intr_status;
    
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
        return;
    }

    // record the difference between this signal time and the last one in our delta buffer
    if (!l_bRecording)
    {
        l_bRecording = true;
        l_uiLastSignalTime = signalTime;
    }
    else
    {
        uint32_t uiDelta = signalTime - l_uiLastSignalTime;
        l_uiLastSignalTime = signalTime;
        if (l_uiDeltaPos + 1 < cuiDeltaBufSize)
        {
            if (uiDelta < 32768)
            {
                DELTA_ITEM(l_uiDeltaPos) = (uint16_t) uiDelta;
                l_uiDeltaPos++;
            }
            else
            {
                DELTA_ITEM(l_uiDeltaPos) = (uint16_t) (uiDelta >> 16) | 0x8000;
                DELTA_ITEM(l_uiDeltaPos + 1) = (uint16_t) (uiDelta & 0xffff);
                l_uiDeltaPos += 2;
            }
            l_uiDeltaCnt++;
        }
    }
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
