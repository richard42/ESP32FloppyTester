
#include <driver/timer.h>

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

const gpio_num_t l_aiPinsInput[] = { FDC_INDEX, FDC_TRK00, FDC_WPT, FDC_RDATA, FDC_DISKCHG };
const int l_iNumInputs = sizeof(l_aiPinsInput) / sizeof(gpio_num_t);
const gpio_num_t l_aiPinsOutput[] = { FDC_DENSEL, FDC_SEL06, FDC_SEL10, FDC_SEL12, FDC_SEL14, FDC_SEL16, FDC_DIR, FDC_STEP, FDC_WDATA, FDC_WGATE, FDC_SIDE1 };
const int l_iNumOutputs = sizeof(l_aiPinsOutput) / sizeof(gpio_num_t);

int l_iPinFDDbySignal[40];

//////////////////////////////////////////////////////////////////////////
// Forward declarations

void gpio_reset(void);
void gpio_init(void);

std::string command_input(void);

//////////////////////////////////////////////////////////////////////////
// Local variables

static bool l_bGPIOInit = false;
static int  l_iDriveSelect = FDC_SEL10;
static int  l_iDriveMotor = FDC_SEL16;

//////////////////////////////////////////////////////////////////////////
// Initialization

void setup()
{
    // setup serial port (which internally uses UART 0)
    Serial.begin(115200);
    Serial.write("\r\n\r\n\r\nESP32_FloppyTester v" E32FT_VERSION " ready.\r\n");

    // set up GPIO pins and set outputs to 0
    gpio_init();

    // setup timer 0-0 to run as 80MHz incrementing counter, to use for timing, and start it
    timer_config_t config0 = {
        .alarm_en = TIMER_ALARM_DIS,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_DIS,
        .divider = 1,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_disable_intr(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_0);

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
    else
    {
        Serial.write("Unknown command.\r\n");
    }
}

//////////////////////////////////////////////////////////////////////////
// Helper functions

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
