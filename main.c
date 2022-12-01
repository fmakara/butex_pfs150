#include <stdint.h>
#ifndef PFS154
#define PFS154
#endif
#ifndef F_CPU
#define F_CPU 8000000
#endif
#ifndef TARGET_VDD_MV
#define TARGET_VDD_MV 3300
#endif
#include <pdk/device.h>
#include <auto_sysclock.h>

// ---------------------------- Pin definitions ------------------------------
#define PIN_KEY    (6)
#define PIN_IN     (3)
#define PIN_OUT    (4)

// -------------------------- Timing definitions -----------------------------
#define TIMING_SHORT     (5)
#define TIMING_SHORT_TH  (10)
#define TIMING_LONG      (20)
#define TIMING_MAX       (220)

// -------------------------- General operations -----------------------------
#define READ_KEY    ((PA&(1<<PIN_KEY))?1:0)
#define READ_IN     ((PA&(1<<PIN_IN))?1:0)
#define READ_OUT    ((PA&(1<<PIN_OUT))?1:0)

#define PULLUP_IN   PAPH|=(1<<PIN_IN)
#define PULLUP_OUT  PAPH|=(1<<PIN_OUT)
#define PULLNO_IN   PAPH&=~(1<<PIN_IN)
#define PULLNO_OUT  PAPH&=~(1<<PIN_OUT)

#define INPUT_IN    PAC&=~(1<<PIN_IN)
#define INPUT_OUT   PAC&=~(1<<PIN_OUT)
#define OUTPUT_IN   PAC|=(1<<PIN_IN)
#define OUTPUT_OUT  PAC|=(1<<PIN_OUT)

#define SET_IN      PA|=(1<<PIN_IN)
#define SET_OUT     PA|=(1<<PIN_OUT)
#define UNSET_IN    PA&=~(1<<PIN_IN)
#define UNSET_OUT   PA&=~(1<<PIN_OUT)

#define TIMER        TM2CT
#define CLEAR_TIMER  TM2CT=0
#define SLEEP(us)    CLEAR_TIMER; while(TIMER<us)

// ------------------------------ Main program -------------------------------
void main() {
    uint8_t avoidBtnEvt = 0;
    uint8_t lastKey = 0;
    uint8_t data = 0;
    PDK_DISABLE_ILRC();
    PAC = (uint8_t)~((1<<PIN_KEY)|(1<<PIN_IN)|(1<<PIN_OUT)); // all start as input
    PAPH = (1<<PIN_KEY)|(1<<PIN_IN)|(1<<PIN_OUT); // all start as pullup
    PADIER = (1<<PIN_KEY)|(1<<PIN_IN)|(1<<PIN_OUT); // any can wakeup
    MISC = (1<<MISC_FAST_WAKEUP_ENABLE_BIT);
    TM2C = TM2C_CLK_ILRC;
    TM2S = 8; // 1 count = 1us (for 8mhz sysclk)
    CLEAR_TIMER;
    //T16M = (uint8_t)T16M_CLK_IHRC;
    //T16C = 0; // is this the counter?
    while (1) {
        uint8_t key = READ_KEY; // always sampling once per loop
        // default state: all pins are inputs with pullups
        if(avoidBtnEvt){
            if(TIMER>TIMING_MAX){
                avoidBtnEvt = 0;
            }
        } else if(key != lastKey) {
            lastKey = key;
            data = !key;
            // key is changed, notify backwards and forwards to sample the
            // current state of button
            OUTPUT_IN;
            UNSET_IN;
            OUTPUT_OUT;
            UNSET_OUT;
            PULLNO_IN;
            PULLNO_OUT;
            SLEEP(TIMING_LONG);
            SET_IN;
            SET_OUT;
            PULLUP_IN;
            PULLUP_OUT;
            INPUT_IN;
            INPUT_OUT;
            avoidBtnEvt = 1;
            CLEAR_TIMER;
        }
        if(!READ_OUT) {
            OUTPUT_IN;
            UNSET_IN;
            PULLNO_IN;
            // in any input, always forward at least for TIMING_LONG
            SLEEP(TIMING_LONG);
            while(!READ_OUT); // wait until OUT is back to HIGH
            SET_IN;
            PULLUP_IN;
            INPUT_IN;
            if(!avoidBtnEvt){
                lastKey = key;
                data = !key;
                avoidBtnEvt = 1;
            }
            CLEAR_TIMER;
        }
        if(!READ_IN) {
            OUTPUT_OUT;
            UNSET_OUT;
            PULLNO_OUT;
            CLEAR_TIMER;
            while(!READ_IN);
            // The biggest assumption: the microcontroller will always send a
            // pulse smaller than 255us
            volatile uint8_t dt = TIMER<TIMING_SHORT_TH;
            if(dt) {
                uint8_t read = 0;
                // keep pulling line until minimum time
                while(TIMER<TIMING_SHORT);
                SET_OUT;
                PULLUP_OUT;
                INPUT_OUT;

                OUTPUT_IN;
                if(data) SET_IN; else UNSET_IN;
                PULLNO_IN;
                CLEAR_TIMER;
                while(TIMER<TIMING_SHORT) read |= READ_OUT;
                SET_IN;
                while(TIMER<TIMING_SHORT_TH) read |= READ_OUT;
                PULLUP_IN;
                INPUT_IN;

                data = read;
                avoidBtnEvt = 1;
                CLEAR_TIMER;

            } else {
                // keep pulling line until minimum time
                while(TIMER<TIMING_LONG);
                SET_OUT;
                PULLUP_OUT;
                INPUT_OUT;
                // (re) capture key
                lastKey = key;
                data = !key;
                avoidBtnEvt = 1;
                CLEAR_TIMER;
            }
        }
    }
}

// --------------------------- Clock calibration -----------------------------
unsigned char _sdcc_external_startup(void) {
    // Initialize the system clock (CLKMD register) with the IHRC, ILRC, or
    // EOSC clock source and correct divider.
    // The AUTO_INIT_SYSCLOCK() macro uses F_CPU (defined in the Makefile) to
    // choose the IHRC or ILRC clock source and divider.
    // Alternatively, replace this with the more specific PDK_SET_SYSCLOCK(...)
    // macro from pdk/sysclock.h
    AUTO_INIT_SYSCLOCK();

    // Insert placeholder code to tell EasyPdkProg to calibrate the IHRC or
    // ILRC internal oscillator.
    // The AUTO_CALIBRATE_SYSCLOCK(...) macro uses F_CPU (defined in the
    // Makefile) to choose the IHRC or ILRC oscillator.
    // Alternatively, replace this with the more specific
    // EASY_PDK_CALIBRATE_IHRC(...) or EASY_PDK_CALIBRATE_ILRC(...) macro from
    // easy-pdk/calibrate.h
    AUTO_CALIBRATE_SYSCLOCK(TARGET_VDD_MV);

    return 0; // Return 0 to inform SDCC to continue with normal initialization
}
