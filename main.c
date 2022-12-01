#include <pdk/device.h>
#include "auto_sysclock.h"
#include "delay.h"

#define PIN_KEY (6)
#define PIN_IN (3)
#define PIN_OUT (4)

#define READ_IN ()

// Main program
void main() {
    while (1) {
        _delay_ms(1000);
    }
}

// Startup code - Setup/calibrate system clock
unsigned char _sdcc_external_startup(void) {
    // Initialize the system clock (CLKMD register) with the IHRC, ILRC, or EOSC clock source and correct divider.
    // The AUTO_INIT_SYSCLOCK() macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC clock source and divider.
    // Alternatively, replace this with the more specific PDK_SET_SYSCLOCK(...) macro from pdk/sysclock.h
    AUTO_INIT_SYSCLOCK();

    // Insert placeholder code to tell EasyPdkProg to calibrate the IHRC or ILRC internal oscillator.
    // The AUTO_CALIBRATE_SYSCLOCK(...) macro uses F_CPU (defined in the Makefile) to choose the IHRC or ILRC oscillator.
    // Alternatively, replace this with the more specific EASY_PDK_CALIBRATE_IHRC(...) or EASY_PDK_CALIBRATE_ILRC(...) macro from easy-pdk/calibrate.h
    AUTO_CALIBRATE_SYSCLOCK(TARGET_VDD_MV);

    return 0; // Return 0 to inform SDCC to continue with normal initialization.
}
