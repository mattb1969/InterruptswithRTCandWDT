/* This project is testing the functionality of interrupts when in different operating modes
* It has taken inspiration from my weather station project 

This is version 1
This has a simple setup of interrupts to be triggered on edge when in deep sleep
This is version 2
It has the added timer counter to transmit evry 30 seconds

*/

#include <MKRWAN.h>

static const uint8_t        INTERRUPT_SENSOR_PIN = 4;
static const uint16_t       SEND_FREQUENCY = 30;         // Send freq in Seconds

static const uint8_t        TESTMODE_SET = 1;

volatile bool               rtc_triggered = false;

volatile bool               TEST_MODE = false;
volatile bool               DEBUG = true;
volatile bool               WATCHDOG_ENABLED = false;
volatile bool               RTC_ENABLED = false;

volatile int                WDTCounter;
volatile int                RTCDuration = 5;

//------------------------------------------------------------------------------
// Generic modules

void blink_led(int ontime=100, int offtime=50, int flashes=1) {
    while (flashes >0) {
        digitalWrite(LED_BUILTIN, HIGH);
        delayMicroseconds(ontime*1000);
        digitalWrite(LED_BUILTIN, LOW);
        if (flashes > 1) delayMicroseconds(offtime*1000);           // If it is not the last flash, wait until returning to the top of the loop
        flashes --;
    }
    return;
}

// ISR Routine for the pin being activated
void interruptCalled() {
    digitalWrite(LED_BUILTIN, HIGH);
    if (DEBUG) Serial.println("Interrupt");

    delayMicroseconds(250*1000);

    digitalWrite(LED_BUILTIN, LOW);
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * RTC Routines
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// The RTC Handler is called by the overriding arduino core
void RTC_Handler(void) {
    //need to set a flag that the interrupt has happened
    rtc_triggered = true;

    // IN the RTCCounter, the next period for the RTC is set 

    RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0;              // Clear the interrupt flag

    return;
}

void EICRisingEdgeConfig() {
    GCLK->CLKCTRL.reg =     GCLK_CLKCTRL_CLKEN |                    //Enable Generic Clock
                            GCLK_CLKCTRL_GEN_GCLK1 |                // Select GCLK1 as XOSC32K
                            GCLK_CLKCTRL_ID_EIC;                    // Route GCLK1 to the EIC
    
    while (GCLK->STATUS.bit.SYNCBUSY); //Wait for the settings to be synchronized
    
    // Not sure if this bit is right as the code example was FILTEN6 . . .
   // EIC->CONFIG[0].reg |=   EIC_CONFIG_FILTEN6 |                    // 
   //                         EIC_CONFIG_SENSE6_RISE;

    while (EIC->STATUS.bit.SYNCBUSY); //Wait for the settings to be synchronized

    EIC->CTRL.bit.ENABLE = 1;                                                // Enable the EIC peripheral
                                                                            // This is already set in the WinInterrupts.c in the core
    while (EIC->STATUS.bit.SYNCBUSY); 
    
}

void rtcDisable() {
    // This sets the CTRL.ENABLE = 0
    RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;              // disable RTC, by setting the register to the inverse of the enable bit
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete
}

void rtcEnable() {
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;               // enable RTC by setting the register
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete
}

void rtcSetup() {


    // All taken from RTC Begin...
    
    // This line is in the config file
    // PM->APBAMASK.reg |= PM_APBAMASK_RTC; // turn on digital interface clock

    //This is from config32kOSC
    //RTC is using XOSC32K
    SYSCTRL->XOSC32K.reg =  SYSCTRL_XOSC32K_ONDEMAND |          //S et it to run on demand
                            SYSCTRL_XOSC32K_RUNSTDBY |          // Set it to run in standby mode
                            SYSCTRL_XOSC32K_EN32K |             // Enable the Oscillator
                            SYSCTRL_XOSC32K_XTALEN |            // enablesd external crystal as a oscillator
                            SYSCTRL_XOSC32K_STARTUP(6) |        // Time allowed for startup stablisation
                            SYSCTRL_XOSC32K_ENABLE;             // 

    // The above code configures the clock, setting it to run in Idle or Standby sleep modes
    // Setting run standby = 1, on demand = 1 and enable = 1

    // This is from configureClock

    // The bit below contains one subtle change to the RTC counter example.
    // Where the sample code uses GCLK2, I've changed it to GCLK1 in the places marked ***

    // Next configure the clock to the Generic Clock Generator, plan to use GEN CLOCK 1
    GCLK->GENDIV.reg =      GCLK_GENDIV_ID(1) |                 // Select Generic Clock Controller 1        ***
                            GCLK_GENDIV_DIV(4);                 // Set division factor to be 4
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  
    GCLK->GENCTRL.reg =     GCLK_GENCTRL_ID(1) |                // Select Generic Clock Controller 1        ***
                            GCLK_GENCTRL_GENEN |                // Enable the Generic Clock
                            GCLK_GENCTRL_SRC_XOSC32K |          // Set the source to be XOSC32K
                            GCLK_GENCTRL_DIVSEL;                // Enabler Divide Selection
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  
    GCLK->CLKCTRL.reg = (uint32_t)
                            GCLK_CLKCTRL_CLKEN |                // Enable the generic clock by setting the Clock Enabler bit
                            GCLK_CLKCTRL_GEN_GCLK1 |            // Select Generic Clock 1
                            (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos);     // Identify it as the RTC Clock - shifting it to the right position
//ToDo1:Try this instead of the line above                            //GCLK_CLKCTRL_ID_RTC
    while (GCLK->STATUS.bit.SYNCBUSY);                          // Wait for synchronisation to complete


    // Ensure the RTC is disabled before changing the values
    rtcDisable();

    // RTC Reset
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete

    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;                // software reset
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete

    RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;               // disable continuously mode
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete
    
    uint16_t tmp_reg = 0;
    tmp_reg |= RTC_MODE0_CTRL_MODE_COUNT32;                     // set clock operating mode
    tmp_reg |= RTC_MODE0_CTRL_PRESCALER_DIV1024;                // set prescaler to 1024 for MODE0
    tmp_reg &= ~RTC_MODE0_CTRL_MATCHCLR;                        // disable clear on match
    
    RTC->MODE0.CTRL.reg = tmp_reg;

    NVIC_EnableIRQ(RTC_IRQn);                                   // enable RTC interrupt 
    NVIC_SetPriority(RTC_IRQn, 0x03);

    //Enable the RTC                                I weonder if this should be later as I have not yet set the counter
    //rtcEnable();

}

void rtcSetDuration(uint8_t duration) {
    // Duration is in seconds, whole values only allowed
    uint32_t    mode0Counter;

    //gety2kepoch

    RTC->MODE0.READREQ.reg = RTC_READREQ_RREQ;                  // request a synchronisation of the address register (fixed to 0x10)
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete

    mode0Counter = RTC->MODE0.COUNT.reg;                        // Read the current value from the mode0 counter register
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete

    // calculate the new value for the compare value
    uint32_t nextAlarm = mode0Counter + (uint32_t) duration;


    // setAlarmY2kEpoch
    RTC->MODE0.COMP[0].reg = nextAlarm;                         // Compare value for the counter - triggers interrrupt flag on next clock cycle
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);                     // Wait for synchronisation to complete

    RTC->MODE0.INTENSET.bit.CMP0 = 1;                           // Set interrupt compare register to enable the overflow interrupt

    return;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * Watchdog Routines
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// The WDT Handler is called by the overriding arduino core
void WDT_Handler(void) {
    // This is called when teh watchdog is triggered, either early warning or timeout
    WDTCounter--;                               // EWT down counter, makes multi cycle WDT possible
    if (WDTCounter<=0) {                        // Software EWT counter run out of time : Reset
        // The following line is used in the class where WDT)SHutdown is defined as a function to call on soft shutdown
        //if (WDT_Shutdown != NULL) WDT_Shutdown();   // run extra Shutdown functions if defined
        blink_led(50,50,5);
        WDT->CLEAR.reg = 0xFF;                      // value different than WDT_CLEAR_CLEAR_KEY causes reset
        while(true);
    }
    else {
        WDT->INTFLAG.bit.EW = 1;                    // Clear Interrupt Eearl Warning Flag
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;       // Reset the timeout for the watchdog
        while(WDT->STATUS.bit.SYNCBUSY); 
    }


}

void wdtSetup() {
    /* * * * * * Setup values
     * Watchdog Timeout Period (PER) - A value that determines the watchdog timeout trigger, 
     *      It is a hard trigger, so no calling of routines, just a reboot of the processor
     *      It is the quantity of cycles, not a number - e.g. 0x03 = 64 clock cycle, 0x09 = 4096 clock cycles
     *      For this example, it is not used as the Early Warning system is used instead, but the value MUST be higher, 
     *      so timeoutPeriod is set to 0x0b
     * Early Waring Offset (EW) - A value that determines the early warning trigger
     *      It is a quantity of cycles, not a number - e.g. value 0x02 = 32 clock cycles, 0x08 = 2048 clock cycles
     *      For this example ewOffset is set to 0x0A = 8,196 clock cycles 
     * Generic Clock Division Factor - A value that divides the clock BEFORE it reaches the watchdog
     *      Set in the Generic Clock Controller and is linked to the chosen clock.
     *      For this example, gclkDivisor is set to 4, so the 32,768Hz clock is output at 1,024Hz (4 equates to 2^4+1 = 32)
     * The WDTCounter is an external variable that allows the WDT_HAndler to be called multiple times and therefore allow
     * longer periods of time. In this example it is set to zero as only 1 soft cycle is required
     */

    uint8_t ewOffset = 0x0a;
    uint8_t timeoutPeriod = 0x0b;
    uint8_t gclkDivisor = 4;

    WDTCounter = 0;

    // Configure the Generic Clokc Generator
    // Generic clock generator 2, divisor = 32 (2^(DIV+1))  = _x
    GCLK->GENDIV.reg =  GCLK_GENDIV_ID(2) |                 // Select Generic Clock Controller 2
                        GCLK_GENDIV_DIV(gclkDivisor);                // Set division factor to be _x

    // Enable clock generator 2 using low-power 32.768kHz oscillator.
    // With /32 divisor above, this yields 1024Hz clock.
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |                // Select Generic Clock Controller 2
                        GCLK_GENCTRL_GENEN |                // Enable the Generic Clock                 
                        GCLK_GENCTRL_SRC_OSCULP32K |        // Set the source to be OSCULP32K
                        GCLK_GENCTRL_DIVSEL;                // Enabler Divide Selection
    while(GCLK->STATUS.bit.SYNCBUSY);                       // Wait for synchronisation to complete

    // WDT clock = clock gen 2
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |               // Identify it as the WDT Clock
                        GCLK_CLKCTRL_CLKEN |                // Enable the generic clock by setting the Clock Enabler bit
                        GCLK_CLKCTRL_GEN_GCLK2;             // Select Generic Clock 2
    while(GCLK->STATUS.bit.SYNCBUSY);                       // Wait for synchronisation to complete


    // Enable WDT early-warning interrupt
    NVIC_DisableIRQ(WDT_IRQn);
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_SetPriority(WDT_IRQn, 0);                          // Set it to Top priority
    NVIC_EnableIRQ(WDT_IRQn);

    // Configure the WDT settings for timneout control
    WDT->CTRL.bit.WEN        = 0;                           // Disable window mode
    WDT->INTENSET.bit.EW     = 1;                           // Enable early warning interrupt - enable the multi cycle mechanism via WDT_Handler()
    WDT->EWCTRL.bit.EWOFFSET = ewOffset;                    // Early Warning Interrupt Time Offset 0x6 - 512 clockcycles = 0.5 seconds => trigger ISR
    WDT->CONFIG.bit.PER      = timeoutPeriod;               // Set period before hard WDT overflow <0x8 - 0xb>
    while(WDT->STATUS.bit.SYNCBUSY);                        // Wait for synchronisation to complete

    return;
}

void wdtEnable() {
    // Turn on the watchdog
    WDT->CTRL.bit.ENABLE     = 1;
    while(WDT->STATUS.bit.SYNCBUSY);                        // Wait for synchronisation to complete
    return;
}

void wdtDisable() {
    // Turns off the watchdog
    WDT->CTRL.bit.ENABLE     = 1;
    while(WDT->STATUS.bit.SYNCBUSY);                        // Wait for synchronisation to complete
    return;
}

void wdtReset() {
    // Reset the timeout and the early warning Watchdog counter
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;                   // Clear WTD bit
    while(WDT->STATUS.bit.SYNCBUSY);                        // Wait for synchronisation to complete
    WDTCounter = 0;                                         // Reset the early warning down counter value
    return;
}

void setup () {
    
    //Set all pins to input and 
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TESTMODE_SET, INPUT_PULLDOWN);
    pinMode(INTERRUPT_SENSOR_PIN, INPUT_PULLDOWN);
    
    USBDevice.detach();
    blink_led(50, 450, 5);

    //EICRisingEdgeConfig();            // Not required as script works without it.

    rtcSetup();

    wdtSetup();

    // Setup interrupts
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_SENSOR_PIN), interruptCalled, RISING);

    SYSCTRL->VREG.bit.RUNSTDBY = 1; // Keep the voltage regulator in normal configuration during run standby

    SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1;     // Enable the DFLL48M clock in standby mode as this is used by the EIC

    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED; // Disable auto power reduction during sleep - SAMD21 Errata 1.14.2

    rtcSetDuration(RTCDuration);

    // Enable the Watchdog
    wdtEnable();

    //Enable the RTC
    rtcEnable();


}

void loop() {


    // Go to sleep and wait for an interrupt to trigger
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();

    if (rtc_triggered) {
        // RTC has timed out

        // reset the bool vsrisbler to false for next time
        rtc_triggered = false;

        //Flash the LED twice when it comes out of sleep before it goes back to sleep
        blink_led(100, 200, 2);

        rtcSetDuration(RTCDuration);
        RTCDuration ++;                     //Gradually increase the duration
        
        rtcEnable();
        wdtReset();
    }

}


void base_setup() {
    //Set all pins to input and 
    
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TESTMODE_SET, INPUT_PULLDOWN);
    pinMode(INTERRUPT_SENSOR_PIN, INPUT_PULLDOWN);
    
    USBDevice.detach();
    blink_led(50, 450, 5);

}

void base_loop() {

    // Test for power

    digitalWrite(LED_BUILTIN, HIGH);
    if (DEBUG) Serial.println("Interrupt");

    delayMicroseconds(250*1000);

    digitalWrite(LED_BUILTIN, LOW);

    delayMicroseconds(2500*1000);

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();

}

/*

To Try
Use only 1 clock
Use only OSCULP32K

Use a single Generic Clock Controller

Set all I/O to input with pullup



*/
