# Interrupts with RTC and WDT

A sample repository for the MKRWAN 1310 to undersrtand and configure the various clocks on the SAMD21 processor to enable
- Edge triggering (the sample uses rising)
- A RTC that is configured to interrupt sleep every 5 seconds
    (this does increase to demonstrate the Watchdog)
- A WDT to monitor the software in case of failure

The example demonostrates the software by slowly increasing the RTC time until the RTC > WDT at which point the WDT triggers and the board resets.

The project is written in VS Code, the attached libraries were there for reference only, but have now been removed for clarity

## Clock Configuration

Clock | Clock Conroller | Usage
------|-----------------|------
**OSCULP32K | **GCLK 1 | **WDT
| |  **RTC

###OSCULP32K

This clock is always on, so no additional configuration is required


###GCLK 1

Set the Divisor to be 4. This is not divide by 4, but divide by 2^(4+1) = 2^5 = 32
Enable the GCLK
Enable Divisor Mode

Output from GCLK1 is 32,768 / 32 = 1024Hz

###RTC Setup

In Clock Control
Set source to be GCLK 1
Enable the GCLK1

In Mode0 setup
Set to mode 0, 32 bit count
Set divisor to 1024 - so resulting clock speed is 1Hz or 1 per second
Disable Clear on Match

###WDT Setup

In Clock Control
Set source to be GCLK 1
Enable the GCLK1

In WDT settings
Disable Window mode
Enable Early Warning interrupt
Set Early Warning Interrupt Time Offset 0xa - 8,192 cycles - 8s with this config
Set Period before hard WDT overflow <0x8 - 0xb> - 0xb - longer than the early warning interrupt
