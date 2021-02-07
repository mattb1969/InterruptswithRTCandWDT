# Interrupts with RTC and WDT

A sample repository for the MKRWAN 1310 to undersrtand and configure the various clocks on the SAMD21 processor to enable
- Edge triggering (the sample uses rising)
- A RTC that is configured to interrupt sleep every 5 seconds
    (this does increase to demonstrate the Watchdog)
- A WDT to monitor the software in case of failure

The example demonostrates the software by slowly increasing the RTC time until the RTC > WDT at which point the WDT triggers and the board resets.

The project is written in VS Code, teh attached libraries are there for reference only and are not called