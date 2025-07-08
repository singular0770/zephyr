.. zephyr:code-sample:: ktd202x
   :name: KTD202X LED
   :relevant-api: led_interface

   Control 3 LEDs connected to a KTD2026 driver chip.

Overview
********

This sample controls 3 LEDs connected to a KTD2026 driver, using the
following pattern:

 1. turn on LEDs
 2. turn off LEDs
 3. set the brightness to 50%
 4. turn off LEDs
 5. blink the LEDs
 6. turn off LEDs

Building and Running
********************

Build the application for the :zephyr:board:`nucleo_g431rb` board, and connect
a KTD2026 LED driver on the bus I2C Arduino.

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/led/ktd2026
   :board: nucleo_g431rb_board
   :goals: build
   :compact:

For flashing the application, refer to the Flashing section of the
:zephyr:board:`nucleo_g431rb` board documentation.

References
**********

- KTD202X: https://www.kinet-ic.com/uploads/web/KTD2026/KTD2026-7-04h.pdf
