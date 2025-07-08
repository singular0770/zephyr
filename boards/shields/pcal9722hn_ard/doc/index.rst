.. _pcal9722hn_ard:

NXP PCAL9722 Evaluation Board
####################################

Overview
********

The NXP PCAL9722HN-ARD shield is an Arduino
UNO R3 compatible shield based on the NXP PCAL9722 IC.

More information about the shield can be found
at the `NXP PCAL9722HN-ARD product information page`_.

Pins Assignments
================

+-----------------------+---------------------+
| Shield Connector Pin  | Function            |
+=======================+=====================+
| D10                   | SPI - CS            |
+-----------------------+---------------------+
| D11                   | SPI - MOSI          |
+-----------------------+---------------------+
| D12                   | SPI - MISO          |
+-----------------------+---------------------+
| D13                   | SPI - SCK           |
+-----------------------+---------------------+
| D7                    | IRQ                 |
+-----------------------+---------------------+

Programming
***********

Set ``--shield pcal9722hn_ard`` when you invoke ``west build``.
For example:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/button
   :board: nucleo_g431rb
   :shield: pcal9722hn_ard
   :goals: build

References
**********

.. target-notes::

.. _NXP PCAL9722HN-ARD product information page:
   https://www.nxp.com/part/PCAL9722HN-ARD
