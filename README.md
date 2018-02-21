#XLR8USB
## A FPGA implementation for a USB (full_speed) HOST used to connected to a USB Mouse
##

Currently works with these Logitech Mouse models, M325, M325u, M187.
Other mice may work.  

A sketch showing the use of the USB commands is included in the examples directory

To use the XLR8USB XLR8 block follow the directions that are explained in the following webpage.
https://github.com/AloriumTechnology/XLR8USB

A few components will need to be assembled and attached to the XLR8 board.

USB Connector                                        XLR8 Pin
  5v --------------------------------------------------- 5v

  D+ ----------------------------------+---------------- A4
                                       |
  D- --------------------+------------ | --------------- A5
                         |             |
                         >             >
                 15kohm  >             > 15kohm
                         >             >
                         |             |
  GND -------------------+-------------+---------------- GND

