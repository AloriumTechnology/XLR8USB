/*--------------------------------------------------------------------
 Copyright 2018 Alorium Technology. All rights reserved.
 Written by Ted Holler of Alorium Technology (info@aloriumtech.com)

 Permission is hereby granted, free of charge, to any person obtaining 
 a copy of this software and associated documentation files (the 
 "Software"), to deal in the Software without restriction, including 
 without limitation the rights to use, copy, modify, merge, publish, 
 distribute, sublicense, and/or sell copies of the Software, and to 
 permit persons to whom the Software is furnished to do so, subject to 
 the following conditions:

 The above copyright notice and this permission notice shall be 
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--------------------------------------------------------------------*/

#ifndef XLR8USB_H
#define XLR8USB_H

// #ARDUINO_XLR8 is passed from IDE to the compiler if XLR8 is selected properly
#ifdef ARDUINO_XLR8

class XLR8USB {
public:
    uint8_t  get_usb_ctrl(void);  //bit0 reset  bit7 = usb_ready
    uint8_t  get_btns(void); //bit0 Left, bit1 Right,  bit2 Middle
    uint8_t  get_left_btn(void);
    uint8_t  get_right_btn(void);
    uint8_t  get_middle_btn(void);
    int8_t   get_xmov(void);
    int8_t   get_ymov(void);
    int8_t   get_scrl(void);
    void     init(void);
};

#else
#error "XLR8USB library requires Tools->Board->XLR8xxx selection. Install boards from https://github.com/AloriumTechnology/Arduino_Boards"
#endif

#endif
