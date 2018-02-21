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

#include <Arduino.h>

#include "XLR8USB.h"

#define USB_CTRL  _SFR_MEM8(0xE0)
#define USB_BTNS  _SFR_MEM8(0xE1)
#define USB_XMOV  _SFR_MEM8(0xE2)
#define USB_YMOV  _SFR_MEM8(0xE3)
#define USB_SCRL  _SFR_MEM8(0xE4)

uint8_t XLR8USB::get_usb_ctrl(void) {
  return USB_CTRL;
}

uint8_t XLR8USB::get_btns(void) {
  return USB_BTNS;  //2^0 = Left,   2^1 = Right,  2^2 = Middle
}

uint8_t XLR8USB::get_left_btn(void) {
  uint8_t btn;
  btn = ((USB_BTNS & 1)==1) ? 1 : 0;
  return btn;
}

uint8_t XLR8USB::get_right_btn(void) {
  uint8_t btn;
  btn = ((USB_BTNS & 2)==2) ? 1 : 0;
  return btn;
}
  
uint8_t XLR8USB::get_middle_btn(void) {
  uint8_t btn;
  btn = ((USB_BTNS & 4)==4) ? 1 : 0;
  return btn;
}
  
int8_t XLR8USB::get_xmov(void) {
  return USB_XMOV;
}

int8_t XLR8USB::get_ymov(void) {
  return USB_YMOV;
}
 
int8_t XLR8USB::get_scrl(void) {
  return USB_SCRL;
}

void XLR8USB::init(void)
{
  int usb_rdy;
  int timeout=0;
  do {
    USB_CTRL =1;
    delay(1);
    USB_CTRL = 0;
    delay(100);
    usb_rdy = get_usb_ctrl();
    timeout++;
    delay(10);
  } while(usb_rdy==0 && timeout<20);

  // **** couldn't connect to the USB ***** 
  // Flash the LED 
  if (timeout==20) {
    pinMode(13, OUTPUT);
    while(1){
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
  }
}

