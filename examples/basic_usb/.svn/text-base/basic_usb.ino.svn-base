#include <XLR8USB.h>

XLR8USB usb;

void setup() {
  Serial.begin(115200);
  usb.init();
}

void loop() {

  uint8_t xmov;   //Mouse X movement
  uint8_t ymov;   //Mouse Y movement
  uint8_t scrl;   //Mouse Scoll movement

  int8_t  buttons; //Mouse buttons  bit0=left button  bit1=right button  bit2=middle button

  xmov = usb.get_xmov();
  ymov = usb.get_ymov();
  scrl = usb.get_scrl();
  buttons = usb.get_btns();

  Serial.print("Buttons: ");Serial.print(buttons);
  Serial.print("   Xmov: ");Serial.print(xmov);
  Serial.print("   Ymov: ");Serial.print(ymov);
  Serial.print("   Scrl: ");Serial.println(scrl);
  delay(100);

}

