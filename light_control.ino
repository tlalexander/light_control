//#include <avr/io.h>
#include <stdio.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <cc2500.h>

#define GDO0_PIN  9

#define LED      5
#define LED_GND  4
#define LED_EXT  6


#define RX_MODE        1
#define TX_MODE        2
#define MODE           TX_MODE


#define TX_SLIDER      1
#define TX_BLUETOOTH   2
#define TX_INPUT       TX_BLUETOOTH

#define DEBUG



CC2500 * xmtr = NULL;
SoftwareSerial softSerial(2, 3);

#if MODE == TX_MODE
void loop()
{
  static byte value = 0;

#if TX_INPUT == TX_SLIDER
  // Slider control
  value = (byte)(analogRead(A0) >> 2);
#endif

#if TX_INPUT == TX_BLUETOOTH
  // Bluetooth control
  int data = softSerial.read();
  if (data > 0)
  {
    value = (byte)data;
  }
#endif
  
  analogWrite(LED, value);
  xmtr->write(value);
  
#ifdef DEBUG
  char msg[50] = "TX Write ";
  char buf[10];
  strcat(msg, itoa(value, buf, 10));
  Serial.println(msg);
#endif

  delay(25);
}
#endif

#if MODE == RX_MODE
void loop()
{
  static double lastvalue = 0.0;
  
  byte data[4] = {0xff, 0, 0, 0};
  int bytes = xmtr->read(data, sizeof(data));
  if (bytes > 0)
  {
    
#ifdef DEBUG
    char msg[50] = "RX Read ";
    char buf[10];
    strcat(msg, itoa(bytes, buf, 10));
    strcat(msg, ", Data ");
    strcat(msg, itoa(data[0], buf, 16));
    strcat(msg, ", - ");
    strcat(msg, itoa(data[1], buf, 16));
    strcat(msg, ", ");
    strcat(msg, itoa(data[2], buf, 16));
    strcat(msg, ", ");
    strcat(msg, itoa(data[3], buf, 16));
    Serial.println(msg);
#endif
    
    // Low pass the value a bit
    double currentvalue = (double)data[0] / 255.0;
    double diff = currentvalue - lastvalue;
    if (diff > 0)
    {
      diff /= 7.0;
    }
    else
    {
      diff /= 0.7;
    }
    
    currentvalue = lastvalue + diff;
    if (currentvalue > 1.0) currentvalue = 1.0;
    if (currentvalue < 0.0) currentvalue = 0.0;
    lastvalue = currentvalue;
    
    analogWrite(LED_EXT, (byte)(currentvalue * 255.0));
    analogWrite(LED, data[0]);
  }

  delay(20);
}
#endif

void setup()
{
  
  pinMode(LED_GND, OUTPUT);
  pinMode(LED, OUTPUT);
  
  digitalWrite(LED, HIGH);
  digitalWrite(LED_GND, LOW);
  
#if MODE == RX_MODE

  // RX mode, we need to configure the LED's
  pinMode(LED_EXT, OUTPUT);
  analogWrite(LED_EXT, 0x00);
  
#endif

#if MODE == TX_MODE && TX_INPUT == TX_BLUETOOTH
  softSerial.begin(19200);
  
#endif


  Serial.begin(9600);

  xmtr = new CC2500(GDO0_PIN);
  xmtr->initialize();
}
