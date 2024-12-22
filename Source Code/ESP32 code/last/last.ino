#define BLYNK_TEMPLATE_ID "TMPLVelNNCpx"
-#define BLYNK_DEVICE_NAME "ESP32 Fire and smoke sensor"
#define BLYNK_AUTH_TOKEN "aVCWLgGRxYjuDi5f0v9Z2jmmJ1rN2cot"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] ="Hiuhiu";
char pass[] = "1111222233";

HardwareSerial SerialPort(2);
int num1,num2;

BlynkTimer timer;

void myTimerEvent()
{*/
  Blynk.virtualWrite(V0,num1);
  Blynk.virtualWrite(V1,num2);
   if(num1 > 90 || num2 == 0){
    Blynk.logEvent("flame_notification","HAVE A FIRE !!! PLEASE BE CAREFUL!!!");
  }  
}

void setup()
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("UART Start");
  Blynk.begin(auth,ssid,pass);
  timer.setInterval(500L, myTimerEvent);
}
void loop(){
  Blynk.run();
  timer.run();
  String st = "";
  char*a;
  char*p;
  while (SerialPort.available())
  { 
    
    char inChar = (char)SerialPort.read();
    st +=  inChar;
    if (inChar == 'D')
    {
        int n= st.length();
        st[n-1]='\0';
        
    Serial.println("Du lieu Nhan: " + st);     
    Serial.print("Du lieu tach:");   
    
              p = strtok((char*)&st,",");
              a = strtok(NULL, ",");

              num1=map(atoi(p),0,700,0,100);
              num2=atoi(a);
              
    Serial.print(num1);
    Serial.print(" and ");  
    Serial.println(num2);  
    break;
        }
    }
  }
