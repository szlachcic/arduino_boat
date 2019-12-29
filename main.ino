#include "Arduino.h"
#include "gps.h"
#include <Arduino_FreeRTOS.h>


GPS gps;

void TaskGPSLoop( void *pvParameters );
void TaskAnalogRead( void *pvParameters );

void setup()
{
  Serial.begin(9600);

  gps.begin();


  xTaskCreate(
    TaskGPSLoop
    ,  (const portCHAR *)"GPS" 
    ,  1024
    ,  NULL
    ,  1 
    ,  NULL );

    xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  2  // priority
    ,  NULL );

}

void loop()
{
  while(true)
  {

  }
    
}

void TaskGPSLoop(void *pvParameters)
{
  (void) pvParameters;
  while(true)
  {

    gps.receive_next_msg();
    Serial.println(gps.gpgga.latitude,5);
    Serial.println(gps.gpgga.longitude,5);
    Serial.println(gps.gpgga.hdop,5);
    vTaskDelay(1);

  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}




