#include "Arduino.h"
#include "gps.h"
#include <Arduino_FreeRTOS.h>
#include <HMC5883L.h>
#include "motor.h"

GPS gps;
HMC5883L compass;
MOTOR motor;

float headingDegrees = 0;

volatile int pulse1_micros = 0;    
volatile int pulse2_micros = 0;         
volatile int pulse3_micros = 0; 
volatile int pulse4_micros = 0;

volatile int pulse1_start = 0; 
volatile int pulse2_start = 0; 
volatile int pulse3_start = 0;  
volatile int pulse4_start = 0;

volatile int chanel1 = 0;
volatile int chanel2 = 0;
volatile int chanel3 = 0;
volatile int chanel4 = 0;

void TaskGPSLoop( void *pvParameters );
void TaskMAGLoop( void *pvParameters );
void TaskSIGLoop( void *pvParameters );

void pulse1_rising() 
{
  attachInterrupt(5, pulse1_falling, FALLING);
  pulse1_start = micros();
}

void pulse1_falling() 
{
  attachInterrupt(5, pulse1_rising, RISING);
  pulse1_micros = micros()-pulse1_start;
}

void pulse2_rising() {
  attachInterrupt(4, pulse2_falling, FALLING);
  pulse2_start = micros();
}

void pulse2_falling() {
  attachInterrupt(4, pulse2_rising, RISING);
  pulse2_micros = micros()-pulse2_start; 
}

void pulse3_rising() {
  attachInterrupt(1, pulse3_falling, FALLING);
  pulse3_start = micros();
}

void pulse3_falling() {
  attachInterrupt(1, pulse3_rising, RISING);
  pulse3_micros = micros()-pulse3_start; 
}

void pulse4_rising() {
  attachInterrupt(0, pulse4_falling, FALLING);
  pulse4_start = micros();
}

void pulse4_falling() {
  attachInterrupt(0, pulse4_rising, RISING);
  pulse4_micros = micros()-pulse4_start; 
}

void setup()
{
  Serial.begin(9600);

  motor.begin();
  gps.begin();
 
  magSetup();

  delay(5000);

  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  attachInterrupt(5, pulse1_rising, RISING); 
  attachInterrupt(4, pulse2_rising, RISING); 
  attachInterrupt(1, pulse3_rising, RISING); 
  attachInterrupt(0, pulse4_rising, RISING); 

  xTaskCreate(
    TaskGPSLoop
    ,  (const portCHAR *)"GPS" 
    ,  1024
    ,  NULL
    ,  1 
    ,  NULL );

  xTaskCreate(
    TaskMAGLoop
    ,  (const portCHAR *) "MAG"
    ,  1024
    ,  NULL
    ,  1  
    ,  NULL );
  xTaskCreate(
    TaskSIGLoop
    ,  (const portCHAR *) "SIG"
    ,  1024
    ,  NULL
    ,  1  
    ,  NULL );

  checkSystem();
}

void loop()
{
 
  // while(true)
  // {
  //   Serial.println(" Degress = ");
  //   Serial.println(headingDegrees);
  //   Serial.println(" Possition = ");
  //   if (gps.is_new_data==true)
  //   {
  //     Serial.println(gps.gpgga.latitude,5);
  //     Serial.println(gps.gpgga.longitude,5);
  //     Serial.println(gps.gpgga.hdop,5);
  //     gps.is_new_data = false;
  //   }
  //   else Serial.println(" Unknown = ");
    
  //   Serial.println(" Signal = ");
  //   Serial.println(chanel1);
  //   Serial.println(chanel2);
  //   Serial.println(chanel3);
  //   Serial.println(chanel4);
  //   delay(1000);
  // }
  
    
}

void TaskGPSLoop(void *pvParameters)
{
  (void) pvParameters;
  while(true)
  {
    gps.receive_next_msg();
    gps.is_new_data = true;

    vTaskDelay(10);
  }
}

void TaskMAGLoop(void *pvParameters)
{
  (void) pvParameters;

  while(true)
  {
    Vector norm = compass.readNormalize();
    float heading = atan2(norm.YAxis, norm.XAxis);
    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
    heading += declinationAngle;
 
    if (heading < 0)
    {
      heading += 2 * PI;
    }
    if (heading > 2 * PI)
    {
      heading -= 2 * PI;
    }
    headingDegrees = heading * 180/M_PI; 

    vTaskDelay(10);
  }
}

void TaskSIGLoop(void *pvParameters)
{
  (void) pvParameters;
  while(true)
  {
    chanel1 = map(pulse1_micros, 1040, 1860, -10, 10);
    chanel2 = map(pulse2_micros, 1040, 1860, -10, 10);
    chanel3 = map(pulse3_micros, 1040, 1860, -10, 10);
    chanel4 = map(pulse4_micros, 1040, 1860, -10, 10);

    vTaskDelay(10);
  }
}

void magSetup()
{
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_4);
  delay(1000);
}

bool checkSystem()
{
  checkMAG();
  delay(1000);
  checkGPS();
  
}

void checkGPS()
{
  Serial.println("GPS checkout: ");

  while (gps.is_new_data=false)
  {
    Serial.println("Waiting for GPS signal");
    delay(5000);
  }
  Serial.println("GPS READY");
}
void checkMAG()
{
  Serial.println("Selected range: ");
  
  switch (compass.getRange())
  {
    case HMC5883L_RANGE_0_88GA: Serial.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  Serial.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  Serial.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  Serial.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    Serial.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  Serial.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  Serial.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  Serial.println("8.1 Ga"); break;
    default: Serial.println("Bad range!");
  }
  
  Serial.print("Selected Measurement Mode: ");
  switch (compass.getMeasurementMode())
  {  
    case HMC5883L_IDLE: Serial.println("Idle mode"); break;
    case HMC5883L_SINGLE:  Serial.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  Serial.println("Continuous-Measurement"); break;
    default: Serial.println("Bad mode!");
  }

  Serial.print("Selected Data Rate: ");
  switch (compass.getDataRate())
  {  
    case HMC5883L_DATARATE_0_75_HZ: Serial.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  Serial.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  Serial.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: Serial.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  Serial.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: Serial.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  Serial.println("75 Hz"); break;
    default: Serial.println("Bad data rate!");
  }
  
  Serial.print("Selected number of samples: ");
  switch (compass.getSamples())
  {  
    case HMC5883L_SAMPLES_1: Serial.println("1"); break;
    case HMC5883L_SAMPLES_2: Serial.println("2"); break;
    case HMC5883L_SAMPLES_4: Serial.println("4"); break;
    case HMC5883L_SAMPLES_8: Serial.println("8"); break;
    default: Serial.println("Bad number of samples!");
  }
  Serial.println("MAGNETOMETR READY");
}


