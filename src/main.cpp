#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int PUSHBUTTONPIN = 2;
const int BUZZERPIN = 3;
const int GYROBUFFERSIZE = 50;
const float SHOTTHREASHHOLDGYRO = 0.06;
// earth gravity + 2G
const float SHOTTHREASHHOLDACC = 9.81 + 2.0;
sensors_vec_t gyrobuffer[GYROBUFFERSIZE];
float x[GYROBUFFERSIZE];
float y[GYROBUFFERSIZE];
float z[GYROBUFFERSIZE];
int shots = 0;
unsigned long looptimesum = 0;
unsigned int looptimecount = 0;

void initGyro()
{
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  Serial.print("CycleRate set to: ");
  switch (mpu.getCycleRate())
  {
  case MPU6050_CYCLE_1_25_HZ:
    Serial.println("1.25 Hz");
    break;
  case MPU6050_CYCLE_5_HZ:
    Serial.println("5 Hz");
    break;
  case MPU6050_CYCLE_20_HZ:
    Serial.println("20 Hz");
    break;
  case MPU6050_CYCLE_40_HZ:
    Serial.println("40 Hz");
    break;
  }

  Serial.println();
}

void printValues(sensors_event_t a, sensors_event_t g, sensors_event_t temp, boolean buttonPushed, float dacc, float ddeg)
{
  /* Print out the values */
  Serial.print("  dacc, dgyro: ");
  Serial.print(dacc);
  Serial.print(", ");
  Serial.print(ddeg);

  Serial.print("  Acc X,Y,Z: ");

  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(" m/s^2");

  Serial.print("  Rot X,Y,Z: ");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.print(" rad/s");

  Serial.print(" Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" degC");

  Serial.print("  shots: ");
  Serial.print(shots);

  Serial.print("  button push: ");
  Serial.print(buttonPushed);

  Serial.print("  avg: ");
  Serial.print(looptimesum/looptimecount);

  Serial.println();
}

void saveGyroToEEPROM(int startfrom)
{
  // save to EEPROM x,y,z global gyro values
  Serial.print("Saving gyro data to EEPROM. Total bytes: ");
  Serial.println(__SIZEOF_INT__ + GYROBUFFERSIZE * 3 * __SIZEOF_FLOAT__);
  int offset = 0;
  EEPROM.put(offset + 0, shots);
  offset += __SIZEOF_INT__;
  for (int i = 0; i < GYROBUFFERSIZE; i++)
  {
    EEPROM.put(offset + 3 * i * __SIZEOF_FLOAT__, x[((startfrom + i) % GYROBUFFERSIZE)]);
    EEPROM.put(offset + 3 * i * __SIZEOF_FLOAT__ + __SIZEOF_FLOAT__, y[((startfrom + i) % GYROBUFFERSIZE)]);
    EEPROM.put(offset + 3 * i * __SIZEOF_FLOAT__ + 2 * __SIZEOF_FLOAT__, z[((startfrom + i) % GYROBUFFERSIZE)]);
  }
  offset += 3 * GYROBUFFERSIZE * __SIZEOF_FLOAT__;
  int avg = (int)(looptimesum / looptimecount);
  // Serial.print(" avg: ");
  Serial.println(avg);
  EEPROM.put(offset, avg);
}

void readGyroFromEEPROM()
{
  // read from EEPROM x,y,z global gyro values
  Serial.print("Reading gyro data from EEPROM. Total bytes: ");
  Serial.println(__SIZEOF_INT__ + GYROBUFFERSIZE * 3 * __SIZEOF_FLOAT__);
  int offset = 0;
  EEPROM.get(offset + 0, shots);
  offset += __SIZEOF_INT__;
  for (int i = 0; i < GYROBUFFERSIZE; i++)
  {
    EEPROM.get(offset + 3 * i * __SIZEOF_FLOAT__, x[i]);
    EEPROM.get(offset + 3 * i * __SIZEOF_FLOAT__ + __SIZEOF_FLOAT__, y[i]);
    EEPROM.get(offset + 3 * i * __SIZEOF_FLOAT__ + 2 * __SIZEOF_FLOAT__, z[i]);
  }
  offset += 3 * GYROBUFFERSIZE * __SIZEOF_FLOAT__;
  int avg;
  EEPROM.get(offset, avg);
  // Serial.print(" avg: ");
  Serial.println(avg);
}

void printGyro()
{
  Serial.print("shots:  ");
  Serial.print(shots);
  for (long i = 0; i < GYROBUFFERSIZE; i++)
  {
    if (i % 10 == 0)
      Serial.println();
    Serial.print(x[i]);
    Serial.print(",");
    Serial.print(y[i]);
    Serial.print(",");
    Serial.print(z[i]);
    Serial.print("#");
  }
  Serial.println();
}

void beep()
{
  digitalWrite(BUZZERPIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(BUZZERPIN, LOW);
}

void blink()
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(50);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
}

void setup(void)
{
  blink();
  Serial.begin(9600);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  initGyro();

  pinMode(PUSHBUTTONPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Print length of data to run CRC on.
  Serial.print("EEPROM length: ");
  Serial.println(EEPROM.length());

  readGyroFromEEPROM();
  printGyro();
  beep();
  blink();
}

void loop()
{
  unsigned long startTime = millis();
  static boolean ledon = true;
  digitalWrite(LED_BUILTIN, ledon);
  ledon = !ledon;
  static int i = 0;
  boolean buttonPushed = digitalRead(PUSHBUTTONPIN) == LOW;
  if (buttonPushed)
  {
    //initGyro();
    shots = 0;
    delay(1000);
  }

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float dacc = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  float ddeg = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z);

  x[i] = g.gyro.x;
  y[i] = g.gyro.y;
  z[i] = g.gyro.z;

  Serial.print(" i: ");
  Serial.print(i);
  Serial.print(" : ");
  printValues(a, g, temp, buttonPushed, dacc, ddeg);

  // shot detected
  if (ddeg > SHOTTHREASHHOLDGYRO && dacc > SHOTTHREASHHOLDACC)
  {
    blink();
    Serial.println();
    Serial.print("Shot detected with deg: ");
    Serial.print(ddeg);
    Serial.print(", degthr: ");
    Serial.print(SHOTTHREASHHOLDGYRO);
    Serial.print(", acc: ");
    Serial.print(dacc);
    Serial.print(", accthr: ");
    Serial.println(SHOTTHREASHHOLDACC);
    shots++;
    //printGyro();
    saveGyroToEEPROM(i + 1); // write from the first available reading
    blink();
  }
  /*
  // print out values, if more than 1000 ms passed
  if (millis() - lastUpdate > 1000)
  {
    lastUpdate = millis();
    printValues(a, g, temp, buttonPushed);
  }
  */
  unsigned long endTime = millis();
  unsigned int elapsed = endTime - startTime;
  // Serial.print("elapsed: ");
  // Serial.println(elapsed);
  looptimesum += elapsed;
  looptimecount++;
  // if (looptimecount != 0)
  // {
  //   Serial.print("looptimeavg: ");
  //   Serial.println(looptimesum / looptimecount);
  // }

  if (++i >= GYROBUFFERSIZE)
    i = 0;
}
