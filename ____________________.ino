#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // указываем пины rx и tx соответственно
MPU6050 accelgyro;
I2Cdev I2C_M;
uint8_t buffer_m[6];
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading;
float tiltheading;
double Axyz[3];
double Gxyz[3];
void setup()
{
 pinMode(2, INPUT);
 pinMode(3, OUTPUT);
 //подключаемся к шине I2C (I2Cdev не может сделать это самостоятельно)
 Wire.begin();
 // инициализация подключения в Мониторе порта
 Serial.begin(115200);
 mySerial.begin(115200);
 Serial.println("start prg");
 // Инициализация устройства
 Serial.println("Initializing I2C devices...");
 accelgyro.initialize();
 // Подтверждение подключения
 Serial.println("Testing device connections...");
 Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 Serial.println(" ");
}
void loop()
{ 
 getAccel_Data(); // Получение значений Акселерометра
 getGyro_Data(); // Получение значений Гироскопа
 getHeading(); // после чего мы получаем откалиброванные значения углов поворота
 getTiltHeading(); // и наклона
 
 mySerial.print(Axyz[0]);
 mySerial.print(" ");
 mySerial.print(Axyz[1]);
 mySerial.print(" ");
 mySerial.print(Axyz[2]);
 mySerial.print(" ");
 mySerial.print(Gxyz[0]);
 mySerial.print(" ");
 mySerial.print(Gxyz[1]);
 mySerial.print(" ");
 mySerial.println(Gxyz[2]);
}
void getAccel_Data(void)
{
 accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz);
 Axyz[0] = (double) ax / (double) 16384;
 Axyz[1] = (double) ay / (double) 16384;
 Axyz[2] = (double) az / (double) 16384;
}
void getGyro_Data(void)
{
 accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz);
 Gxyz[0] = (double) gx / (double) 16384;
 Gxyz[1] = (double) gy / (double) 16384;
 Gxyz[2] = (double) gz / (double) 16384;
}
