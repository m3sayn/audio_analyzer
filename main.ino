
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// можете включить вывод отладочной информации в Serial на 115200
//#define REMOTEXY__DEBUGLOG    

// определение режима соединения и подключение библиотеки RemoteXY 
#define REMOTEXY_MODE__WIFI_CLOUD

#include <ESP8266WiFi.h>

// настройки соединения 
#define REMOTEXY_WIFI_SSID "Samsung"
#define REMOTEXY_WIFI_PASSWORD "74gfueu3"
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "47b2334b5c9cd5049206e034396a46d4"

#include <RemoteXY.h>

#include "arduinoFFT.h" // Стандартная Arduino FFT(Быстрое преобразование Фурье) библиотека https://github.com/kosme/arduinoFFT

#include <SPI.h>          //Подключение библиотек
#include <Adafruit_GFX.h>
#include <Max72xxPanel.h>

int pinCS = 2;
int numberOfHorizontalDisplays = 2; // количество матриц по-горизонтали
int numberOfVerticalDisplays = 1; // количество матриц по-вертикали

Max72xxPanel matrix = Max72xxPanel(pinCS, numberOfHorizontalDisplays, numberOfVerticalDisplays);

#define SAMPLES 256               // должна быть степенью 2
#define SAMPLING_FREQ 24000        // Гц, должно быть 40000 или меньше из-за времени преобразования АЦП. Определяет максимальную частоту, которую можно проанализировать с помощью БПФ Fmax=sampleF/2.
#define AUDIO_IN_PIN 0             // пин микрофон
#define NOISE 2500              // Используется в качестве фильтра грубого шума, значения ниже этого значения игнорируются.

unsigned int sampling_period_us;
int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);
byte maxValue;


// конфигурация интерфейса RemoteXY  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 62 bytes
  { 255,13,0,4,0,55,0,17,0,0,0,237,1,106,200,1,1,4,0,2,
  30,174,44,22,1,240,26,31,31,79,78,0,79,70,70,0,3,42,114,24,
  44,2,240,26,68,9,8,89,49,1,8,239,7,20,79,69,14,36,240,26,
  240,11 };
  
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // input variables
  uint8_t switch_01; // =1 если переключатель включен и =0 если отключен
  uint8_t select_01; // oт 0 до 2
  char edit_01[11]; // string UTF8 end zero

    // output variables
  float onlineGraph_02_var1;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

void setup() 
{
  RemoteXY_Init (); 
  
  // TODO you setup code

  Serial.begin(9600);
  
  matrix.setIntensity(5);                    // Задаем яркость от 0 до 15
  matrix.setRotation(2);                      // Направление текста 1,2,3,4
  delay(2000);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
  
}

void loop() 
{ 
  RemoteXY_Handler ();
  

  int value = atoi (RemoteXY.edit_01);

  if (value <= 13 && value >= 0) matrix.setIntensity(value);
  else if (value <0) matrix.setIntensity(0);
  else if (value >13) matrix.setIntensity(13);

   switch (RemoteXY.select_01) {
    case 0:
      /*  текущее состояние A */
      matrix.setRotation(2); 
      break;
    case 1:
      /*  текущее состояние B */
      matrix.setRotation(4); 
      break;
    case 2:
      /*  текущее состояние C */
      
      break;
  }
  
  // Очистить bandValues[]
  for (int i = 0; i<16; i++){
    bandValues[i] = 0;
  }

  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN); 
    RemoteXY.onlineGraph_02_var1 = float (vReal[i]);
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* ничего не делать, ждать */ }
  }
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  maxValue = 0;
    matrix.fillScreen(LOW);
    delay(2);
    
  for (int i = 2; i < (SAMPLES/2); i++){  // Не используйте i = 0, можно использовать только первые SAMPLES/2.
                // Каждый элемент массива представляет частоту, а его значение — амплитуду. Обратите внимание, что частоты не дискретны.
    if (vReal[i] > NOISE) {       // Добавьте фильтр грубого шума с амплитудой 10 x или более.
      if (i<=2 )           bandValues[0]  += (int)vReal[i];
      if (i>2   && i<=3  ) bandValues[1]  += (int)vReal[i];
      if (i>3   && i<=4  ) bandValues[2]  += (int)vReal[i];
      if (i>4   && i<=5  ) bandValues[3]  += (int)vReal[i];
      if (i>5   && i<=7  ) bandValues[4]  += (int)vReal[i];
      if (i>7   && i<=9  ) bandValues[5]  += (int)vReal[i];
      if (i>9   && i<=11  ) bandValues[6]  += (int)vReal[i];
      if (i>11   && i<=15  ) bandValues[7]  += (int)vReal[i];
      if (i>15   && i<=20  ) bandValues[8]  += (int)vReal[i];
      if (i>20   && i<=25  ) bandValues[9]  += (int)vReal[i];
      if (i>25   && i<=35  ) bandValues[10]  += (int)vReal[i];
      if (i>35   && i<=48  ) bandValues[11]  += (int)vReal[i];
      if (i>48   && i<=68  ) bandValues[12]  += (int)vReal[i];
      if (i>68   && i<=94  ) bandValues[13]  += (int)vReal[i];
      if (i>94   && i<=120  ) bandValues[14]  += (int)vReal[i];
      if (i>120             ) bandValues[15]  += (int)vReal[i];
    }

    
    for (int x = 0; x < 16; x++ )  {
      int Height = bandValues[x]/1500;
      if ( Height > 8) Height = 7;
      Height =  constrain(Height, 0, 7);
      //Serial.println (Height);
      if (Height > 0) matrix.drawLine(x, 7, x, 7 - Height, HIGH);
    }
  }
  matrix.write();
  
  if (RemoteXY.switch_01!=0) {
    /*  выключатель включен  */
    matrix.write();                 
  }
  else {
    /*  выключатель выключен */
    matrix.fillScreen(LOW);
    matrix.write();  
  }
}
