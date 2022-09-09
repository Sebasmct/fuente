#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "actuador_gps.h"


#define PinLed 12
#define NUMPIXELS 16

// pines de los reles
#define rele_1 9
#define rele_2 8
#define rele_3 7
#define rele_4 6
#define rele_5 5
#define rele_6 4
#define rele_7 3
#define rele_8 2
#define rele_9 23
#define rele_10 22
#define rele_11 21
#define rele_12 20
#define rele_13 17
#define rele_14 16
#define rele_15 15
#define rele_16 14

int PosRelay[16] = {9, 8, 7, 6, 5, 4, 3, 2, 23, 22, 21, 20, 17, 16, 15, 14};

// registros de configuracion INA219
#define INA219_REG_CALIBRATION                 (0x05)
#define INA219_REG_CONFIG                      (0x00)
#define INA219_REG_BUSVOLTAGE                  (0x02)
#define INA219_REG_SHUNTVOLTAGE                (0x01)
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)
#define INA219_REG_CURRENT                     (0x04)

unsigned char vector_ina_address[16] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F};

//variables ina219
uint8_t ina219_i2caddr;
uint32_t ina219_calValue;
float busVoltage_V, shuntVoltage_mV, voltage_V_1 = 0, voltage_V_2 = 0, current_mA = 0, current_mA_41 = 0;
int i, reg;
int contador = 0;

//funcioes ina219
unsigned char ina219_currentDivider_mA, ina219_powerDivider_mW;

int PosLedRelay[16] = {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};
float corrientes[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float corrientesmemory[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float voltage[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Rele deja de pasar corriente con LOW
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PinLed, NEO_GRB + NEO_KHZ800);


void setup() {
  //funcion ina219
  for ( int i = 0; i <= 15; i++) {

    setCalibration_32V_2A(vector_ina_address[i]);
    delay(100);
  }

  Wire.setSCL(19);
  Wire.setSDA(18);
  Wire.begin();

  pixels.begin();
  Serial.begin(115200);

  //endendido leds
  iniciar_leds();
  apagar_leds();


  //definimos los reles como salidas
  pinMode(rele_1, OUTPUT);
  pinMode(rele_2, OUTPUT);
  pinMode(rele_3, OUTPUT);
  pinMode(rele_4, OUTPUT);
  pinMode(rele_5, OUTPUT);
  pinMode(rele_6, OUTPUT);
  pinMode(rele_7, OUTPUT);
  pinMode(rele_8, OUTPUT);
  pinMode(rele_9, OUTPUT);
  pinMode(rele_10, OUTPUT);
  pinMode(rele_11, OUTPUT);
  pinMode(rele_12, OUTPUT);
  pinMode(rele_13, OUTPUT);
  pinMode(rele_14, OUTPUT);
  pinMode(rele_15, OUTPUT);
  pinMode(rele_16, OUTPUT);
  //apagamos los relays
  Reles_off();
  //Reles_on();
}

void loop() {

  datos_sensor();
  carga_unidades();
}


void datos_sensor() {

  //obtencion datos ina219

  for (int i = 0; i <= 15; i++) {
    corrientes[i] = get_data(vector_ina_address[i]);
    voltage[i] = get_voltaje(vector_ina_address[i]);
  }

}


void carga_unidades() {

  for (int i = 0; i <= 15; i++) {

    if (corrientes[i] < 1) {
      datos_sensor();
      digitalWrite(PosRelay[i], HIGH);
      pixels.setPixelColor(PosLedRelay[i], pixels.Color(0, 0, 0));
      pixels.show();
      delay(5);
    }

    else if (corrientes[i] > 400) {
      //datos_sensor();
      digitalWrite(PosRelay[i], HIGH);
      //color rojo
      pixels.setPixelColor(PosLedRelay[i], pixels.Color(255, 0, 0));
      pixels.show();
      delay(5);
      //      Serial.println("rojo");
      //      delay(10);
    }
    else if (corrientes[i] >= 30 &&  corrientes[i] <= 400) {
      //amarillo
      pixels.setPixelColor(PosLedRelay[i], pixels.Color(255, 80, 0));
      pixels.show();
      delay(5);
      //      Serial.println("amarillo");
      //      delay(10);
    }

    else if (corrientes[i] <= 29 && corrientes[i] >= 6) {
      //verde
      contador ++;
      //digitalWrite(PosRelay[i], LOW);
      pixels.setPixelColor(PosLedRelay[i], pixels.Color(0, 255, 0));
      pixels.show();
      delay(5);
      
    }
    if (contador > 7400) {
        digitalWrite(PosRelay[i], LOW);
        pixels.setPixelColor(PosLedRelay[i], pixels.Color(0, 255, 0));
        contador =0;
      }

    

    if (voltage[i] < 2) {
      digitalWrite(PosRelay[i], LOW);
      pixels.setPixelColor(PosLedRelay[i], pixels.Color(180, 180, 180));
      pixels.show();
      delay(5);
    }

  }

  Serial.print(contador);
}

void Reles_on (void) {

  for (int i = 0; i <= 15; i++) {
    digitalWrite(PosRelay[i], LOW);
    delay(5);
  }

}

void Reles_off (void) {

  for (int i = 0; i <= 15; i++) {

    digitalWrite(PosRelay[i], HIGH);
    delay(5);
  }
}


// leds para encendido
void iniciar_leds(void)
{
  apagar_leds();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(PosLedRelay[i], pixels.Color(150, 150, 150));
    pixels.show();
    delay(150);
  }
  //  Serial.print("mostrar led");
  pixels.show();
  delay(1000);
}

////led amarillo
//  pixels.setPixelColor(PosLedRelay[0], pixels.Color(255, 80, 0));


//led verde
//  pixels.setPixelColor(PosLedRelay[0], pixels.Color(0, 255, 0));


////led rojo
//  pixels.setPixelColor(PosLedRelay[0], pixels.Color(255, 0, 0));

void apagar_leds(void)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
  }
  pixels.show();
}

void wireWriteRegister (unsigned char SlaveAddress, uint8_t reg, uint16_t value) {

  Wire.beginTransmission(SlaveAddress);
#if ARDUINO >= 100

  Wire.write(reg);                       // Register
  Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
  Wire.write(value & 0xFF);              // Lower 8-bits
#else
  Wire.send(reg);                        // Register
  Wire.send(value >> 8);                 // Upper 8-bits
  Wire.send(value & 0xFF);               // Lower 8-bits
#endif
  Wire.endTransmission();
}

void wireReadRegister(unsigned char SlaveAddress, uint8_t reg, uint16_t *value) {

  Wire.beginTransmission(SlaveAddress);
#if ARDUINO >= 100
  Wire.write(reg);                       // Register
#else
  Wire.send(reg);                        // Register
#endif
  Wire.endTransmission();

  delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(SlaveAddress, (uint8_t)2);
#if ARDUINO >= 100
  // Shift values to create properly formed integer
  *value = ((Wire.read() << 8) | Wire.read());
#else
  // Shift values to create properly formed integer
  *value = ((Wire.receive() << 8) | Wire.receive());
#endif

}

void setCalibration_32V_2A(unsigned char SlaveAddress) {

  ina219_calValue = 4096;
  ina219_currentDivider_mA = 10;  // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

  wireWriteRegister(SlaveAddress, INA219_REG_CALIBRATION, ina219_calValue);
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(SlaveAddress, INA219_REG_CONFIG, config);

}


float getBusVoltage_V(unsigned char Slaveaddress) {
  int16_t value = getBusVoltage_raw(Slaveaddress);
  return value * 0.001;
}

int getBusVoltage_raw(unsigned char Slaveaddress) {
  uint16_t value;
  wireReadRegister(Slaveaddress, INA219_REG_BUSVOLTAGE, &value);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}


float getShuntVoltage_mV(unsigned char Slaveaddress) {
  int16_t value;
  value = getShuntVoltage_raw(Slaveaddress);
  return value * 0.01;
}

int getShuntVoltage_raw (unsigned char Slaveaddress) {
  uint16_t value;
  wireReadRegister(Slaveaddress, INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

int16_t getCurrent_raw(unsigned char SlaveAddress) {

  uint16_t value;
  wireWriteRegister(SlaveAddress, INA219_REG_CALIBRATION, ina219_calValue);
  wireReadRegister(SlaveAddress, INA219_REG_CURRENT, &value);
  return (int16_t)value;

}

float getCurrent_mA(unsigned char SlaveAddress) {
  float valueDec = getCurrent_raw(SlaveAddress);
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

float get_data(unsigned char SlaveAddress) {

  busVoltage_V = getBusVoltage_V(SlaveAddress);
  shuntVoltage_mV = getShuntVoltage_mV(SlaveAddress);
  voltage_V_1 = busVoltage_V + (shuntVoltage_mV / 1000);
  current_mA = getCurrent_mA(SlaveAddress);
  imprimir_Datos(SlaveAddress);
  return   current_mA;
  //return voltage_V_1;

}

float get_voltaje(unsigned char SlaveAddress) {

  busVoltage_V = getBusVoltage_V(SlaveAddress);
  shuntVoltage_mV = getShuntVoltage_mV(SlaveAddress);
  voltage_V_1 = busVoltage_V + (shuntVoltage_mV / 1000);
  current_mA = getCurrent_mA(SlaveAddress);
  //imprimir_Datos(SlaveAddress);
  //return   current_mA;
  return voltage_V_1;

}

void imprimir_Datos(unsigned char SlaveAddress) {
  switch (SlaveAddress) {
    case 0x40: Serial.print(" 0x40, "); Serial.print(current_mA ); delay(10); break;
    case 0x41: Serial.print(" 0x41, "); Serial.print(current_mA ); delay(10); break;
    case 0x42: Serial.print(" 0x42, "); Serial.print(current_mA ); delay(10); break;
    case 0x43: Serial.print(" 0x43, "); Serial.print(current_mA ); delay(10); break;
    case 0x44: Serial.print(" 0x44, "); Serial.print(current_mA ); delay(10); break;
    case 0x45: Serial.print(" 0x45, "); Serial.print(current_mA ); delay(10); break;
    case 0x46: Serial.print(" 0x46, "); Serial.print(current_mA ); delay(10); break;
    case 0x47: Serial.print(" 0x47, "); Serial.print(current_mA ); delay(10); break;
    case 0x48: Serial.print(" 0x48, "); Serial.print(current_mA ); delay(10); break;
    case 0x49: Serial.print(" 0x49, "); Serial.print(current_mA ); delay(10); break;
    case 0x4A: Serial.print(" 0x4A, "); Serial.print(current_mA ); delay(10); break;
    case 0x4B: Serial.print(" 0x4B, "); Serial.print(current_mA ); delay(10); break;
    case 0x4C: Serial.print(" 0x4C, "); Serial.print(current_mA ); delay(10); break;
    case 0x4D: Serial.print(" 0x4D, "); Serial.print(current_mA ); delay(10); break;
    case 0x4E: Serial.print(" 0x4E, "); Serial.print(current_mA ); delay(10); break;
    case 0x4F: Serial.print(" 0x4F, "); Serial.println(current_mA ); delay(10); break;
  }

}
