#include <Wire.h>  // I2C library
#include <stdint.h> // Standard C, allows explicit data type declaration

byte fetch_pressure(unsigned int *p_Pressure); // Function to fetch pressure data

// MS4525D sensor characteristic
const uint8_t MS4525DAddress = 0x28;
const int16_t MS4525FullScaleRange = 1; // 1 psi
const int16_t MS4525MinScaleCounts = 1638;  // Voff
const int16_t MS4525FullScaleCounts = 14732; // Vfso
const int16_t MS4525Span = MS4525FullScaleCounts - MS4525MinScaleCounts;  // Span
const int16_t MS4525ZeroCounts = (MS4525FullScaleCounts + MS4525MinScaleCounts) / 2;  // Zero point

byte _status; // I2C status
uint16_t P_dat, T_dat; // 14-bit pressure data and 11-bit temperature data
float psi, Vms, Vkmh, psioff;
float temperatureC; // Variable to store temperature in Celsius
float airDensity; // Variable to store calculated air density

// Manual Relative Humidity Input (adjust this value manually)
float relativeHumidity = 0.506; // Example: 52% RH is entered as 0.52

// Constants for air density calculation
const float R_d = 287.05; // J/(kg·K) - specific gas constant for dry air
const float R_v = 461.5;  // J/(kg·K) - specific gas constant for water vapor

byte fetch_pressure(uint16_t &P_dat, uint16_t &T_dat) {
  byte _status, Press_H, Press_L, Temp_H, Temp_L;
  Wire.beginTransmission(MS4525DAddress);
  delay(100);
  Wire.requestFrom(MS4525DAddress, (uint8_t)4, (uint8_t)false); // Request 4 bytes (pressure + temperature)
  delay(100);
  Press_H = Wire.read();
  Press_L = Wire.read();
  Temp_H = Wire.read();
  Temp_L = Wire.read();
  Wire.endTransmission();

  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((uint16_t)Press_H) << 8) | Press_L;

  Temp_L = (Temp_L >> 5);
  T_dat = (((uint16_t)Temp_H) << 3) | Temp_L;

  return _status;
}

float calculateAirDensity(float temperatureC, float pressurePascal, float relativeHumidity) {
  // Convert temperature to Kelvin
  float temperatureK = temperatureC + 273.15;

  // Calculate saturation vapor pressure (e_sat) in Pascals
  float e_sat = 610.78 * exp((17.27 * temperatureC) / (temperatureC + 237.3));

  // Calculate air density using the formula
  float airDensity = (pressurePascal / (R_d * temperatureK)) * (1 - ((relativeHumidity * e_sat) / pressurePascal) * (1 - (R_d / R_v)));
  
  return airDensity;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  while (!Serial) {
    delay(1);
  }
  Serial.println("MS4525D0 test");
  _status = fetch_pressure(P_dat, T_dat);
  Serial.print("P_raw Offset: ");
  Serial.print(P_dat);
  psioff = abs((float)((int16_t)P_dat - MS4525ZeroCounts) / (float)MS4525Span * (float)MS4525FullScaleRange);
  Serial.print("\t");
  Serial.print("Psi Offset: ");
  Serial.println(psioff, 6);
}

void loop() {
  _status = fetch_pressure(P_dat, T_dat);
  switch (_status) {
    case 0: break; // OK
    case 1: Serial.println("Busy"); break;
    case 2: Serial.println("Slate"); break;
    default: Serial.println("Error"); break;
  }

  // Calculate Pressure
  psi = (float)((int16_t)P_dat - MS4525ZeroCounts) / (float)MS4525Span * (float)MS4525FullScaleRange - psioff;
  psi = abs(psi);

  // Convert psi to Pascal (1 psi = 6894.76 Pascals)
  float pressurePascal = psi * 6894.76;

  // Calculate Speed (m/s and km/h)
  Vms = sqrt((psi * 13789.5144) / 1.172);  // Speed in m/s
  Vkmh = Vms * 3.6;  // Speed in km/h

  // Calculate Temperature in Celsius
  temperatureC = ((float)T_dat / 2047.0) * 200.0 - 50.0;

  // Calculate Air Density
  airDensity = calculateAirDensity(temperatureC, pressurePascal, relativeHumidity);

  // Output to Serial Monitor
  Serial.print("psioff: "); Serial.print(psioff, 6);
  Serial.print(" , ");
  Serial.print("raw Pressure: "); Serial.print(P_dat);
  Serial.print(" , ");
  Serial.print("pressure psi: "); Serial.print(psi, 6);
  Serial.print(" , ");
  Serial.print("speed m/s: "); Serial.print(Vms, 2);
  Serial.print(" , ");
  Serial.print("speed km/h: "); Serial.print(Vkmh, 2);
  Serial.print(" , ");
  Serial.print("Temperature (°C): "); Serial.print(temperatureC, 2);
  Serial.print(" , ");
  Serial.print("Air Density (kg/m^3): "); Serial.println(airDensity, 6);

  delay(100);
}
