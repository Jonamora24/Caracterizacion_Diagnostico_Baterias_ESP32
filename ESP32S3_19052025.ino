/*
  Caracterizacion y Diagnostico de Baterias utilizando un ESP32S3
  ----------------------------------------------------
  Lectura de sensores de voltaje, corriente y temperatura.
  Cálculo de SOC, SOH y RUL con fusión Kalman y Coulomb counting.
  Visualización en OLED SSD1306 y envío de datos a Google Sheets.

  Desarrollado: Jhonatan Mora
  Placa: ESP32-S3
  Librerías requeridas:
    - WiFi.h
    - HTTPClient.h
    - OneWire.h
    - DallasTemperature.h
    - Adafruit_GFX.h
    - Adafruit_SSD1306.h
*/
// ---------------------------------------------
// 1. Librerías
// ---------------------------------------------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ---------------------------------------------
// 2. Definiciones de pantalla OLED
// ---------------------------------------------
#define ANCHO       128    // Anchura de la pantalla OLED
#define ALTO        64     // Altura de la pantalla OLED
#define OLED_RESET  4      // Pin de reset (no usado por SSD1306)
Adafruit_SSD1306 oled(ANCHO, ALTO, &Wire, OLED_RESET);

// ---------------------------------------------
// 3. Credenciales WiFi y URL de Google Sheets
// ---------------------------------------------
const char* ssid      = "lab_eer";
const char* password  = "electronica-100";
const char* scriptURL = "https://script.google.com/macros/s/AKfy...NVzm/exec";

// ---------------------------------------------
// 4. Pines de sensores y parámetros físicos
// ---------------------------------------------
#define DS18B20_PIN  10    // GPIO para sensor DS18B20
#define ACS758_PIN    1    // ADC para sensor de corriente ACS758
#define FZ0430_PIN    2    // ADC para sensor de voltaje FZ0430
#define OFFSET       0.35  // Offset del ACS758 en V

// ---------------------------------------------
// 5. Parámetros de muestreo y filtros
// ---------------------------------------------
const int   numSamples      = 20;      // Lecturas para promedio de corriente
const float eficiencia      = 0.98;    // Eficiencia coulómbica
const float tiempoEnAnios   = 2.0;     // Horizonte de vida útil en años
/*La variable tiempoEnAnios....Depende del amperaje y el tipo de bateria: 2 a 3 años (300 a 500 ciclos de carga/descarga
  y 5 a 10 años (2000 a 5000 ciclos de carga/descarga.  Si es menor a 1 amperio, por seguridad, se coloca 1 año*/

// ---------------------------------------------
// 6. Parámetros Kalman (SOC)
// ---------------------------------------------
float Q      = 0.00001;  // Ruido de proceso
float R      = 0.01;     // Ruido de medición
float P      = 1.0;      // Covarianza inicial
float K      = 0.0;      // Ganancia de Kalman

// ---------------------------------------------
// 7. Parámetros de la batería
// ---------------------------------------------
float capacityNominal = 4.1;   // Capacidad nominal en Ah

// ---------------------------------------------
// 8. Variables globales de estado
// ---------------------------------------------
float voltaje, corriente, temperatura;
float SOC_inicial, SOC, SOH, RUL, capacidad;

unsigned long previousTime   = 0;   // Timestamp de último muestreo (ms)
bool mostrarVentana1         = true; // Flag para alternar vistas OLED

// ---------------------------------------------
// 9. Objetos de sensores
//Este caso, se enfoca en el sensor de temperatura
// ---------------------------------------------
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// ---------------------------------------------
// 10. Prototipos de funciones
// ---------------------------------------------
float voltaje_a_SOC(float volt);
float readCurrent();
float readAverageCurrent();
void LeerSensores();
void mostrarVentanaSensores();
void mostrarVentanaEstimaciones();
void enviarDatosGoogle();

// =============================================
// Setup inicial
// =============================================
void setup() {
  Serial.begin(115200);

  // Conectar a WiFi
  WiFi.begin(ssid, password);

  // Inicializar sensor de temperatura DS18B20
  sensors.begin();
  analogReadResolution(12);

  // Inicializar bus I2C y pantalla OLED
  Wire.begin(8, 9);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.println(F("Caracterizacion y diagnostico baterias")); //imprimir texto en OLED
  oled.display();
  delay(2000);

  // Estado inicial de muestreo y SOC
  previousTime = millis();
  SOC_inicial  = voltaje_a_SOC(voltaje);
}

// =============================================
// Bucle principal
// =============================================
void loop() {
  LeerSensores();

  // Alternar vista cada 4 segundos
  if (millis() - previousTime >= 4000) {
    mostrarVentana1 = !mostrarVentana1;
    previousTime   = millis();
  }

  // Mostrar datos según flag
  if (mostrarVentana1) {
    mostrarVentanaSensores();
  } else {
    mostrarVentanaEstimaciones();
  }
}

// =============================================
// 11. Implementación de funciones
// =============================================

/**
 * Primera aproximacion del SOC
 */
float voltaje_a_SOC(float volt) {
  const float V_min = 2.5;
  const float V_max = 4.0;
  return constrain((volt - V_min) / (V_max - V_min), 0.0, 1.0);
}

/**
 * Lee corriente instantánea (A) desde ACS758
 */
float readCurrent() {
  int raw = analogRead(ACS758_PIN);
  float volt = (raw / 4095.0) * 3.3;
  return ((volt - OFFSET) / 0.061) + 7.87;
}

/**
 * Calcula corriente promedio de numSamples lecturas
 */
float readAverageCurrent() {
  float total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += readCurrent();
    delay(50);
  }
  return total / numSamples;
}

/**
 * Actualiza lecturas de sensores y calcula capacidad actual
 */
void LeerSensores() {
  sensors.requestTemperatures(); //inicializa sensor de temperatura
  temperatura = sensors.getTempCByIndex(0) - 1.5; //afectado por condiciones climaticas para hallar la temperatura ambiente.

  int rawV = analogRead(FZ0430_PIN);
  voltaje = (rawV / 4095.0) * 3.3 * 5.0; //incializa sensor de voltaje

  corriente = readAverageCurrent(); //funcion de corriente
  unsigned long now = millis();
  float deltaSec = (now - previousTime) / 1000.0;  //encontrar delta

  float capInit = capacityNominal * SOC_inicial;  //variable dependiente para hallas capacidad
  capacidad = capInit + (corriente * deltaSec / 3600.0) * eficiencia; //formula para capacidad
}

/**
 * Muestra lecturas en OLED y envía datos a Google Sheets
 */
void mostrarVentanaSensores() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Caracterizacion"));
  oled.setCursor(0, 10);
  oled.println("Capacidad: " + String(capacidad, 1) + " Ah");
  oled.setCursor(0, 20);
  oled.println("Voltaje: " + String(voltaje, 1) + " V");
  oled.setCursor(0, 30);
  oled.println("Corriente: " + String(corriente, 1) + " A");
  oled.setCursor(0, 40);
  oled.println("Temp: " + String(temperatura, 1) + " C");
  oled.display();
  delay(1000);
  enviarDatosGoogle();
}

/**
 * Estima SOC, SOH y RUL, muestra en OLED y envía datos
 */
void mostrarVentanaEstimaciones() {
  const float DeltaTime = 1.0;

  // Predicción y corrección Kalman para SOC
  float SOC_pred = SOC_inicial - (corriente * DeltaTime) / (capacityNominal * 3600.0);
  P += Q;
  float SOC_mes = voltaje_a_SOC(voltaje);
  K = P / (P + R);
  SOC = SOC_pred + K * (SOC_mes - SOC_pred);
  P *= (1 - K);
  SOC = constrain(SOC, 0.0, 1.0);
  SOC_inicial = SOC;

  // Cálculo de SOH
  SOH = capacidad / capacityNominal;

  // Cálculo de RUL (horas)
  float degrP = (1 - (capacidad / capacityNominal)) * 100;
  float tasaAno = degrP / tiempoEnAnios;
  float tasaAh  = (capacityNominal * tasaAno) / 100;
  float umbral  = 0.8 * capacityNominal;
  RUL = ((capacidad - umbral) / tasaAh) * 8760.0;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Diagnostico"));
  oled.setCursor(0, 10);
  oled.println("SOC: " + String(SOC*100, 1) + " %");
  oled.setCursor(0, 20);
  oled.println("SOH: " + String(SOH*100, 1) + " %");
  oled.setCursor(0, 30);
  oled.println("RUL: " + String(RUL, 1) + " hrs");
  oled.display();
  delay(1000);
  enviarDatosGoogle();
}

/**
 * Envía datos a Google Sheets mediante HTTP GET
 */
void enviarDatosGoogle() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(scriptURL) + "?volt=" + String(voltaje,2)
      + "&corr=" + String(corriente,2)
      + "&temp=" + String(temperatura,2)
      + "&SOC=" + String(SOC*100,2)
      + "&SOH=" + String(SOH*100,2)
      + "&RUL=" + String(RUL,2);
    http.begin(url);
    http.GET();
    http.end();
  } else {
    Serial.println("WiFi desconectado.");
  }
}

/*
                                        Recomendaciones para asegurar identidad de resultados
-------------------------------------------------------------------------------------------------------------------------------------------------
- Inicialización idéntica: comprueba que SOC_inicial y P arranquen con el mismo valor en la hoja de calculo y en el sketch.

- Consistencia de muestreo: si el Excel asume que cada iteración es un minuto en lugar de un segundo, modificar DeltaTime = 60.0 en el código.

- Comprobación de flotantes: imprime en Arduino (por Serial.println) los valores intermedios que en Excel tienes en columnas, y compara fila
por fila.

- Evita acumulación de error: si en Excel redondeas cada paso a 2 decimales, haz lo mismo en Arduino con roundf(valor * 100)/100.0 
para mantener paridad.

- Tener presenta cambiar las variables independientes del inicio del codigo
 */
