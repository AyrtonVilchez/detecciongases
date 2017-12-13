#include <GPRS_Shield_Arduino.h>

#include <SoftwareSerial.h>

#define pinTX 7
#define pinRX 8
#define rychlostSim 9600
#define PHONE_NUMBER "984473116"
#define MESSAGE1  "Hola, el sensor MQ2 supero su limite!"
#define MESSAGE2  "Hola, el sensor MQ135 supero su limite!"
char gprsServer[] = "api.thingspeak.com";

int rojoMQ2 = 5;
int rojoMQ135 = 6;
String API = "1GQ1ZPLP6P4CHR8K";

boolean vypis = 0;

GPRS sim900(pinTX , pinRX , rychlostSim);

const int MQ_PIN2 = A0;// Pin del sensor
const int MQ_PIN135 = A1; // Pin del sensor
const int RL_VALUE = 5;      // Resistencia RL del modulo en Kilo ohms
const int R0 = 10;          // Resistencia R0 del sensor en Kilo ohms

// Datos para lectura multiple
const int READ_SAMPLE_INTERVAL = 100;    // Tiempo entre muestras
const int READ_SAMPLE_TIMES = 5;       // Numero muestras

// Ajustar estos valores para vuestro sensor según el Datasheet
// (opcionalmente, según la calibración que hayáis realizado)
const float X0 = 200;
const float Y0 = 1.7;
const float X1 = 10000;
const float Y1 = 0.28;

// Puntos de la curva de concentración {X, Y}
const float punto0[] = { log10(X0), log10(Y0) };
const float punto1[] = { log10(X1), log10(Y1) };

// Calcular pendiente y coordenada abscisas
const float scope = (punto1[1] - punto0[1]) / (punto1[0] - punto0[0]);
const float coord = punto0[1] - punto0[0] * scope;

void setup() {

  Serial.begin(9600);
  pinMode(rojoMQ2, OUTPUT);
  pinMode(rojoMQ135, OUTPUT);
  digitalWrite(rojoMQ2, HIGH);
  
  while (!sim900.init()) {
    Serial.println("Error al inicializar!!");
    delay(1000);
  }
  delay(5000);
  Serial.println("inicialización tiene éxito ...");
  while (!sim900.join(F("movistar.pe"), F("movistar"), F("movistar"))) {
    Serial.println("Error al conectar con fecha de GPRS!");
    delay(2000);
  }

  Serial.print("dirección IP asignada: ");
  Serial.println(sim900.getIPAddress());
}

void loop() {


  
  float rs_med1 = readMQ(MQ_PIN2);      // Obtener la Rs promedio
  float rs_med2 = readMQ(MQ_PIN135);    // Obtener la Rs promedio
  float concentration1 = getConcentration(rs_med1 / R0); // Obtener la concentración
  float concentration2 = getConcentration(rs_med2 / R0); // Obtener la concentración
  String conc1 = String(concentration1);
  String conc2 = String(concentration2);
  // convertimos el float en un string
  if (!sim900.connect(TCP, gprsServer, 80)) {
    Serial.println("error con conexión TCP");
    delay(5000);
  }else {
    Serial.print("-> conectado al servidor");
    Serial.print(gprsServer);
    Serial.println("éxito"); 
    
    String zprava = "GET /update?api_key=";
    zprava += API;
    zprava += "&field1=";
    zprava += conc1;
    if(concentration1 >= 35000 ){
      digitalWrite(rojoMQ2, HIGH);
      delay(3000);
      Serial.println("start to send message ...");
      sim900.sendSMS(PHONE_NUMBER,MESSAGE1);
    }
    zprava += "&field2=";
    zprava += conc2;
    if(concentration2 >= 30000 ){
      digitalWrite(rojoMQ135, HIGH);
      delay(3000);
      Serial.println("start to send message ...");
      sim900.sendSMS(PHONE_NUMBER,MESSAGE2);
    }
    digitalWrite(rojoMQ135, LOW);
    digitalWrite(rojoMQ2, LOW);
    zprava += " HTTP/1.0\r\n\r\n";
    
    char zpravaChar[zprava.length() + 1];
    
    zprava.toCharArray(zpravaChar, zprava.length() + 1);
    
    sim900.send(zpravaChar, zprava.length());
    
    char odpoved[512];
    while (true) {
     
      int znaky = sim900.recv(odpoved, sizeof(odpoved) - 1);
      
      if (znaky <= 0) {
        if (vypis) {
          Serial.println("-> respuesta final.");
        }
        break;
      }
      odpoved[znaky] = '\0';
      if (vypis) {
        Serial.print("recibido ");
        Serial.print(znaky);
        Serial.print(" bytu: ");
        Serial.println(odpoved);
      }
    }
    
    sim900.close();
    sim900.disconnect();
    Serial.println("-> desconectar una conexión de datos.");
    
    delay(30000);
  }
}

// Obtener la resistencia promedio en N muestras
float readMQ(int mq_pin)
{
  float rs = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += getMQResistance(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  return rs / READ_SAMPLE_TIMES;
}

// Obtener resistencia a partir de la lectura analogica
float getMQResistance(int raw_adc)
{
  return (((float)RL_VALUE / 1000.0 * (1023 - raw_adc) / raw_adc));
}

// Obtener concentracion 10^(coord + scope * log (rs/r0)
float getConcentration(float rs_ro_ratio)
{
  return pow(10, coord + scope * log(rs_ro_ratio));
}


