#include <AX12A.h>
#include "BluetoothSerial.h"

// ——— Parámetros Dynamixel ———
#define DXL_BAUD       1000000ul   // Velocidad Half-Duplex a 1 Mbps
#define DXL_DIR_PIN    2u          // Pin para controlar la dirección (RX/TX)
const uint8_t ID_BASE     = 1;
const uint8_t ID_SHOULDER = 2;
const uint8_t ID_SHOULDERB = 3;
const uint8_t ID_ELBOW    = 4;
const uint8_t ID_GRIPPER  = 5;

// Variables para leer datos de sensores y posición
int posBase, tempBase, voltBase, loadBase;
int posShoulder, posElbow, posGripper;
int posShoulderB=511;
// ——— Pines para Serial2 (bus DXL) ———
const int DXL2_RX = 16;
const int DXL2_TX = 17;
// ——— Instancia del bus Dynamixel ———
AX12A dxlBus;

// ——— Bluetooth Serial ———
BluetoothSerial SerialBT;

// ——— Buffer para recibir la cadena de comando y contar comas ———
String inputBuf;
uint8_t commaCount = 0;

// ———————————————————————————————
// Convierte grados (0–180) a unidad de posición 0–1023
// ———————————————————————————————
int gradosAposicion(int grados) {
  float factor = 1023.0f / 300.0f;       // 300° cubre toda la escala 0–1023
  return round(grados * factor);
}

// ———————————————————————————————
// Convierte unidad de posición (0–1023) a ángulo real 0–300°
// ———————————————————————————————
float posicionAangulo(int pos) {
  return (pos * 300.0f) / 1023.0f;
}

void irAHOME() {
  // Objetivos HOME
  int homeBase     = 0;
  int homeShoulder = 818;
  int homeShoulderB= 204;
  int homeElbow    = 511;
  int homeGripper  = 511;

  // Variables para posiciones objetivo (incrementales)
  int targetBase, targetShoulder, targetShoulderB, targetElbow, targetGripper;
  
  // Variables auxiliares para controlar movimiento
  bool reachedBase     = true;
  bool reachedShoulder = true;
  bool reachedShoulderB= true;
  bool reachedElbow    = true;
  bool reachedGripper  = true;
  
  Serial.println("Iniciando movimiento a HOME...");
  
  // Lee posiciones iniciales y las asigna como objetivos iniciales
  posBase     = dxlBus.readPosition(ID_BASE);
  posShoulder = dxlBus.readPosition(ID_SHOULDER);
  posShoulderB = dxlBus.readPosition(ID_SHOULDERB);
  posElbow    = dxlBus.readPosition(ID_ELBOW);
  posGripper  = dxlBus.readPosition(ID_GRIPPER);
  
  // Inicializa objetivos con posiciones actuales
  targetBase = posBase;
  targetShoulder = posShoulder;
  targetShoulderB = posShoulderB;
  targetElbow = posElbow;
  targetGripper = posGripper;

  // Bucle hasta que todos lleguen a su posición
  while (!(reachedBase && reachedShoulder && reachedShoulderB && reachedElbow && reachedGripper)) {
    
    // *** LECTURA CONSTANTE DE POSICIONES REALES ***
    int realPosBase     = dxlBus.readPosition(ID_BASE);
    int realPosShoulder = dxlBus.readPosition(ID_SHOULDER);
    int realPosShoulderB = dxlBus.readPosition(ID_SHOULDERB);
    int realPosElbow    = dxlBus.readPosition(ID_ELBOW);
    int realPosGripper  = dxlBus.readPosition(ID_GRIPPER);
    
    // Actualiza variables globales con posiciones reales
    if (realPosBase != -1) posBase = realPosBase;
    if (realPosShoulder != -1) posShoulder = realPosShoulder;
    if (realPosShoulderB != -1) posShoulderB = realPosShoulderB;
    if (realPosElbow != -1) posElbow = realPosElbow;
    if (realPosGripper != -1) posGripper = realPosGripper;

    // Verificar estado de motores
    bool estado = checkMotorsPowered();

    Serial.printf("Estado motores: B:%d S:%d SB:%d E:%d G:%d - Powered: %s\n",
              posBase, posShoulder, posShoulderB,
              posElbow, posGripper, estado ? "Sí" : "No");

    // Control de BASE - movimiento de 1 en 1
    if (!reachedBase && estado && realPosBase != -1) {
      if (targetBase < homeBase) {
        targetBase++;
        dxlBus.move(ID_BASE, targetBase);
      } else if (targetBase > homeBase) {
        targetBase--;
        dxlBus.move(ID_BASE, targetBase);
      } else {
        reachedBase = true;
        Serial.println("BASE llegó a HOME");
      }
    }

    // Control de SHOULDER - movimiento de 1 en 1
    if (!reachedShoulder && estado && realPosShoulder != -1) {
      if (targetShoulder < homeShoulder) {
        targetShoulder++;
        dxlBus.move(ID_SHOULDER, targetShoulder);
      } else if (targetShoulder > homeShoulder) {
        targetShoulder--;
        dxlBus.move(ID_SHOULDER, targetShoulder);
      } else {
        reachedShoulder = true;
        Serial.println("SHOULDER llegó a HOME");
      }
    }

    // Control de SHOULDERB - movimiento de 1 en 1
    if (!reachedShoulderB && estado && realPosShoulderB != -1) {
      if (targetShoulderB < homeShoulderB) {
        targetShoulderB++;
        dxlBus.move(ID_SHOULDERB, targetShoulderB);
      } else if (targetShoulderB > homeShoulderB) {
        targetShoulderB--;
        dxlBus.move(ID_SHOULDERB, targetShoulderB);
      } else {
        reachedShoulderB = true;
        Serial.println("SHOULDERB llegó a HOME");
      }
    }

    // Control de ELBOW - movimiento de 1 en 1 (solo si no hay error de comunicación)
    if (!reachedElbow && estado && realPosElbow != -1) {
      if (targetElbow < homeElbow) {
        targetElbow++;
        dxlBus.move(ID_ELBOW, targetElbow);
      } else if (targetElbow > homeElbow) {
        targetElbow--;
        dxlBus.move(ID_ELBOW, targetElbow);
      } else {
        reachedElbow = true;
        Serial.println("ELBOW llegó a HOME");
      }
    } else if (realPosElbow == -1) {
      Serial.println("ERROR: No se puede comunicar con ELBOW");
      reachedElbow = true; // Marca como completado para evitar bucle infinito
    }

    // Control de GRIPPER - movimiento de 1 en 1 (solo si no hay error de comunicación)
    if (!reachedGripper && estado && realPosGripper != -1) {
      if (targetGripper < homeGripper) {
        targetGripper++;
        dxlBus.move(ID_GRIPPER, targetGripper);
      } else if (targetGripper > homeGripper) {
        targetGripper--;
        dxlBus.move(ID_GRIPPER, targetGripper);
      } else {
        reachedGripper = true;
        Serial.println("GRIPPER llegó a HOME");
      }
    } else if (realPosGripper == -1) {
      Serial.println("ERROR: No se puede comunicar con GRIPPER");
      reachedGripper = true; // Marca como completado para evitar bucle infinito
    }

    delay(5); // Delay ajustable para suavidad del movimiento
  }

  Serial.println("Todos los motores llegaron a HOME.");
  
  // Lectura final para confirmar posiciones
  /*posBase     = dxlBus.readPosition(ID_BASE);
  posShoulder = dxlBus.readPosition(ID_SHOULDER);
  posShoulderB = dxlBus.readPosition(ID_SHOULDERB);
  posElbow    = dxlBus.readPosition(ID_ELBOW);
  posGripper  = dxlBus.readPosition(ID_GRIPPER);
  
  Serial.printf("Posiciones finales HOME: B:%d S:%d SB:%d E:%d G:%d\n",
                posBase, posShoulder, posShoulderB, posElbow, posGripper);*/
}


const int MIN_VOLTAGE = 75/10; // 7.5V representado en décimas

// Verifica si todos los motores tienen energía suficiente
bool checkMotorsPowered() {
  int vBase = dxlBus.readVoltage(ID_BASE);
  int vShoulder = dxlBus.readVoltage(ID_SHOULDER);
  int vShoulderB = dxlBus.readVoltage(ID_SHOULDERB);
  int vElbow = dxlBus.readVoltage(ID_ELBOW);
  int vGripper = dxlBus.readVoltage(ID_GRIPPER);

Serial.printf(
    "Voltajes: B:%d,S:%d.SB:%d.E:%d.G:%d \n",
    vBase / 10,
    vShoulder / 10,
    vShoulderB / 10,
    vElbow / 10,
    vGripper / 10
  );

 return (//vBase >= MIN_VOLTAGE && 
          vShoulder >= MIN_VOLTAGE && 
          vShoulderB >= MIN_VOLTAGE &&
          vElbow >= MIN_VOLTAGE && 
          vGripper >= MIN_VOLTAGE
          );

}
// ———————————————————————————————
void setup() {
  // Inicia el USB Serial para debug
  Serial.begin(115200);
  while (!Serial);

  // Inicia Bluetooth con nombre "ESP32_RosBT"
  SerialBT.begin("ESP32_RosBT");

  // Inicia el puerto Serial2 para hablar con los AX-12A
  Serial2.begin(DXL_BAUD, SERIAL_8N1, DXL2_RX, DXL2_TX);
  delay(10);
  pinMode(DXL_DIR_PIN, OUTPUT);
  dxlBus.begin(DXL_BAUD, DXL_DIR_PIN, &Serial2);

  // Deshabilita modo continuo para cada ID
  dxlBus.setEndless(ID_BASE,     OFF);
  dxlBus.setEndless(ID_SHOULDER, OFF);
  dxlBus.setEndless(ID_SHOULDERB, OFF);
  dxlBus.setEndless(ID_ELBOW,    OFF);
  dxlBus.setEndless(ID_GRIPPER,  OFF);
  delay(20);

  posBase     = dxlBus.readPosition(ID_BASE);
  posShoulder = dxlBus.readPosition(ID_SHOULDER);
  posShoulderB = dxlBus.readPosition(ID_SHOULDERB);
  posElbow    = dxlBus.readPosition(ID_ELBOW);
  posGripper  = dxlBus.readPosition(ID_GRIPPER);

  irAHOME();

  Serial.println("Setup Core0 OK: USB y BT inicializados, DXL listos.");

  // Crea la tarea en Core 1 que procesará los comandos entrantes
  xTaskCreatePinnedToCore(
    taskDXL,
    "DXL Task",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );


}

// ———————————————————————————————
// Loop principal: lee posición y envía feedback
// ———————————————————————————————
void loop() {
  // 1) Puente Serial USB → Bluetooth (reenvía comandos desde USB si hay)


  // 2) Lee datos del motor base (temperatura, voltaje, carga)
  tempBase  = dxlBus.readTemperature(ID_BASE);
  voltBase  = dxlBus.readVoltage(ID_BASE);  // décimas de voltio
  loadBase  = dxlBus.readLoad(ID_BASE);     // 0–1023

  // 3) Lee las posiciones crudas (0–1023)
  posBase     = dxlBus.readPosition(ID_BASE);
  posShoulder = dxlBus.readPosition(ID_SHOULDER);
  posShoulderB = dxlBus.readPosition(ID_SHOULDERB);
  posElbow    = dxlBus.readPosition(ID_ELBOW);
  posGripper  = dxlBus.readPosition(ID_GRIPPER);

  // 4) Convierte las posiciones a ángulos reales (0°–300°)
  int angBase     = posicionAangulo(posBase);
  int angShoulder = posicionAangulo(posShoulder);
  int angShoulderB = posicionAangulo(posShoulderB);
  int angElbow    = posicionAangulo(posElbow);
  int angGripper  = posicionAangulo(posGripper);
  
Serial.printf("%d,%d,%d,%d\n",
               angBase,
               angShoulder,
               angElbow,
               angGripper);

  delay(50);  // para evitar saturar el bus serial
}

// ———————————————————————————————
// Tarea Core 1: procesa comandos entrantes y mueve motores
// ———————————————————————————————
void taskDXL(void*) {
  for (;;) {
       
      
    while (Serial.available()) {


      char c = Serial.read();
      inputBuf += c;
      if (c == ',') {
        commaCount++;
        // Cuando recibimos 5 comas, tenemos b..,s..,e..,g..,
        if (commaCount >= 5) {
          int angles_deg[5] = {0, 0, 0, 0, 0};
          int start = 0;
          // Extrae cada token entre comas
          for (int t = 0; t < 5; ++t) {
            int comma = inputBuf.indexOf(',', start);
            if (comma < 0) break;
            String tok = inputBuf.substring(start, comma);
            start = comma + 1;
            // Primer carácter indica la articulación, el resto el valor
            if (tok.length() > 1) {
              char joint = tok.charAt(0);
              int value = tok.substring(1).toInt(); // 0–180
              switch (joint) {
                case 'b': angles_deg[0] = constrain(value, 0, 300); break;
                case 's': angles_deg[1] = constrain(value, 55, 245); break;
                case 'p': angles_deg[2] = constrain(value, 55, 245); break;
                case 'e': angles_deg[3] = constrain(value, 50, 250); break;
                case 'g': angles_deg[4] = constrain(value, 40, 260); break;
              }
            }
          }

          // Convierte ángulos 0–180 a 0–1023 para el motor
          int pos_base     = gradosAposicion(angles_deg[0]);
          int pos_shoulder = gradosAposicion(angles_deg[1]);
          int pos_shoulderB = gradosAposicion(angles_deg[2]);
          int pos_elbow    = gradosAposicion(angles_deg[3]);
          int pos_gripper  = gradosAposicion(angles_deg[4]);


   // Opcional: confirma por Bluetooth el comando recibido

          // Envía comando de movimiento a cada motor
          dxlBus.move(ID_BASE,     pos_base);
          dxlBus.move(ID_SHOULDER, pos_shoulder);
          dxlBus.move(ID_SHOULDERB, pos_shoulderB);
          dxlBus.move(ID_ELBOW,    pos_elbow);
          dxlBus.move(ID_GRIPPER,  pos_gripper);

          // Opcional: confirma por Bluetooth el comando recibido
       /*   SerialBT.printf(
            "DXL → b:%d° s:%d° p:%d° e:%d° g:%d°\n",
            angles_deg[0], angles_deg[1],
            angles_deg[2],angles_deg[3], angles_deg[4]
          );
             SerialBT.printf(
            "DXL → bM:%d sM:%d pM:%d eM:%d gM:%d \n",
            posBase , posShoulder,posShoulderB,
            posElbow,pos_elbow, pos_gripper
          );
*/

          // Reinicia buffer y contador
          inputBuf = "";
          commaCount = 0;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
