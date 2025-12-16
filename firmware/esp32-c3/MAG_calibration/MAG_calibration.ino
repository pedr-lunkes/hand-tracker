#include <MPU9250_WE.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// Variáveis para armazenar extremos
float magMinX = 1000.0, magMaxX = -1000.0;
float magMinY = 1000.0, magMaxY = -1000.0;
float magMinZ = 1000.0, magMaxZ = -1000.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if(!myMPU9250.init()){
    Serial.println("MPU9250 não respondeu!");
    while(1);
  }
  
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetômetro não respondeu!");
    while(1);
  }
  
  // Configuração igual ao seu código BLE para garantir consistência
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  
  Serial.println("========================================");
  Serial.println("   CALIBRAÇÃO MANUAL DO MAGNETÔMETRO    ");
  Serial.println("========================================");
  Serial.println("Gire o sensor em formato de '8' em TODAS as direções.");
  Serial.println("Os valores vão convergir conforme você cobre todos os ângulos.");
  delay(2000);
}

void loop() {
  // Lê os dados do magnetômetro
  xyzFloat mag = myMPU9250.getMagValues();
  
  // Atualiza Mínimos e Máximos X
  if (mag.x < magMinX) magMinX = mag.x;
  if (mag.x > magMaxX) magMaxX = mag.x;
  
  // Atualiza Mínimos e Máximos Y
  if (mag.y < magMinY) magMinY = mag.y;
  if (mag.y > magMaxY) magMaxY = mag.y;
  
  // Atualiza Mínimos e Máximos Z
  if (mag.z < magMinZ) magMinZ = mag.z;
  if (mag.z > magMaxZ) magMaxZ = mag.z;

  // --- CÁLCULOS (Hard Iron e Soft Iron) ---
  
  // 1. Hard Iron (Bias) = Média entre Min e Max
  // Isso centraliza a esfera no (0,0,0)
  float biasX = (magMaxX + magMinX) / 2.0;
  float biasY = (magMaxY + magMinY) / 2.0;
  float biasZ = (magMaxZ + magMinZ) / 2.0;

  // 2. Soft Iron (Scale) = Corrige a elipticidade para virar uma esfera
  // Calculamos a "corda" (distância min-max) de cada eixo
  float chordX = (magMaxX - magMinX) / 2.0;
  float chordY = (magMaxY - magMinY) / 2.0;
  float chordZ = (magMaxZ - magMinZ) / 2.0;

  // Média das cordas
  float avgChord = (chordX + chordY + chordZ) / 3.0;

  // Fator de escala
  float scaleX = avgChord / chordX;
  float scaleY = avgChord / chordY;
  float scaleZ = avgChord / chordZ;

  // Evita divisão por zero no início
  if (chordX == 0) scaleX = 1.0;
  if (chordY == 0) scaleY = 1.0;
  if (chordZ == 0) scaleZ = 1.0;

  // --- SAÍDA FORMATADA ---
  // Imprime a cada 500ms para não poluir demais
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.println("\n--- Copie estes valores para ekf_config_real.yaml ---");
    
    Serial.print("magnetometer:\n");
    
    Serial.print("  bias: [");
    Serial.print(biasX); Serial.print(", ");
    Serial.print(biasY); Serial.print(", ");
    Serial.print(biasZ); Serial.println("]");
    
    Serial.print("  scale: [");
    Serial.print(scaleX); Serial.print(", ");
    Serial.print(scaleY); Serial.print(", ");
    Serial.print(scaleZ); Serial.println("]");
    
    Serial.print("Status: (Min/Max X: "); Serial.print(magMinX); Serial.print("/"); Serial.print(magMaxX); Serial.println(")");
    
    lastPrint = millis();
  }
  
  delay(10);
}