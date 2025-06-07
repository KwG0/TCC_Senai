#include "Tiny4kOLED.h"
#include "DHT.h"
#include "Servo.h"

#define TipoDHT DHT22
#define SensorUmidade A0
#define SensorLDR A3

const int servoPino = 9;
const int bomba = 8;
int porcentagem = 0;
int LDR = 0;
const float GAMMA = 0.7;
const float RL10 = 50;
unsigned long tempoAnterior = 0;
const unsigned long intervalo = 1000;

DHT dht(SensorUmidade, TipoDHT);


void inicio() {
  oled.clear();
  oled.setCursor(42, 1);
  oled.print("U.A.P.P");
  oled.setCursor(47, 3);
  oled.print("DREAM");
}

void Servo(int angulo) {
  int PWM = map(angulo, 0, 180, 544, 2400);
  digitalWrite(servoPino, HIGH);
  delayMicroseconds(PWM);
  digitalWrite(servoPino, LOW);
  delay(20 - PWM / 1000);
}

void setup() {
  Serial.begin(9600);
  pinMode(SensorUmidade, INPUT);
  pinMode(bomba, OUTPUT);
  pinMode(servoPino, OUTPUT);

  // Inicializa o OLED
  oled.begin(128, 64, sizeof(tiny4koled_init_128x64br), tiny4koled_init_128x64br);
  oled.setFont(FONT6X8);
  oled.clear();
  oled.on();

  inicio();
  dht.begin();
  delay(3000);
}

// Movimento do servo para sol pleno
void Sol() {
  for (int angulo = 0; angulo <= 180; angulo++) {
    Servo(angulo);
    delay(100);
  }
  delay(2000);

  for (int angulo = 180; angulo >= 90; angulo--) {
    Servo(angulo);
    delay(100);
  }
  delay(2000);
}

// Movimento do servo para meia-luz
void meiaLuz() {
  for (int angulo = 90; angulo >= 30; angulo--) {
    Servo(angulo);
    delay(50);
  }
  delay(2000);
  for (int angulo = 30; angulo <= 80; angulo++) {
    Servo(angulo);
    delay(50);
  }
  delay(2000);
  for (int angulo = 80; angulo <= 110; angulo++) {
    Servo(angulo);
    delay(50);
  }
  delay(2000);
}

void Rega() {
  unsigned long tempoAtual = millis();

  if (tempoAtual - tempoAnterior >= intervalo) {
    tempoAnterior = tempoAtual;

    if (porcentagem < 50) {      // Solo seco
      digitalWrite(bomba, LOW);  // Liga a bomba
      oled.setCursor(5, 3);
      oled.print("Regando...");
      delay(3000);
    } else if (porcentagem > 50) {
      digitalWrite(bomba, HIGH);  // Desliga a bomba
    }
  }
}

void loop() {
  // Leitura da umidade do solo
  int leituraUmidade = analogRead(SensorUmidade);
  porcentagem = map(leituraUmidade, 1023, 0, 0, 100);

  // Leitura do LDR (CORRIGIDA)
  LDR = analogRead(SensorLDR);
  int tensao = (LDR / 1024) * 5.0;
  int resistenciaLDR = 10000.0 * tensao / (1 - tensao / 5);  // Calcula resistência do LDR (assumindo resistor de 10kΩ)
  int luminosidade = pow(RL10 * 1e3 * pow(10, GAMMA) / resistenciaLDR, 1.0 / GAMMA);

  // Exibição no Serial Monitor (DEBUG)
  Serial.print("LDR Raw: ");
  Serial.print(LDR);
  Serial.print(" | Tensão: ");
  Serial.print(tensao, 2);
  Serial.print("V | R_LDR: ");
  Serial.print(resistenciaLDR / 1000.0, 1);  // Em kΩ
  Serial.print("kΩ | Lux: ");
  Serial.print(luminosidade, 1);
  Serial.print(" | Umidade: ");
  Serial.print(porcentagem);
  Serial.println("%");

  // Exibição no OLED
  oled.clear();
  oled.setCursor(3, 2);
  oled.print("Umidade = ");
  oled.print(porcentagem);
  oled.print("%");
  Rega();


  // Controle baseado na luminosidade (CONDIÇÕES CORRIGIDAS)
  if (resistenciaLDR > 65000) {  // Muita resistência = pouca luz
    oled.setCursor(3, 5);
    oled.print("Captacao Ruim!");
    meiaLuz();
    delay(3000);
  } else if (resistenciaLDR <= 60000 && resistenciaLDR > 5000) {  // Luz moderada
    oled.setCursor(3, 5);
    oled.print("Captacao Mediana!");
    delay(3000);
  } else if (resistenciaLDR <= 5000) {  // Pouca resistência = muita luz
    oled.setCursor(3, 5);
    oled.print("Captacao Boa!");
    delay(3000);
    Sol();
  }
  delay(5000);
}