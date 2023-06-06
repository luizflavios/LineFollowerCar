
#include <Arduino.h>

// Definir os pinos do Arduino
const int leftMotorForwardPin = 9;
const int leftMotorBackwardPin = 6;
const int rightMotorForwardPin = 5;
const int rightMotorBackwardPin = 3;
const int extremalRigthSensorPin = 8;
const int extremalLeftSensorPin = 10;
const int middleSensorPin = 12;
const int leftSensorPin = 13;
const int rightSensorPin = 11;

// Estado dos sensores
bool middleSensorState = HIGH;
bool leftSensorState = HIGH;
bool rightSensorState = HIGH;
bool extremalRigthSensorState = HIGH;
bool extremalLeftSensorState = HIGH;

// Definir os valores dos estados dos sensores
const bool lineDetected = LOW;
const bool lineNotDetected = HIGH;

// Definir os estados de movimento
const int moveForward = 0;
const int turnLeft = 1;
const int turnRight = 2;

// Variável para armazenar o último movimento realizado
int lastMovement = moveForward;
int work = 0;

// Métodos
boolean sensoresLateraisDesligados();
boolean todosSensoresDesligados();
boolean movimentoPerfeito();
boolean verificarTempoSensor(const int sensor, unsigned long tempoLimite);
void desligaMotores();
void diminuirVelocidade();

void setup()
{
  // Configurar os pinos dos motores como saídas
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);

  // Iniciar a comunicação serial
  Serial.begin(9600);
}

void loop()
{
  // Ler os estados dos sensores
  middleSensorState = digitalRead(middleSensorPin);
  leftSensorState = digitalRead(leftSensorPin);
  rightSensorState = digitalRead(rightSensorPin);
  extremalLeftSensorState = digitalRead(extremalLeftSensorPin);
  extremalRigthSensorState = digitalRead(extremalRigthSensorPin);

  // Se o sensor do meio detectar a linha
  if (middleSensorState == lineDetected && sensoresLateraisDesligados())
  {
    // Ambos os motores seguem em frente com velocidade máxima
    analogWrite(leftMotorForwardPin, 180);
    analogWrite(rightMotorForwardPin, 180);
    lastMovement = moveForward; // Armazena o movimento de frente
    Serial.println("Movimento: Frente");
  }
  // Se o sensor esquerdo detectar que saiu da linha
  else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) 
          && rightSensorState == lineNotDetected)
  {
    // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 80);
    analogWrite(rightMotorForwardPin, 180);
    lastMovement = turnLeft; // Armazena o movimento para a esquerda
    Serial.println("Movimento: Esquerda");
  }
  // Se o sensor direito detectar que saiu da linha
  else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) 
          && (leftSensorState == lineNotDetected && work == 0))
  {
    // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 180);
    analogWrite(rightMotorForwardPin, 80);
    lastMovement = turnRight; // Armazena o movimento para a direita
    Serial.println("Movimento: Direita");
    Serial.println(middleSensorState);
    Serial.println(rightSensorState);
    Serial.println(leftSensorState);
    Serial.println(extremalLeftSensorState);
    Serial.println(extremalRigthSensorState);
  }
  else if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected 
            && extremalLeftSensorState == lineNotDetected 
            && leftSensorState == lineNotDetected 
            && rightSensorState == lineNotDetected)
  {

    Serial.println("IDENTIFICANDO QUADRADO...");

    Serial.println(middleSensorState);
    Serial.println(rightSensorState);
    Serial.println(leftSensorState);
    Serial.println(extremalLeftSensorState);
    Serial.println(extremalRigthSensorState);

    if (middleSensorState == 0 && rightSensorState == 1 && leftSensorState == 1)
    {
      Serial.println("IDENTIFICANDO LINHA DEPOIS DO QUADRADO...");

      while (!movimentoPerfeito())
      {
        Serial.println("VIRANDO A DIREITA...");
        analogWrite(leftMotorForwardPin, 80);
      }
    }
    Serial.println("NAO IDENFITIFICOU LINHA DEPOIS DO QUADRADO...");
  }
  // Se nenhum dos sensores detectar a linha
  else
  {
    // Executa o último movimento realizado
    if (lastMovement == moveForward)
    {
      analogWrite(leftMotorForwardPin, 180);
      analogWrite(rightMotorForwardPin, 180);
    }
    else if (lastMovement == turnLeft)
    {
      analogWrite(leftMotorForwardPin, 80);
      analogWrite(rightMotorForwardPin, 180);
    }
    else if (lastMovement == turnRight)
    {
      analogWrite(leftMotorForwardPin, 180);
      analogWrite(rightMotorForwardPin, 80);
    }
    else
    {
      digitalWrite(leftMotorForwardPin, LOW);
      digitalWrite(leftMotorBackwardPin, LOW);
      digitalWrite(rightMotorForwardPin, LOW);
      digitalWrite(rightMotorBackwardPin, LOW);
    }
  }
}

boolean sensoresLateraisDesligados()
{
  return rightSensorState == lineNotDetected && leftSensorState == lineNotDetected && extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected;
}

boolean todosSensoresDesligados()
{
  return sensoresLateraisDesligados() && middleSensorState == lineNotDetected;
}

boolean movimentoPerfeito()
{
  return middleSensorState == lineDetected && sensoresLateraisDesligados();
}

void desligaMotores()
{
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
}

void diminuirVelocidade()
{
  analogWrite(leftMotorForwardPin, 50);
  analogWrite(rightMotorForwardPin, 50);
}

boolean verificarTempoSensor(const int sensor, unsigned long tempoLimite)
{
  // Faça a leitura do sensor
  int leituraSensor = digitalRead(sensor);

  // Verifique se o sensor ficou ligado pelo tempo informado
  if (leituraSensor == 1)
  {
    Serial.println("Entrou 1");
    unsigned long tempoInicial = millis();
    unsigned long tempoDecorrido = 0;

    // Aguarde até que o tempo decorrido atinja o tempo limite
    while (leituraSensor != 0)
    {

      Serial.println(digitalRead(sensor));

      if (leituraSensor == 0)
      {
        // Sensor desligado antes de atingir o tempo limite
        return false;
      }

      tempoDecorrido = millis() - tempoInicial;
    }

    tempoInicial = 0;
    tempoDecorrido = 0;
    // Tempo limite atingido, o sensor ficou ligado pelo tempo informado
    return true;
  }

  // O sensor não estava ligado no início
  return false;
}