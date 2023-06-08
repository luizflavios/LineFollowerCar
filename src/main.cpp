
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

// Métodos
boolean sensoresLateraisDesligados();
boolean sensoresLateraisLigados();
boolean todosSensoresDesligados();
boolean movimentoPerfeito();
boolean verificarTempoSensor(const int sensor, unsigned long tempoLimite);
void desligaMotores();
void diminuirVelocidade();
void leitura();
void imprimirSensor();
int rotatoriaDireita(int square, int blockOnLoop);

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

  int square = 0;
  int blockOnLoop = 0;
  leitura();

  // Se o sensor do meio detectar a linha
  if (middleSensorState == lineDetected && sensoresLateraisDesligados())
  {
    // Ambos os motores seguem em frente com velocidade máxima
    analogWrite(leftMotorForwardPin, 135);
    analogWrite(rightMotorForwardPin, 150);
    lastMovement = moveForward; // Armazena o movimento de frente
    Serial.println("Movimento: Frente");
  }
  // Se o sensor esquerdo detectar que saiu da linha
  else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
  {
    // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 230);
    lastMovement = turnLeft; // Armazena o movimento para a esquerda
    Serial.println("Movimento: Esquerda");
  }
  // Se o sensor direito detectar que saiu da linha
  else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
  {
    // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 230);
    analogWrite(rightMotorForwardPin, 100);
    lastMovement = turnRight; // Armazena o movimento para a direita
    Serial.println("Movimento: Direita");
  }
  // Lógica da quadrado a direita
  else if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {
    Serial.println("IDENTIFICANDO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 100);
    square++;
    blockOnLoop++;

    while (blockOnLoop != 0)
    {
      Serial.println("PRESO NA LOGICA ROTATORIA OU CRUZAMENTO...");
      blockOnLoop = rotatoriaDireita(square, blockOnLoop);
    }

    square = 0;
  }
  // Lógica da quadrado a esquerda
  else if (middleSensorState == lineDetected && extremalLeftSensorState == lineDetected && extremalRigthSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {
    Serial.println("IDENTIFICANDO QUADRADO A ESQUERDA, REDUZINDO A VELOCIDADE...");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 100);
    blockOnLoop++;

    while (blockOnLoop != 0)
    {
      Serial.println("PRESO NA LOGICA DO CRUZAMENTO ESQUERDA...");
      leitura();

      if ((middleSensorState == lineDetected && leftSensorState == lineDetected) && (extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected && rightSensorState == lineNotDetected))
      {
        Serial.println("IDENTIFICANDO LINHA DEPOIS DO QUADRADO ESQUEDO...");

        desligaMotores();

        while (!movimentoPerfeito())
        {
          Serial.println("VIRANDO A ESQUERDA...");
          analogWrite(rightMotorForwardPin, 150);
          analogWrite(leftMotorForwardPin, 0);
          leitura();
        }

        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 150);
        blockOnLoop = 0;
      }
    }
  }
  // Lógica da faixa de pedestre
  else if (middleSensorState == lineNotDetected && sensoresLateraisLigados())
  {
    Serial.println("Faixa de pedestre esta chegando, diminuindo a velocidade....");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 100);
    blockOnLoop++;

    while (blockOnLoop != 0)
    {
      Serial.println("Preso na lógica da Faixa de pedestre....");
      leitura();

      if ((middleSensorState == lineDetected && rightSensorState == lineDetected && leftSensorState == lineDetected) || middleSensorState == lineDetected || rightSensorState == lineDetected || leftSensorState == lineDetected)
      {
        Serial.println("Faixa de pedestre chegou, parando....");
        desligaMotores();
        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 150);
        Serial.println("Saindo da faixa de pedestre....");
        blockOnLoop++;
      }
    }
  }
  // Se nenhum dos sensores detectar a linha
  else
  {
    // Executa o último movimento realizado
    if (lastMovement == moveForward)
    {
      analogWrite(leftMotorForwardPin, 150);
      analogWrite(rightMotorForwardPin, 150);
    }
    else if (lastMovement == turnLeft)
    {
      analogWrite(leftMotorForwardPin, 100);
      analogWrite(rightMotorForwardPin, 230);
    }
    else if (lastMovement == turnRight)
    {
      analogWrite(leftMotorForwardPin, 230);
      analogWrite(rightMotorForwardPin, 100);
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

boolean sensoresLateraisLigados()
{
  return rightSensorState == lineDetected && leftSensorState == lineDetected && extremalLeftSensorState == lineDetected && extremalRigthSensorState == lineDetected;
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
  delay(3000);
}

void diminuirVelocidade()
{
  analogWrite(leftMotorForwardPin, 90);
  analogWrite(rightMotorForwardPin, 90);
}

void leitura()
{
  // Ler os estados dos sensores
  middleSensorState = digitalRead(middleSensorPin);
  leftSensorState = digitalRead(leftSensorPin);
  rightSensorState = digitalRead(rightSensorPin);
  extremalLeftSensorState = digitalRead(extremalLeftSensorPin);
  extremalRigthSensorState = digitalRead(extremalRigthSensorPin);
}

void imprimirSensor()
{
  Serial.println(extremalLeftSensorState);
  Serial.println(leftSensorState);
  Serial.println(middleSensorState);
  Serial.println(rightSensorState);
  Serial.println(extremalRigthSensorState);
  Serial.println("     =====    ");
}

int rotatoriaDireita(int square, int blockOnLoop)
{
  leitura();

  if (((middleSensorState == lineDetected && rightSensorState == lineDetected && leftSensorState == lineDetected) || (middleSensorState == lineDetected && rightSensorState == lineDetected) || (middleSensorState == lineDetected && leftSensorState == lineDetected)) && (extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected))
  {
    Serial.println("IDENTIFICANDO LINHA DEPOIS DO QUADRADO...");

    desligaMotores();

    while (!movimentoPerfeito())
    {
      Serial.println("VIRANDO A DIREITA...");
      analogWrite(leftMotorForwardPin, 150);
      analogWrite(rightMotorForwardPin, 0);
      leitura();
    }

    analogWrite(leftMotorForwardPin, 150);
    analogWrite(rightMotorForwardPin, 150);
    return 0;
  }
  else if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {

    Serial.println("IDENTIFICANDO SEGUNDO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
    square++;

    analogWrite(leftMotorForwardPin, 90);
    analogWrite(rightMotorForwardPin, 90);

    if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
    {

      Serial.println("IDENTIFICANDO TERCEIRO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
      square++;
      analogWrite(leftMotorForwardPin, 90);
      analogWrite(rightMotorForwardPin, 90);

      if (square == 3 && middleSensorState == lineDetected && rightSensorState == lineDetected && leftSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected)
      {
        Serial.println("IDENTIFICANDO LINHA DEPOIS DO TERCEIRO QUADRADO...");

        while (!movimentoPerfeito())
        {
          Serial.println("VIRANDO A DIREITA...");
          analogWrite(leftMotorForwardPin, 150);
          analogWrite(rightMotorForwardPin, 0);
          leitura();
        }

        square = 0;
        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 150);
        return 0;
      }
    }
    else if (movimentoPerfeito())
    {
      return 0;
    }

    return 1;
  }
  return 1;
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