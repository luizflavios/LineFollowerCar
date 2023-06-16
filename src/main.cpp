
#include <Arduino.h>

// MOTOR DIREITO 93,3333% - MOTOR ESQUERDO 100%

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
int leftSquare = 0;
int black = 0;

// Métodos
boolean sensoresLateraisDesligados();
boolean sensoresLateraisLigados();
boolean todosSensoresDesligados();
boolean movimentoPerfeito();
boolean verificarTempoSensor(const int sensor, unsigned long tempoLimite);
void desligaMotores(unsigned long ms);
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
    analogWrite(leftMotorForwardPin, 150);
    analogWrite(rightMotorForwardPin, 140);
    lastMovement = moveForward; // Armazena o movimento de frente
    Serial.println("Movimento: Frente");
  }
  // Se o sensor esquerdo detectar que saiu da linha
  else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
  {
    // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 10);
    analogWrite(rightMotorForwardPin, 93);
    lastMovement = turnLeft; // Armazena o movimento para a esquerda
    Serial.println("Movimento: Esquerda");
  }
  // Se o sensor direito detectar que saiu da linha
  else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
  {
    // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 10);
    lastMovement = turnRight; // Armazena o movimento para a direita
    Serial.println("Movimento: Direita");
  }
  // Lógica da quadrado a direita
  else if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {
    Serial.println("IDENTIFICANDO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 93);
    square++;
    while (extremalRigthSensorState == lineDetected)
    {
      delay(2);
      leitura();
    }
    Serial.println(square);

    blockOnLoop++;

    while (blockOnLoop != 0)
    {
      Serial.println("PRESO NA LOGICA ROTATORIA OU CRUZAMENTO...");
      blockOnLoop = rotatoriaDireita(square, blockOnLoop);
    }

    square = 0;
  }
  // Lógica da quadrado a esquerda
  else if (leftSquare == 0 && middleSensorState == lineDetected && extremalLeftSensorState == lineDetected && extremalRigthSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {
    Serial.println("IDENTIFICANDO LINHA A ESQUERDA, REDUZINDO A VELOCIDADE...");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 93);
    blockOnLoop++;
    leftSquare++;
    delay(2000);
    leitura();

    while (leftSquare != 0)
    {

      leitura();

      // Se o sensor do meio detectar a linha
      if (middleSensorState == lineDetected && sensoresLateraisDesligados())
      {
        // Ambos os motores seguem em frente com velocidade máxima
        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 140);
        lastMovement = moveForward; // Armazena o movimento de frente
        Serial.println("Movimento: Frente");
        leitura();
      }
      // Se o sensor esquerdo detectar que saiu da linha
      else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
      {
        // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
        analogWrite(leftMotorForwardPin, 10);
        analogWrite(rightMotorForwardPin, 93);
        lastMovement = turnLeft; // Armazena o movimento para a esquerda
        Serial.println("Movimento: Esquerda");
        leitura();
      }
      // Se o sensor direito detectar que saiu da linha
      else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
      {
        // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
        analogWrite(leftMotorForwardPin, 100);
        analogWrite(rightMotorForwardPin, 10);
        lastMovement = turnRight; // Armazena o movimento para a direita
        Serial.println("Movimento: Direita");
        leitura();
      }
      else if (leftSquare != 0 && middleSensorState == lineDetected && extremalLeftSensorState == lineDetected && extremalRigthSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
      {
        while (blockOnLoop != 0)
        {
          Serial.println("PRESO NA LOGICA DO CRUZAMENTO ESQUERDA...");
          leitura();

          if ((middleSensorState == lineDetected && leftSensorState == lineDetected) && (extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected && rightSensorState == lineNotDetected))
          {
            Serial.println("IDENTIFICANDO LINHA DEPOIS DO QUADRADO ESQUEDO...");

            while (!movimentoPerfeito())
            {
              Serial.println("VIRANDO A ESQUERDA...");
              analogWrite(rightMotorForwardPin, 150);
              analogWrite(leftMotorForwardPin, 0);
              leitura();
            }

            analogWrite(leftMotorForwardPin, 150);
            analogWrite(rightMotorForwardPin, 140);
            blockOnLoop = 0;
            leftSquare = 0;
          }
        }
      }
    }
  }
  // Lógica da faixa de pedestre
  else if (middleSensorState == lineNotDetected && sensoresLateraisLigados())
  {
    Serial.println("Faixa de pedestre esta chegando, diminuindo a velocidade....");
    analogWrite(leftMotorForwardPin, 100);
    analogWrite(rightMotorForwardPin, 93);
    blockOnLoop++;
    black++;

    while (blockOnLoop != 0)
    {
      Serial.println("Preso na lógica da Faixa de pedestre....");

      while (!(middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected))
      {

        leitura();

        if (middleSensorState == lineNotDetected && sensoresLateraisLigados())
        {
          analogWrite(leftMotorForwardPin, 150);
          analogWrite(rightMotorForwardPin, 140);
          lastMovement = moveForward;
          Serial.println("Andando pra frente antes da faixa");
        }
        else if ((leftSensorState == lineNotDetected || (leftSensorState == lineNotDetected && middleSensorState == lineNotDetected)) && rightSensorState == lineDetected)
        {
          analogWrite(leftMotorForwardPin, 10);
          analogWrite(rightMotorForwardPin, 93);
          lastMovement = turnLeft;
          Serial.println("Corrigindo para esquerda antes da faixa");
        }
        else if ((rightSensorState == lineNotDetected || (rightSensorState == lineNotDetected && middleSensorState == lineNotDetected)) && leftSensorState == lineDetected)
        {
          analogWrite(leftMotorForwardPin, 100);
          analogWrite(rightMotorForwardPin, 10);
          lastMovement = turnRight;
          Serial.println("Corrigindo para direita antes da faixa");
        }
      }

      leitura();

      while (black != 0)
      {
        Serial.println("Sai do branco e entrei no preto...");
        leitura();

        // Se o sensor do meio detectar a linha
        if (middleSensorState == lineDetected && sensoresLateraisDesligados())
        {
          // Ambos os motores seguem em frente com velocidade máxima
          analogWrite(leftMotorForwardPin, 150);
          analogWrite(rightMotorForwardPin, 140);
          lastMovement = moveForward; // Armazena o movimento de frente
          Serial.println("Movimento: Frente");
        }
        // Se o sensor esquerdo detectar que saiu da linha
        else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
        {
          // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
          analogWrite(leftMotorForwardPin, 10);
          analogWrite(rightMotorForwardPin, 93);
          lastMovement = turnLeft; // Armazena o movimento para a esquerda
          Serial.println("Movimento: Esquerda");
        }
        // Se o sensor direito detectar que saiu da linha
        else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
        {
          // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
          analogWrite(leftMotorForwardPin, 100);
          analogWrite(rightMotorForwardPin, 10);
          lastMovement = turnRight; // Armazena o movimento para a direita
          Serial.println("Movimento: Direita");
        }
        else if (middleSensorState == lineNotDetected && sensoresLateraisDesligados())
        {
          Serial.println("Cheguei no preto... parando...");
          desligaMotores(5000);

          while (!movimentoPerfeito())
          {
            analogWrite(leftMotorForwardPin, 150);
            analogWrite(rightMotorForwardPin, 140);
            lastMovement = moveForward; // Armazena o movimento de frente
            Serial.println("Movimento: Frente");
            black = 0;
          }
        }
      }

      Serial.println("Ligando motores antes da faixa....");
      analogWrite(leftMotorForwardPin, 150);
      analogWrite(rightMotorForwardPin, 140);
      delay(3000); // TODO: AJUSTAR O TEMPO...
      leitura();

      while (middleSensorState == lineNotDetected && rightSensorState == lineNotDetected && leftSensorState == lineNotDetected)
      {
        Serial.println("ANDANDO NO PRETO.....");
        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 140);
        leitura();
      }

      Serial.println("SAIU DO PRETO.....");
      leitura();

      if (middleSensorState == lineDetected || rightSensorState == lineDetected || leftSensorState == lineDetected)
      {
        while (middleSensorState == lineDetected || rightSensorState == lineDetected || leftSensorState == lineDetected)
        {
          Serial.println("Faixa de pedestre chegou, acelerando....");
          analogWrite(leftMotorForwardPin, 150);
          analogWrite(rightMotorForwardPin, 140);
          leitura();
        }
        Serial.println("Saindo da faixa de pedestre....");
        lastMovement = moveForward;
        blockOnLoop = 0;
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
      analogWrite(rightMotorForwardPin, 140);
    }
    else if (lastMovement == turnLeft)
    {
      analogWrite(leftMotorForwardPin, 10);
      analogWrite(rightMotorForwardPin, 93);
    }
    else if (lastMovement == turnRight)
    {
      analogWrite(leftMotorForwardPin, 100);
      analogWrite(rightMotorForwardPin, 10);
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

void desligaMotores(unsigned long ms)
{
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  delay(ms);
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

    desligaMotores(1000);

    while (!movimentoPerfeito())
    {
      Serial.println("VIRANDO A DIREITA...");
      analogWrite(leftMotorForwardPin, 150);
      analogWrite(rightMotorForwardPin, 0);
      delay(300);
      leitura();
    }

    analogWrite(leftMotorForwardPin, 150);
    analogWrite(rightMotorForwardPin, ]);
    return 0;
  }
  else if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
  {

    Serial.println("IDENTIFICANDO SEGUNDO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
    square++;
    analogWrite(leftMotorForwardPin, 90);
    analogWrite(rightMotorForwardPin, 84);

    while (extremalRigthSensorState == lineDetected)
    {
      delay(2);
      leitura();
    }
    Serial.println(square);

    delay(2000);
    leitura();

    while (square == 2)
    {

      int contarDireita = 0;
      leitura();

      // Se o sensor do meio detectar a linha
      if (middleSensorState == lineDetected && sensoresLateraisDesligados())
      {
        // Ambos os motores seguem em frente com velocidade máxima
        analogWrite(leftMotorForwardPin, 150);
        analogWrite(rightMotorForwardPin, 140);
        lastMovement = moveForward; // Armazena o movimento de frente
        Serial.println("Movimento: Frente");
        leitura();
      }
      // Se o sensor esquerdo detectar que saiu da linha
      else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
      {
        // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
        analogWrite(leftMotorForwardPin, 10);
        analogWrite(rightMotorForwardPin, 93);
        lastMovement = turnLeft; // Armazena o movimento para a esquerda
        Serial.println("Movimento: Esquerda");

        leitura();
      }
      // Se o sensor direito detectar que saiu da linha
      else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
      {
        // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
        analogWrite(leftMotorForwardPin, 100);
        analogWrite(rightMotorForwardPin, 10);
        lastMovement = turnRight; // Armazena o movimento para a direita
        Serial.println("Movimento: Direita");

        leitura();
      }
      else if (middleSensorState == lineDetected && leftSensorState == lineDetected && rightSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && extremalRigthSensorState == lineNotDetected)
      {

        while (!movimentoPerfeito())
        {
          Serial.println("VIRANDO A DIREITA...");
          analogWrite(leftMotorForwardPin, 150);
          analogWrite(rightMotorForwardPin, 0);
          delay(300);
          contarDireita++;
          leitura();
        }

        while (contarDireita != 0)
        {
          Serial.println("TO ESPERANDO A SAIDA DA ROTATORIA...");
          leitura();
          // Se o sensor do meio detectar a linha
          if (middleSensorState == lineDetected && sensoresLateraisDesligados())
          {
            // Ambos os motores seguem em frente com velocidade máxima
            analogWrite(leftMotorForwardPin, 150);
            analogWrite(rightMotorForwardPin, 140);
            lastMovement = moveForward; // Armazena o movimento de frente
            Serial.println("Movimento: Frente");
            leitura();

            if (extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected)
            {
              Serial.println("INCREMENTANDO CONTAR DIREITA DURANTE Movimento: Frente");
              contarDireita++;
              Serial.println(contarDireita);
            }
          }
          // Se o sensor esquerdo detectar que saiu da linha
          else if ((leftSensorState == lineDetected || (leftSensorState == lineDetected && middleSensorState == lineDetected)) && rightSensorState == lineNotDetected)
          {
            // O motor esquerdo gira em sentido contrário ao dos motores com velocidade reduzida
            analogWrite(leftMotorForwardPin, 10);
            analogWrite(rightMotorForwardPin, 93);
            lastMovement = turnLeft; // Armazena o movimento para a esquerda
            Serial.println("Movimento: Esquerda");

            leitura();

            if (extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected)
            {
              Serial.println("INCREMENTANDO CONTAR DIREITA DURANTE Movimento: Esquerda");
              contarDireita++;
              Serial.println(contarDireita);
            }
          }
          // Se o sensor direito detectar que saiu da linha
          else if ((rightSensorState == lineDetected || (rightSensorState == lineDetected && middleSensorState == lineDetected)) && leftSensorState == lineNotDetected)
          {
            // O motor direito gira em sentido contrário ao dos motores com velocidade reduzida
            analogWrite(leftMotorForwardPin, 100);
            analogWrite(rightMotorForwardPin, 10);
            lastMovement = turnRight; // Armazena o movimento para a direita
            Serial.println("Movimento: Direita");

            leitura();

            if (extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected)
            {
              Serial.println("INCREMENTANDO CONTAR DIREITA DURANTE Movimento: Direitas");
              contarDireita++;
              Serial.println(contarDireita);
            }
          }
          else if (contarDireita == 3)
          {
            while (!movimentoPerfeito())
            {
              Serial.println("VIRANDO A DIREITA...");
              analogWrite(leftMotorForwardPin, 150);
              analogWrite(rightMotorForwardPin, 0);
              delay(300);
              contarDireita = 0;
              leitura();
            }
          }
        }
      }
    }

    if (middleSensorState == lineDetected && extremalRigthSensorState == lineDetected && extremalLeftSensorState == lineNotDetected && leftSensorState == lineNotDetected && rightSensorState == lineNotDetected)
    {

      Serial.println("IDENTIFICANDO TERCEIRO QUADRADO A DIREITA, REDUZINDO A VELOCIDADE...");
      square++;

      while (extremalRigthSensorState == lineDetected)
      {
        delay(2);
        leitura();
      }
      Serial.println(square);

      analogWrite(leftMotorForwardPin, 90);
      analogWrite(rightMotorForwardPin, 84);
    }
    // else if (movimentoPerfeito())
    // {
    //   return 0;
    // }

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