// Wifi  -> #define S7WIFI
// Cable -> #define S7WIRED
#define S7WIRED

#include <SPI.h>
#include <Ethernet.h>
#include "Settimino.h"

// Remova o comentário da próxima linha para realizar acesso a dados pequeno e rápido
#define DO_IT_EXTENDED



// Digite um endereço MAC e um endereço IP para o seu controlador abaixo.
// O endereço IP dependerá da sua rede local:
byte mac[] = { 
  0x28, 0x63, 0x36, 0x8E, 0x76, 0xE3 }; 

IPAddress Local(192,168,0,9); // Local Address
IPAddress PLC(192,168,0,34);   // PLC Address


// As seguintes constantes são necessárias se você estiver se conectando via WIFI
// O ssid é o nome da minha rede WIFI (a senha está obviamente errada)
/* char ssid [] = "SKYNET-AIR"; // O SSID da sua rede (nome)
char pass [] = "sua senha"; // Sua senha de rede (se houver) */
IPAddress Gateway(192, 168, 0, 1);
IPAddress Subnet(255, 255, 255, 0);

int DBNum = 1; // 
int Buffer[1024]; // 

S7Client Client(_S7WIRED);


const int pinoSensor = A2; //PINO ANALÓGICO EM QUE O SENSOR ESTÁ CONECTADO

float tensaoEntrada = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DE TENSÃO DE ENTRADA DO SENSOR
float tensaoMedida = 0.0; //VARIÁVEL PARA ARMAZENAR O VALOR DA TENSÃO MEDIDA PELO SENSOR

float valorR1 = 30000.0; //VALOR DO RESISTOR 1 DO DIVISOR DE TENSÃO
float valorR2 = 7500.0; // VALOR DO RESISTOR 2 DO DIVISOR DE TENSÃO
int leituraSensor = 0; //VARIÁVEL PARA ARMAZENAR A LEITURA DO PINO ANALÓGICO

unsigned long Elapsed; // Para calcular o tempo de execução
// ------------------------------------------------ ----------------------
// Configuração: Init Ethernet e Serial port
// ------------------------------------------------ ----------------------
void setup() {
    // Abra a comunicação serial e aguarde a porta abrir:
    analogReference(EXTERNAL);
    pinMode(pinoSensor, INPUT); //DEFINE O PINO COMO ENTRADA
    Serial.begin(9600);

    
     while (!Serial) {
      ; // aguarde a conexão da porta serial. Necessário apenas para continuar
    }

    // Inicie a biblioteca Ethernet
    Ethernet.begin(mac, Local);
    // Hora da instalação, alguém me disse para sair de 2000 porque alguns
    // placas compatíveis com o lixo são um pouco surdas.
    delay(2000); 
    Serial.println("");
    Serial.println("Cabo conectado");  
    Serial.print("Endereço IP local : ");
    Serial.println(Ethernet.localIP());

     
}
// ------------------------------------------------ ----------------------
// Conecta ao CLP
// ------------------------------------------------ ----------------------
bool Connect()
{
    int Result=Client.ConnectTo(PLC, 
                                  0,  // Rack (ver o doc.)
                                  2); // Slot (ver o doc.)
    Serial.print("Conectando à ");Serial.println(PLC);  
    
    if (Result==0) 
    {
      Serial.print("Conectado ! Comprimento da PDU = ");Serial.println(Client.GetPDULength());
    }
    else
      Serial.println("Erro de conexão");
    return Result==0;

      
}//----------------------------------------------------------------------
// Dumps a buffer (a very rough routine)
//----------------------------------------------------------------------
void Dump(void *Buffer, int Length)
{
  int i, cnt=0;
  pbyte buf;
  
  if (Buffer!=NULL)
    buf = pbyte(Buffer);
  else  
    buf = pbyte(&PDU.DATA[0]);

 // Serial.print("[ Dumping ");Serial.print(Length);
 // Serial.println(" bytes ]===========================");
  for (i=0; i<Length; i++)
  {
    cnt++;
    if (buf[i]<0x10)
   //   Serial.print("0");
   // Serial.print(buf[i], HEX);
    //Serial.print(" ");
    if (cnt==16)
    {
      cnt=0;
     // Serial.println();
    }
  }  
 // Serial.println("===============================================");
}

// ------------------------------------------------ ----------------------
// Imprime o número do erro
// ------------------------------------------------ ----------------------
void CheckError(int ErrNo)
{
  Serial.print("Error No. 0x");
  Serial.println(ErrNo, HEX);

  // Verifica se é um erro grave => precisamos desconectar
  if (ErrNo & 0x00FF)
  {
    Serial.println("ERRO GRAVE, desconectando.");
    Client.Disconnect(); 
  }
}
// ------------------------------------------------ ----------------------
// Rotinas de criação de perfil
// ------------------------------------------------ ----------------------
void MarkTime()
{
  Elapsed=millis();
}
//----------------------------------------------------------------------
void ShowTime()
{
  // Calcs the time
  Elapsed=millis()-Elapsed;
 // Serial.print("Tempo de trabalho (ms) : ");
  //Serial.println(Elapsed);   
}
// ------------------------------------------------ ----------------------
// Loop principal
// ------------------------------------------------ ----------------------
void loop() 
{

   int Size, Result,test;
  void *Target;
   Size=64;
  Target = NULL;  // Uses the internal Buffer (PDU.DATA[])

  
  leituraSensor = analogRead(pinoSensor); //FAZ A LEITURA DO PINO ANALÓGICO E ARMAZENA NA VARIÁVEL O VALOR LIDO
   tensaoEntrada = (leituraSensor * 5.0) / 1024.0; //VARIÁVEL RECEBE O RESULTADO DO CÁLCULO
   tensaoMedida = tensaoEntrada / (valorR2/(valorR1+valorR2)); //VARIÁVEL RECEBE O VALOR DE TENSÃO DC MEDIDA PELO SENSOR
   tensaoMedida = tensaoMedida*0.039;
   // Connection
  while (!Client.Connected)
  {
    if (!Connect())
      delay(500);
  }
  MarkTime();
  //tensaoMedida;
 test=dword(leituraSensor);

  // Serial.print("Leitura do sensor : ");
 //  Serial.println(leituraSensor);   
 //  Serial.print("Leitura do sensor TENSÃO : ");
 //  Serial.println(tensaoMedida);
   
      Result=Client.WriteArea(S7AreaDB, // Estamos solicitando acesso ao banco de dados
                         DBNum,    // Número do banco de dados
                         1,     // Iniciar a partir do byte N.0
                         2,     // Precisamos de bytes "Size"
                         &leituraSensor);  // Coloque-os em nosso destino (Buffer ou PDU)

            
   if (Result==0)
  {
    ShowTime();
    Dump(Target, Size);
  }
  else
    CheckError(Result);
    
  delay(500);  
  
}
