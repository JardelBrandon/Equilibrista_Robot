#include <PID_v1.h> // Inclui biblioteca do PID
#include <LMotorController.h> // Inclui biblioteca do controlador de motores DC
#include "I2Cdev.h"  // Inclui a biblioteca I2C (Protocolo de comunicação usado pelo MPU6050)

#include "MPU6050_6Axis_MotionApps20.h" // Inclui a biblioteca MPU6050_6Axis_MotionApps20 (Configuração do modo escolhido para setup no MPU6050)


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //Implementa se necessário uma comunicação para o arduino com o protocolo I2C incluindo-se a biblioteca Wire
    #include "Wire.h" // Inclui biblioteca Wire (Comunicação do arduino com o protocolo I2C)
#endif //Fim da macro condição 


#define LOG_INPUT 0 // Define a constante LOG_INPUT, se for diferente de 0 imprime no serial os valores obtidos pelo giroscópio; se for 0 não imprime 
#define MANUAL_TUNING 0 // Define a constante MANUAL_TUNING, se for diferente de 0 a configuração do PID se dar por forma manual (Através dos potenciômetros); se for 0 usa o PID pré-definido  
#define LOG_PID_CONSTANTS 0 //Define a constante LOG_PID_CONSTANTS, se for diferente de 0 imprime no serial os valores manuiais obtidos de Kp, Ki e Kd; se for 0 não imprime  
#define MIN_ABS_SPEED 30 // Define a constante MIN_ABS_SPEED, que será o menor valor de velocidade para os motores 

// MPU


MPU6050 mpu; //Inclui as funções do MPU6050

// MPU controle/status das variáveis
bool dmpReady = false;  // Verifica se a conexão com o DMP (Digital Motion Processor) no MPU6050 foi realizada com sucesso, se sim recebe True
uint8_t mpuIntStatus;   // Mantém o byte atual do estado de interrupção do MPU6050
uint8_t devStatus;      // Retorno após cada operação do dispositivo com a comunicação I2Cdev (se for igual a 0 sucesso, se for diferente de 0 erro)
uint16_t packetSize;    // Tamanho do pacote DMP esperado (o padrão é 42 bytes)
uint16_t fifoCount;     // Contagem de todos os bytes atualmente no FIFO (acrônimo para First In, First Out, que em português significa primeiro a entrar, primeiro a sair) 
uint8_t fifoBuffer[64]; // Buffer de armazenamento do FIFO

// Orientação/Movimento das variáveis 
Quaternion q;           // [w, x, y, z]         Recipiente quaternion
VectorFloat gravity;    // [x, y, z]            Vetor gravidade
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll Recipiente e vetor gravidade (ypr[3] é igual a 3 pois usamos o valor de roll)



// PID

#if MANUAL_TUNING // Macro condição, se a constante MANUAL-TUNING for diferente de 0 realiza os comandos do encadeamento abaixo
  double kp , ki, kd; // Define kp, ki e kd como sendo do tipo double 
  double prevKp, prevKi, prevKd; // Define prevKp, prevKi e prevKd como sendo do tipo double  
#endif // Encerra a Macro condição 

double originalSetpoint = 2; // Define originalSetpoint como sendo do tipo double (Esse valor é o ângulo desejado para o robô) 
double setpoint = originalSetpoint; // Define setpoint como sqendo do tipo double e igual ao originalSetpoint (Para efeito de cálculos no PID)
double input, output; // Define input e output como sendo do tipo double (Valores de entrada e saída do cálculo PID)
int moveState=0; // Define moveState como sendo do tipo int (Inteiros) função do movimento do robô, se for 0 = balanceado; 1 = para trás; 2 = frente

#if MANUAL_TUNING // Macro condição, se a constante MANUAL_TUNING for diferente de 0 realiza o comando abaixo (Ajuste do PID de forma Manual com potenciômetros)
  PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT); // Define a função do PID com os valores informados para o input, output, setpoint, Kp, Ki, Kd e o sentido de direção do controle 
#else // Macro condição, se não for diferente de 0 realiza o comando abaixo (Ajuste pré-definido do PID, "forma Automática")
  PID pid(&input, &output, &setpoint, 30, 350, 1.7, DIRECT); // Define a função do PID com os valores informados para o input, output, setpoint, Kp, Ki, Kd e o sentido de direção do controle
#endif  // Encerra a Macro condição
//Obs: Valores Manuais para o PID melhores aceitos no nosso robô KP = 21 KI = 47 KD = 0.4  
//Atualizacao KP = 30, KI = 350, KD = 1.7

// Controle do MOTOR 


int ENA = 9; // Pino de Saída do controle PWM do motor Esquerdo 
int IN1 = 5; // Pino de Saída da ligação do motor Esquerdo, HIGH ou LOW 
int IN2 = 8; // Pino de Saída da ligação do motor Esquerdo, HIGH ou LOW 
int IN3 = 6; // Pino de Saída da ligação do motor Direito, HIGH ou LOW 
int IN4 = 12; // Pino de Saída da ligação do motor Direito, HIGH ou LOW 
int ENB = 10; // Pino de Saída do controle PWM do motor Direito


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1); // Define os pinos na função da biblioteca de controle de motores DC e as constantes de velocidade dos motores 


// Temporizador


long time1Hz = 0; // Define time1Hz como sendo do tipo long 


// MPU e DMP
volatile bool mpuInterrupt = false;     // indica se o pino de interrupção do MPU6050 foi alto (HIGH)
void dmpDataReady() // Define a função dmpDataReady para verificação de inicialização do DMP (Digital Motion Processor)
{
    mpuInterrupt = true; // Análisa se houve interrupção do MPU6050 se sim recebe True 
}


void setup() //Função de configuração do setup para inicialização no arduino
{
    // aderir ao barramento I2C (a biblioteca I2Cdev não faz isso automaticamente)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE  // Verifica se a implementação da comunicação I2Cdev com o arduino é compatível com as funções I2CDEV_ARDUINO_WIRE
        Wire.begin(); // Inicia a comunicação com o MPU6050 e o arduino
        TWBR = 24; // Define a velocidade do Clock de comunicação do MPU e o CPU do arduino
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE //Se não, verifica se a implementação da comunicação I2Cdev com o arduino é compatível com as funções I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true); // Inicia a função de comunicação do MPU com o arduino pela biblioteca Fastwire definindo os paramêtros da função
    #endif // Encerra a Macro condição 

    //Inicia a porta de comunicação Serial 
    Serial.begin(115200);

    // Inicia o dispostivo de comunicação I2C (MPU6050)
    mpu.initialize(); // Função de inicialização do MPU6050

    // Verifica a conexão do dispositivo com o arduino
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); //Realiza a função de teste de conexão com o MPU e imprime se falhou ou funcionou  

    // Carrega e configura o DMP (Digital Motion Processor)
    devStatus = mpu.dmpInitialize(); // Invoca a função de inicialização do DMP e salva seu retorno na variável devStatus 

    // Forneça os valores de deslocamentos do giroscópio, ajustados para a sensibilidade mínima
    mpu.setXGyroOffset(220); // Define o valor como paramêtro da função para o ângulo X do Giroscópio
    mpu.setYGyroOffset(76); // Define o valor como paramêtro da função para o ângulo Y do Giroscópio
    mpu.setZGyroOffset(-85); // Define o valor como paramêtro da função para o ângulo z do Giroscópio
    mpu.setZAccelOffset(1788); // Define o valor como paramêtro da função para o ângulo Z do Acelerômetro 

    // Certifica-se se o valor da inicialização do DMP foi realizada com sucesso (retorna 0 se assim for)
    if (devStatus == 0)
    {
        // Liga o DMP, agora que está ok (Passou no teste)
        mpu.setDMPEnabled(true); // Invoca a função de habilitação do DMP passando o paramêtro verdadeiro

        // Ativa a detecção de interrupção do Arduino
        attachInterrupt(0, dmpDataReady, RISING); // Invoca a função de interrupção do MPU6050 com os paramêtros fornecidos entre parênteses 
        mpuIntStatus = mpu.getIntStatus(); // Invoca a função do status do funcionamento da interrupção do MPU6050 e salva o retorno na variável mpuIntStatus 

        // define nosso sinalizador DMP Ready para que a função loop () principal saiba que está tudo ok
        dmpReady = true; // A variável dmpReady recebe o valor bolleano Verdadeiro

        // obter tamanho de pacote DMP esperado para comparação posterior
        packetSize = mpu.dmpGetFIFOPacketSize(); 
        
        // Configuração do PID
        
        pid.SetMode(AUTOMATIC); // Invoca a função de definição do modo do PID e realiza o setup da forma escolhida 
        pid.SetSampleTime(10); // Invoca a função do tempo de amostragem do PID (Tempo de realização dos cálculos em milisegundos)
        pid.SetOutputLimits(-255, 255); // Invoca a função da saída limite do PID e define os valores máximo e mínimo 
    }
}


void loop()  //Função de configuração do loop de repetição do arduino
{
    // se a programação de configuração falhou, não realiza nada do loop
    if (!dmpReady) return;

    // aguarde interrupção do MPU com pacote(s) extra disponível
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        
        // Quando o MPU6050 envia os dados, é executado os cálculos do PID e de acordo com o cálculo define a velocidade de saída para os motores
        pid.Compute(); // Invoca a função do cálculo do PID da biblioteca do arduino 
        motorController.move(output, MIN_ABS_SPEED); // Invoca a função de controle dos motores DC Usando como paramêtros os motores a velocidade (output do PID) e a velociade Mínima 
        
        unsigned long currentMillis = millis(); // Define currentMillis como sendo do tipo unsigned long, que recebe os milisegundos de cada loop 
        
        if (currentMillis - time1Hz >= 1000) // Se o tempo de currentMillis menos o tempo de time1Hz for menor ou igual a 1000 realiza o código abaixo 
        {
            loopAt1Hz(); // Invoca a função loopAt1Hz que está definida próxima ao final do código 
            time1Hz = currentMillis; // Define que a variável time1Hz é igual a variável currentMillis (De 1000 em 1000 "A cada segundo")
        }
        
    }

    // redefini o sinalizador de interrupção e obtém o valor do byte INT_STATUS
    mpuInterrupt = false; // A variável mpuInterrupt recebe Falso para redefinição  
    mpuIntStatus = mpu.getIntStatus(); // A variável mpuIntStatus recebe o valor do estado do byte de interrupção do MPU6050

    // Obter conta FIFO atual
    fifoCount = mpu.getFIFOCount();

    // Verifica se ocorre Overflow "estouro da memória" (Não deve acontecer com frêquencia)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // Se acontecer o overflow o FIFO do MPU6050 é resetado para continuar limpo
        mpu.resetFIFO(); // Função que reseta o FIFO (acrônimo para First In, First Out, que em português significa primeiro a entrar, primeiro a sair) do MPU6050
    }
    
    // Caso contrário, verifica se há interrupção pronta de dados do DMP (isso deve acontecer com freqüência)
    else if (mpuIntStatus & 0x02)
    {
        // Aguarda o comprimento correto dos dados disponíveis, deve ser uma espera MUITO curta
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Ler um pacote de FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // Pista FIFO contagem aqui caso haja mais que 1 pacote disponível
        // (Isso nos permite ler imediatamente mais sem esperar por uma interrupção de novos dados)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); // Lê os valores do recepiente quartenion do MPU6050 obtidos pelo DMP e com uma ordem FIFO 
        mpu.dmpGetGravity(&gravity, &q); // Lê os valores da gravidade do MPU6050 obtidos pelo DMP e com uma ordem FIFO 
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Lê os valores de Yaw, Pitch e Roll do MPU6050 obtidos pelo DMP e com uma ordem FIFO 
        #if LOG_INPUT // Se a constante LOG_INPUT da Macro decisão for diferente de 0 imprime no Serial os valores de Yaw, Pitch e Roll obtidos pelo MPU6050, Se for igual a 0 não imprime 
            Serial.print("ypr\t"); // Imprime no Serial, "ypr" (Que significa Yaw, Pitch e Roll, respectivamente do MPU6050)
            Serial.print(ypr[0] * 180/M_PI); // Imprime no Serial, o valor de Yaw 
            Serial.print("\t"); // Imprime um espaçamento de tabulação (tab)
            Serial.print(ypr[1] * 180/M_PI); // Imprime no Serial, o valor de Pitch
            Serial.print("\t"); // Imprime um espaçamento de tabulação (tab)
            Serial.println(ypr[2] * 180/M_PI); // Imprime no Serial, o valor de Roll
        #endif // Encerra a Macro condição
        input = ypr[2] * 180/M_PI; // Seleciona o valor desejado para o programa, se ypr[0] = Yaw, se ypr[1] = Pitch, se ypr[2] = Roll
                                         // Armazena no input para o cálculo do PID o valor desejado para o robô, (No nosso caso o valor de Roll ypr[2])
   }
}


void loopAt1Hz() // Define a função loopAt1Hz (É realizada a cada segundo)
{
#if MANUAL_TUNING // Se a constante MANUAL_TUNING for diferente de 0, é realizado o comando abaixo
    setPIDTuningValues(); // Define os valores do Kp, Ki e Kd do PID, que podem ser definidos pelos potenciômetros
#endif // Encerra a Macro condição 
}

//PID Configuração (com 3 potenciômetros)

#if MANUAL_TUNING // Se a constante MANUAL_TUNING for diferente de 0 a macro decisão realiza os códigos abaixo
void setPIDTuningValues() // Invoca a função de setup do PID 
{
    readPIDTuningValues(); // Lê os valores das constantes configuradas para o PID
    
    if (kp != prevKp || ki != prevKi || kd != prevKd) // Se os valores antigos das constantes antigas forem diferentes das atuais, realiza os códigos abaixo
    {
#if LOG_PID_CONSTANTS // Se a constante LOG_PID_CONSTANTS for diferente de zero (Recomendado se a configuração for alterada para manual), realiza o código abaixo 
        Serial.print(kp);Serial.print(", ");Serial.print(ki);Serial.print(", ");Serial.println(kd); // Imprime no Serial, os valores de Kp, Ki e Kd obtidos pelos potenciômetros 
#endif // Encerra a Macro condição 

        pid.SetTunings(kp, ki, kd); // Como os valores atuais são diferentes dos antigos, invoca a função de configuração do PID e faz o setup dos novos valores 
        prevKp = kp; prevKi = ki; prevKd = kd; // Define que os valores antigos são iguais aos atuais após a configuração
    }
}


void readPIDTuningValues() // Define a função readPIDTuningValues(), para leitura dos valores das constantes do PID atráves dos potenciômetros
{
    int potKp = analogRead(A0); //Define potKp como sendo int (inteiros), que recebe o valor do potenciômetro do pino A0 do arduino 
    int potKi = analogRead(A1); //Define potKi como sendo int (inteiros), que recebe o valor do potenciômetro do pino A1 do arduino 
    int potKd = analogRead(A2); //Define potKd como sendo int (inteiros), que recebe o valor do potenciômetro do pino A2 do arduino 
        
    kp = map(potKp, 0, 1023, 0, 25000) / 100.0; //0 - 250 Faz um novo mapeamento "Regra de Três" dos novos valores máximos e mínimos da leitura do potenciômetro representando Kp
    ki = map(potKi, 0, 1023, 0, 100000) / 100.0; //0 - 1000 Faz um novo mapeamento "Regra de Três" dos novos valores máximos e mínimos da leitura do potenciômetro representando Ki
    kd = map(potKd, 0, 1023, 0, 500) / 100.0; //0 - 5 Faz um novo mapeamento "Regra de Três" dos novos valores máximos e mínimos da leitura do potenciômetro representando Kd
}
#endif // Encerra a Macro condição 
