// Configuração dos botões
#define START_BUTTON 34  // GPIO34 (pino analógico pode ser usado como digital)
#define RESET_BUTTON 35  // GPIO35

// Configuração dos encoders 
#define ENC1_A 18  // GPIO18
#define ENC1_B 19  // GPIO19

#define ENC2_A 21  // GPIO21
#define ENC2_B 22  // GPIO22


// Configuração dos motores (usando a pinagem da Adafruit Motorshield V2)
// Nota: A Motorshield V2 usa endereço I2C e não pinos GPIO diretos
#define MOTOR1A_PIN -1  // Não usado com a Motorshield V2
#define MOTOR1B_PIN -1
#define MOTOR2A_PIN -1
#define MOTOR2B_PIN -1

// Configuração do solenoide
#define SOLENOID_PIN_A 23  // GPIO23
#define SOLENOID_PIN_B 25  // GPIO25

// Pinos dos sensores IR (conectados diretamente sem multiplexador)
#define IR1_PIN 36  // GPIO36 (VP)
#define IR2_PIN 39  // GPIO39 (VN)
#define IR3_PIN 32  // GPIO32
#define IR4_PIN 33  // GPIO33
#define IR5_PIN 34  // GPIO34 (pode conflitar com START_BUTTON)

// Removidos defines do multiplexador (não necessário no ESP32)
// #define MUXA_PIN    18
// #define MUXB_PIN    19
// #define MUXC_PIN    20

#define TINY_CTRL_PIN	5  // GPIO5

#define NOMINAL_SPEED  100  //0-255 directly to motors PWM






