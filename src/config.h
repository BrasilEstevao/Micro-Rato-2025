// Configuração dos botões
#define START_BUTTON 26  // GPIO34 (pino analógico pode ser usado como digital)
#define RESET_BUTTON 25  // GPIO35

// Configuração dos encoders 
#define ENC1_A 18  // GPIO18
#define ENC1_B 19  // GPIO19

#define ENC2_A 21  // GPIO21
#define ENC2_B 22  // GPIO22

// Pinos dos sensores IR (conectados diretamente sem multiplexador)
#define IR1_PIN 36  // GPIO36 (VP)
#define IR2_PIN 39  // GPIO39 (VN)
#define IR3_PIN 4  // GPIO32
#define IR4_PIN 35  // GPIO33
#define IR5_PIN 34  // GPIO34 (pode conflitar com START_BUTTON)

#define TINY_CTRL_PIN	5  // GPIO5

#define NOMINAL_SPEED  100  //0-255 directly to motors PWM

#define ENC1_A 2
#define ENC1_B 3

#define ENC2_A 0
#define ENC2_B 1



#define MOTOR1A_PIN 16
#define MOTOR1B_PIN 17

#define MOTOR2A_PIN 14
#define MOTOR2B_PIN 15


#define ADC_IN_PIN	28

#define MUXA_PIN	18
#define MUXB_PIN	19
#define MUXC_PIN	20

#define TINY_CTRL_PIN	21

#define NOMINAL_SPEED  FOLLOW_SPEED +40  //0-255 directly to motors PWM

#define FOLLOW_SPEED   80//0-255 directly to motors PWM
//tested values
//120 best one




