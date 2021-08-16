// #define USE_OLD_CODE
#ifndef USE_OLD_CODE
//s
#include <Arduino.h>
#include <Engine.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <Set_Serial.h>
#include <Get_data.h>

unsigned long timing;
void set_speed_serial();
void work_with_CAN();
void send_message();

constexpr uint8_t M1_WHEEL_BACKWARD = GPIO_NUM_33; // = GPIO_NUM_27;
constexpr uint8_t M1_WHEEL_FORWARD  = GPIO_NUM_25; // = GPIO_NUM_26;
constexpr uint8_t M1_WHEEL_PWM      = GPIO_NUM_32; // = GPIO_NUM_25;
constexpr uint8_t M1_ENCODER1       = GPIO_NUM_36;
constexpr uint8_t M1_ENCODER2       = GPIO_NUM_39;

constexpr uint8_t M2_WHEEL_BACKWARD = GPIO_NUM_27; // = GPIO_NUM_14;
constexpr uint8_t M2_WHEEL_FORWARD  = GPIO_NUM_26; // = GPIO_NUM_12;
constexpr uint8_t M2_WHEEL_PWM      = GPIO_NUM_19; // = GPIO_NUM_13;
constexpr uint8_t M2_ENCODER1       = GPIO_NUM_35;
constexpr uint8_t M2_ENCODER2       = GPIO_NUM_34;

constexpr uint8_t UV_LIGHT          = GPIO_NUM_23;

//#define M1R_M2L
#define M2R_M1L

#if defined (M1R_M2L)
// R = M1
constexpr uint8_t RIGHT_WHEEL_BACKWARD = M1_WHEEL_BACKWARD;
constexpr uint8_t RIGHT_WHEEL_FORWARD  = M1_WHEEL_FORWARD;
constexpr uint8_t RIGHT_WHEEL_PWM      = M1_WHEEL_PWM;
constexpr uint8_t RIGHT_ENCODER        = M1_ENCODER1;
// L = M2
constexpr uint8_t LEFT_WHEEL_BACKWARD  = M2_WHEEL_BACKWARD;
constexpr uint8_t LEFT_WHEEL_FORWARD   = M2_WHEEL_FORWARD;
constexpr uint8_t LEFT_WHEEL_PWM       = M2_WHEEL_PWM;
constexpr uint8_t LEFT_ENCODER         = M2_ENCODER1;
#elif defined (M2R_M1L)
// R = M2
constexpr uint8_t RIGHT_WHEEL_BACKWARD = M2_WHEEL_BACKWARD;
constexpr uint8_t RIGHT_WHEEL_FORWARD  = M2_WHEEL_FORWARD;
constexpr uint8_t RIGHT_WHEEL_PWM      = M2_WHEEL_PWM;
constexpr uint8_t RIGHT_ENCODER        = M2_ENCODER1;
// L = M1
constexpr uint8_t LEFT_WHEEL_BACKWARD  = M1_WHEEL_BACKWARD;
constexpr uint8_t LEFT_WHEEL_FORWARD   = M1_WHEEL_FORWARD;
constexpr uint8_t LEFT_WHEEL_PWM       = M1_WHEEL_PWM;
constexpr uint8_t LEFT_ENCODER         = M1_ENCODER1;
#endif
constexpr uint8_t CHANNEL_RIGHT_WHELL  = 0;
constexpr uint8_t CHANNEL_LEFT_WHELL   = 1;
//init CAN 
CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 500;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

#define TIMER_NUMBER 0
// Частота ESP32 = 80МГц = 80.000.000Гц, при
// значении 80 частота счётчика 1.000.000
#define TIMER_DIVIDER 80
// Количество тиков счётчика для вызова прерываний.
// При частоте 1МГц, 100000 тиков = 100мс
// ULL - unsigned long long для литералов
#define TIMER_COUNTER 100000ULL
// Указатель на используемый таймер

#define ERROR 30000
hw_timer_t * timer = NULL;

uint16_t freq_send = 100;

// объекты двигателя
Engine engine_right;
Engine engine_left;

void hall_left() {
    engine_left.interrupt();
}
void hall_right() {
    engine_right.interrupt();
}

void timer_interrupt() {
    // Serial.print("L");
    engine_left.PID_calculate();
    // Serial.print("R");
    engine_right.PID_calculate();
    // Serial.println();
}

void stop() {
    engine_left.set_target_speed(0);
    engine_right.set_target_speed(0);
}


void iBus_read_fun();

void setup() {
    Serial.begin(115200);

    engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL, hall_left);
    engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL, hall_right);

    // engine_left.set_coefficient(2.5F, 0.0F, 0.01F);
    // engine_left.set_iteration_time(100);
    // engine_right.set_coefficient(2.5F, 0.0F, 0.01F);
    // engine_right.set_iteration_time(100);
    pinMode(UV_LIGHT, OUTPUT);
    engine_left.set_coefficient(2.5F, 0.00F, -0.1F);
    engine_left.set_iteration_time(100);

    engine_right.set_coefficient(2.5F, 0.00F, -0.1F);
    engine_right.set_iteration_time(100);

    engine_left.begin();
    engine_right.begin();

    delay(1000);
    stop();
    /// --- Настройка таймера ---
    // Настраиваем 0-й таймер с делителем в 80 тактов
   // timer = timerBegin(TIMER_NUMBER, TIMER_DIVIDER, true);
    // Привязываем прерывание к функции timer_interrupt()
   // timerAttachInterrupt(timer, &timer_interrupt, true);
    // Настраиваем прерывание на 750мс
  //  timerAlarmWrite(timer, TIMER_COUNTER, true);
    // Запуск таймера
  //  timerAlarmEnable(timer);
    /// --- - ---

    engine_left.set_target_speed(0);
    engine_right.set_target_speed(0);
     //CAN
     
     CAN_cfg.speed = CAN_SPEED_1000KBPS; //esp small platform
 //   CAN_cfg.speed = CAN_SPEED_500KBPS; //esp lora
     CAN_cfg.tx_pin_id = GPIO_NUM_22;
     CAN_cfg.rx_pin_id = GPIO_NUM_21;
     CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
     // Init CAN Module
     ESP32Can.CANInit();

    //  digitalWrite(UV_LIGHT, HIGH);
     
  /*   while(1){
         engine_left.set_power(495);
     }*/
}


void loop() {
  set_speed_serial();
  //engine_left.set_power(999);
  //engine_right.set_power(999);
  //engine_left.set_target_speed(150);
  //Serial.println("work");
 // work_with_CAN();
}

#define MAX 16

uint8_t buffer[MAX];
int16_t val1 = 0, val2 = 0;

void set_speed_serial() {
    Set_Serial set_param;                                 
    work_with_CAN();
    
    if (millis() - timing > freq_send){
        timing = millis(); 
        //work_with_CAN();
        //Serial.println("work");
        //send_message();
    }
    if(Serial.available() != 0) {
        for(int i = 0; i < MAX; ++i){
            buffer[i] = 0;
        }
        //delay(100);
        uint8_t size = Serial.readBytes(&buffer[0], MAX);
        int16_t cmd = set_param.read_command(buffer, size); 
        if(cmd < ERROR) {
            switch(cmd){
                case 0: 
                break;
                case 1:
                    val1 = set_param.check_error_t(&buffer[3], 0);
                    val2 = set_param.check_error_t(&buffer[9], 1);
                    if (val1 < ERROR){
                    //engine_left.set_target_speed(val1);
                    engine_left.set_power(val1*10);
                    }
                    else{
                        Serial.println("val1");
                    }
                    if (val2 < ERROR){
                        //engine_right.set_target_speed(val2);
                        engine_right.set_power(val2*10);
                    }
                    else{
                        Serial.println("val2");
                    }
                    break;
                case 2:
                    val1 = set_param.check_error_c(&buffer[3], 0, buffer[2]);
                    val2 = set_param.check_error_c(&buffer[9], 1, buffer[2]); //проверка на L/R (в самой функции)
                    if (val1 < ERROR){
                    if (buffer[2] == 'L' || buffer[2] == 'l') {
                        if (val1 < ERROR && val2 < ERROR){
                            engine_left.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR){
                        engine_left.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR){
                        engine_left.set_coefficient(10000, 0, val2);
                        }
                        else{
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                    else {
                        if (val1 < ERROR && val2 < ERROR){
                            engine_right.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR){
                            engine_right.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR){
                            engine_right.set_coefficient(10000, 0, val2);
                        }
                        else{
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                    }
                    else { 
                        Serial.println(val1); 
                    }
                    break;
                case 3:
                    val1 = set_param.check_error_f(&buffer[3]);
                    if (val1 < ERROR)
                    {
                        freq_send = val1;
                    }
                    else {
                        Serial.println(val1);
                    }
                    break;
                case 4:
                    //val1 = set_param.check_error_f(&buffer[3]);
                    if (buffer[2] == '+')
                    {
                        digitalWrite(UV_LIGHT, HIGH);
                    }
                    else if (buffer[2] == '-') {
                        digitalWrite(UV_LIGHT, LOW);
                    }
                    break;
            default: 
                Serial.println("Unknown error");
                break;
            }
        }
        
        else{
           Serial.println(cmd);
        }
        val1=0; val2=0;
    }
}
void work_with_CAN(){
    CAN_frame_t rx_frame;
    Get_data data;
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
    //data.print(rx_frame.data.u8,rx_frame.MsgID);
    data.print(rx_frame.data.u8, rx_frame.MsgID);
    }

}

#else

#define USE_OLD_CODE1
#ifndef USE_OLD_CODE1

#include <Arduino.h>


constexpr uint8_t ENCODER1 = GPIO_NUM_36;
constexpr uint8_t ENCODER2 = GPIO_NUM_39;
constexpr uint8_t ENCODER3 = GPIO_NUM_35;
constexpr uint8_t ENCODER4 = GPIO_NUM_34;

uint16_t hall_en_1 = 0;
uint16_t hall_en_2 = 0;
uint16_t hall_en_3 = 0;
uint16_t hall_en_4 = 0;

void hall1() {
    hall_en_1++;
}
void hall2() {
    hall_en_2++;
}
void hall3() {
    hall_en_3++;
}
void hall4() {
    hall_en_4++;
}


#define TIMER_NUMBER 0
#define TIMER_DIVIDER 80
#define TIMER_COUNTER 100000ULL
hw_timer_t * timer = NULL;
void timer_interrupt() {
    Serial.print("EN1 = ");
    Serial.print(hall_en_1);
    Serial.print("  \tEN2 = ");
    Serial.print(hall_en_2);
    Serial.print("  \tEN3 = ");
    Serial.print(hall_en_3);
    Serial.print("  \tEN4 = ");
    Serial.println(hall_en_4);
    hall_en_1 = 0;
    hall_en_2 = 0;
    hall_en_3 = 0;
    hall_en_4 = 0;
}


constexpr uint8_t num_pin1 = GPIO_NUM_33;
constexpr uint8_t num_pin2 = GPIO_NUM_25;
constexpr uint8_t num_pin3 = GPIO_NUM_32;
constexpr uint8_t num_pin4 = GPIO_NUM_27;
constexpr uint8_t num_pin5 = GPIO_NUM_26;
constexpr uint8_t num_pin6 = GPIO_NUM_19;

void setup() {
    Serial.begin(115200);

    pinMode(ENCODER1, INPUT_PULLUP);
    pinMode(ENCODER2, INPUT_PULLUP);
    pinMode(ENCODER3, INPUT_PULLUP);
    pinMode(ENCODER4, INPUT_PULLUP);
    // pinMode(ENCODER1, INPUT);
    // pinMode(ENCODER2, INPUT);
    // pinMode(ENCODER3, INPUT);
    // pinMode(ENCODER4, INPUT);
    attachInterrupt(ENCODER1, hall1, RISING);
    attachInterrupt(ENCODER2, hall2, RISING);
    attachInterrupt(ENCODER3, hall3, RISING);
    attachInterrupt(ENCODER4, hall4, RISING);



    /// --- Настройка таймера ---
    // Настраиваем 0-й таймер с делителем в 80 тактов
    timer = timerBegin(TIMER_NUMBER, TIMER_DIVIDER, true);
    // Привязываем прерывание к функции timer_interrupt()
    timerAttachInterrupt(timer, &timer_interrupt, true);
    // Настраиваем прерывание на 750мс
    timerAlarmWrite(timer, TIMER_COUNTER, true);
    // Запуск таймера
    // timerAlarmEnable(timer);
    /// --- - ---
}

void loop() {
    static uint8_t mas[6] = {num_pin1, num_pin2, num_pin3, num_pin4, num_pin5, num_pin6}; 
    static uint8_t mas2[6] = {0, 0, 0, 0, 0, 0}; 
    for(int i = 0; i < 6; ++i) {
        for(int j = 0; j < 6; ++j) {
            digitalWrite(mas[j], LOW);
            pinMode(mas[j], INPUT);
        }
        pinMode(mas[i], OUTPUT);

        // digitalWrite(mas[i], LOW);
        // delay(10);
        // Serial.print("M["); Serial.print(i); Serial.print("] = ");
        // for(int j = 0; j < 6; ++j) {
        //     mas2[j] = digitalRead(mas[j]); Serial.print(mas2[j]); Serial.print(" ");
        // }
        // Serial.println("(LOW)");
        // delay(1000);

        digitalWrite(mas[i], HIGH);
        delay(10);
        Serial.print("M["); Serial.print(i); Serial.print("] = ");
        for(int j = 0; j < 6; ++j) {
            mas2[j] = digitalRead(mas[j]); Serial.print(mas2[j]); Serial.print(" ");
        }
        // Serial.println("(HIGH)");
        Serial.println();
        delay(1000);

        // digitalWrite(mas[i], LOW);
        // delay(10);
        // Serial.print("M["); Serial.print(i); Serial.print("] = ");
        // for(int j = 0; j < 6; ++j) {
        //     mas2[j] = digitalRead(mas[j]); Serial.print(mas2[j]); Serial.print(" ");
        // }
        // Serial.println("(LOW)\n");
        // delay(1000);
    }
}

#else



#include <Arduino.h>

int8_t read_command();
void print_speed(int16_t);
uint8_t buffer[10];
int16_t left_taarget_speed  = 0;
int16_t right_taarget_speed = 0;
int16_t speed[2];

void setup() {
    Serial.begin(115200);

}
int8_t result = 0;
void loop() {
    result = read_command();
    if(result != 0) {
        if(result == 1) {
            Serial.print(0);
            Serial.print(" L = ");
            print_speed(left_taarget_speed);
            Serial.print(" R = ");
            print_speed(right_taarget_speed);
            Serial.println();
        }
        else {
            if(result != -3)
                Serial.println(result * -1);
            else {
                if(speed[0] > 1000) {
                    Serial.println(speed[0] - 1000 + 2);
                }
                else if(speed[1] > 1000) {
                    Serial.println(speed[1] - 1000 + 2 + 3);
                }
            }
        }
    }
}

void print_speed(int16_t s) {
    if(s < 0) { Serial.print("-"); s *= -1;}
    else        Serial.print("+");
    if(s < 100) Serial.print("0");
    else        Serial.print(s/100);
    if(s%100 < 10) Serial.print("0");
    else           Serial.print(s%100/10);
    Serial.print(s%10);
}

int16_t read_speed(uint8_t *buf) {
    int16_t speed = 0;
    for(int i = 1; i < 4; ++i) {
        if(('0' <= buf[i]) && (buf[i] <= '9')) {
            speed *= 10;
            speed += buf[i] - '0';
        }
        else {
            return 1001; // 3
        }
    }
    if(buf[0] == '-') {
        speed *= -1;
    }
    else if(buf[0] != '+') {
        return 1002; // 4
    }
    if(300< abs(speed))
        return 1003; // 5
    return speed;
}

int8_t read_command() {
    static uint8_t size;
    if(Serial.available() != 0) {
        for(int i = 0; i < 9; ++i)
            buffer[i] = 0;
        delay(1);
        buffer[0] = Serial.read();
        if((buffer[0] != 's') && (buffer[0] != 'S')) {
            if((buffer[0] != '\r') && (buffer[0] != '\n'))
                return -1;
            return 0;
        }
        size = Serial.readBytes(&buffer[1], 9);
        if(size < 9)
            return -2;
        speed[0] = read_speed(&buffer[1]);
        speed[1] = read_speed(&buffer[5]);
        if(speed[0] > 1000 || speed[1] > 1000)
            return -3;
        left_taarget_speed  = speed[0];
        right_taarget_speed = speed[1];
        if((buffer[9] != 'e') && (buffer[9] != 'E'))
            return -9;
        return 1;
    }
    return 0;
}

#endif
#endif