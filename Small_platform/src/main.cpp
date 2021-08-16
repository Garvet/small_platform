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

// --- Моторы ---
// Пины для драйверов (M1, M2)
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

// Подключение моторов к драйверам (L и R к M1 и M2)
// #define M1R_M2L
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

// Номера используемых ШИМ-каналов
constexpr uint8_t CHANNEL_RIGHT_WHELL  = 0;
constexpr uint8_t CHANNEL_LEFT_WHELL   = 1;

// --- CAN шина ---
CAN_device_t CAN_cfg;             // Конфигурация CAN
unsigned long previousMillis = 0; // Время последней отправки CAN сообщения
const int interval = 500;         // Интервал отправки CAN сообщений (мс)
const int rx_queue_size = 10;     // Размер буфера приёма
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_22;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_21;

// --- УФ диод ---
constexpr uint8_t UV_LIGHT = GPIO_NUM_23;



// Объекты двигателей
Engine engine_right;
Engine engine_left;

// Счётчики энкодеров двигателей (прерывания по сигналу)
void hall_left() {
    engine_left.interrupt();
}
void hall_right() {
    engine_right.interrupt();
}

// Остановка двигателей
void stop() {
    engine_left.set_target_speed(0);
    engine_right.set_target_speed(0);
}


void setup() {
    // Запуск UART'а
    Serial.begin(115200);
    
    delay(100);

    // Инициализация УФ-диода
    pinMode(UV_LIGHT, OUTPUT);

    // Инициализация двигателей
    engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL, hall_left);
    engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL, hall_right);
    // engine_left.init(LEFT_WHEEL_FORWARD, LEFT_WHEEL_BACKWARD, LEFT_WHEEL_PWM, LEFT_ENCODER, CHANNEL_LEFT_WHELL);
    // engine_right.init(RIGHT_WHEEL_FORWARD, RIGHT_WHEEL_BACKWARD, RIGHT_WHEEL_PWM, RIGHT_ENCODER, CHANNEL_RIGHT_WHELL);
    engine_left.begin();
    engine_right.begin();
    stop();

    // Инициализация CAN шины
    CAN_cfg.tx_pin_id = CAN_TX_PIN;
    CAN_cfg.rx_pin_id = CAN_RX_PIN;
#if defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 )
    // ESP типа heltec_wifi_lora_32 или heltec_wifi_lora_32_v2
    CAN_cfg.speed = CAN_SPEED_500KBPS;
#elif ~(defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ))
    // Другие ESP
    CAN_cfg.speed = CAN_SPEED_1000KBPS;
#endif
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    ESP32Can.CANInit();
}


void loop() {
    set_speed_serial();
}




#define ERROR 30000
#define MAX 16

uint8_t buffer[MAX];
int16_t val1 = 0, val2 = 0;

void set_speed_serial() {
    Set_Serial set_param;                                 
    work_with_CAN();
    
    if(Serial.available() != 0) {
        for(int i = 0; i < MAX; ++i){
            buffer[i] = 0;
        }
        delay(5);
        uint8_t size = Serial.readBytes(&buffer[0], MAX);
        int16_t cmd = set_param.read_command(buffer, size); 
        if(cmd < ERROR) {
            switch(cmd) {
                case 0: 
                break;
                case 1:
                    val1 = set_param.check_error_t(&buffer[3], 0);
                    val2 = set_param.check_error_t(&buffer[9], 1);
                    if (val1 < ERROR) {
                        //engine_left.set_target_speed(val1);
                        engine_left.set_power(val1*10);
                    }
                    else {
                                                                                    Serial.print("!1!");
                        Serial.println("val1");
                    }
                    if (val2 < ERROR) {
                        //engine_right.set_target_speed(val2);
                        engine_right.set_power(val2*10);
                    }
                    else {
                                                                                    Serial.print("!2!");
                        Serial.println("val2");
                    }
                    break;
                case 2:
                    val1 = set_param.check_error_c(&buffer[3], 0, buffer[2]);
                    val2 = set_param.check_error_c(&buffer[9], 1, buffer[2]); //проверка на L/R (в самой функции)
                    if (val1 < ERROR) {
                    if (buffer[2] == 'L' || buffer[2] == 'l') {
                        if (val1 < ERROR && val2 < ERROR) {
                            engine_left.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR) {
                            engine_left.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR) {
                            engine_left.set_coefficient(10000, 0, val2);
                        }
                        else {
                                                                                    Serial.print("!3!");
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                    else {
                        if (val1 < ERROR && val2 < ERROR) {
                            engine_right.set_coefficient(val1, 0, val2);
                        }
                        else if (val1 < ERROR && val2 >= ERROR) {
                            engine_right.set_coefficient(val1, 0, 10000);
                        }
                        else if (val2 < ERROR && val1 >= ERROR) {
                            engine_right.set_coefficient(10000, 0, val2);
                        }
                        else {
                                                                                    Serial.print("!4!");
                            Serial.println(val1);
                            Serial.println(val2);
                        }
                    }
                    }
                    else { 
                                                                                    Serial.print("!5!");
                        Serial.println(val1); 
                    }
                    break;
                case 3:
                    val1 = set_param.check_error_f(&buffer[3]);
                    if (val1 < ERROR) {
                        // freq_send = val1; (-) -----
                    }
                    else {
                                                                                    Serial.print("!6!");
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
                                                                                    Serial.print("!7!");
                Serial.println("Unknown error");
                break;
            }
        }
        else {
                                                                                    Serial.print("!8!");
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
