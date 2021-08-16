//#define OLD_CODE
#ifndef OLD_CODE
#include "Set_Serial.h"

Set_Serial::Set_Serial(): val(0), size(0), tmp (nullptr){}

Set_Serial::~Set_Serial(){
    val = 0;
    if (tmp != nullptr) {delete[] tmp;}
}
int16_t Set_Serial::check_error_t(uint8_t *buf, bool num){
  val = 0;
  read_val(buf);
    if (val > 495 || val < -495){
       if (!num){  return INTERVAL_T_L;}
       else return INTERVAL_T_R;
    }
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_T_L;}
    else return SIGN_T_R;
    }
    return val;
}

int16_t Set_Serial::check_error_c(uint8_t *buf, bool num, uint8_t LR){
    val = 0;
    read_val(buf);
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_C_P;}
       else return SIGN_C_D;
    }
    if (LR != 'L' && LR != 'l' && LR != 'R' && LR != 'r') return NOT_L_or_R;
    return val;
}

int16_t Set_Serial::check_error_f(uint8_t *buf){
    val = 0;
    read_val(buf);
    if (val >= 10000 ){
        return INTERVAL_F;
    }
   return val;
}

void Set_Serial::read_val(uint8_t *buf) {
    for(int i = 1; i < 6; ++i) {
        if(('0' <= buf[i]) && (buf[i] <= '9')) {
            val *= 10;
            val += buf[i] - '0';
        }
        else {
            val = NOT_NUM;
            return; 
        }
    }
}
int16_t Set_Serial::read_command(uint8_t *buf, uint8_t size) {
    if((buf[S] != 's') && (buf[S] != 'S')) {
        if((buf[S] != '\r') && (buf[S] != '\n')) {return START_SYMBOL;}
        return 0;
    }
   if(size < (SET_MAX)) {return LENGTH_BYTES;}
   if((buf[SET_MAX-1] != 'e') && (buf[SET_MAX-1] != 'E')) {return END_SYMBOL;}
   if (buf[CMD] == 't' || buf[CMD] == 'c' || buf[CMD] == 'f' || buf[CMD] == 'u')
   {
       buf[CMD] -= 32;
   }
   switch(buf[CMD]){
        case 'T':
            return 1;
        case 'C':
            return 2;
        case 'F':
            return 3;
        case 'U':
            return 4;
        default:
            return NOT_CMD;
    } 
}
#else 
void Set_Serial::print(uint8_t *data, uint32_t ID){
    Serial.println("Hello");
    if (ID == 11){
        Serial.print("US\t");
        CAN_data[0] = data[0];
        format_byte(CAN_data, 1);
    }
    else if (ID == 10){
        Serial.print("IR\t");
        for (int i = 0; i < CAN_MAX; i++){
            CAN_data[i] = data[i];
        }
    }
    else Serial.print(ERR_ID);
  // Serial.print(sizeof(data), DEC);
   
    for (int i = 0; i < CAN_MAX; i++){
        Serial.print(CAN_data[i]);
         Serial.print("\t");
    }
    
    Serial.println();
}

//void Set_Serial::print(uint8_t *data, uint32_t ID)
/*void Set_Serial::print(){ 
    Serial.println();
    uint32_t ID1 = 10;
    uint8_t data1[8];
    uint8_t value = 4;
    for (int i = 0; i<8; i++){
        data1[i] = value++;
    }
    if (ID1 == 10){
        tmp = new char[IR_MAX*3 + SERIAL_IR];
        format_byte(data1, IR_MAX);
        tmp[0] = 'S';
        tmp[1] = 'I';
        tmp[IR_MAX*3 + SERIAL_IR - 1] = 'E';
           for (int i = 0; i < IR_MAX*3 + SERIAL_IR; i++){
               Serial.print(tmp[i]);}
    }
    else if (ID > 10) {
        tmp = new char[US_MAX*3 + SERIAL_US];
        format_byte(data, US_MAX);
        tmp[0] = 'S';
        tmp[1] = 'I';
        switch(ID){
            //US1
            case 11:
            tmp[2] = '1';
            //US2
            case 12:
            tmp[2] = '2';
            //US3
            case 13:
            tmp[2] = '3';
            //US4
            case 14:
            tmp[2] = '4';
            default: Serial.print(ERR_ID);
        }
        tmp[US_MAX*3 + SERIAL_US - 1] = 'E';
        for (int i = 0; i < US_MAX*3 + SERIAL_US; i++){
               Serial.print(tmp[i]);}
    }*/
 /*   switch(ID){
        //IR
        case 10: 
        tmp = new uint8_t [IR_MAX*3 + CAN_IR];
        format_byte(data, IR_MAX);
        break;
        //US1
        case 11:
        tmp = new uint8_t[US_MAX*3 + CAN_US];
        format_byte(data, US_MAX);
        //US2
        case 12:
        tmp = new uint8_t[US_MAX*3 + CAN_US];
        format_byte(data, US_MAX);
        //US3
        case 13:
        tmp = new uint8_t[US_MAX*3 + CAN_US];
        format_byte(data, US_MAX);
        //US4
        case 14:
        tmp = new uint8_t[US_MAX*3 + CAN_US];
        format_byte(data, US_MAX);
        default: Serial.print(ERR_ID);
    }
}
*/
/*void Set_Serial::format_byte(uint8_t *data, uint8_t len){
    size = len*3-1; 
  //  uint8_t *new_data = new uint8_t [len*3]; //change our data
    uint8_t *get_data = new uint8_t [len]; //current data
    for (int i = 2; i < size; i++){ //
       // new_data[i] = 0;
        tmp[i] = 0;
    }
    for (int i = 0; i < len; i++){
        get_data[i] = data[i];
    }
    for (int i = len-2; i > -1; i--){ //==0
        for (int j = 0 ; j < 3;  j++){
         //  new_data[size--] = get_data[i] % 10;
           tmp[size--] = get_data[i] % 10 + '0';
           get_data[i]/=10;
        }
    }
    delete[] get_data;
  for (int i = 0; i < len*3; i++){
        Serial.print(new_data[i]);
    }
}*/

void Set_Serial::format_byte(uint8_t *data, uint8_t len){
    uint8_t size = len*3-1;
    uint8_t *new_data = new uint8_t [len*3]; //change our data
    uint8_t *get_data = new uint8_t [len]; //current data
    if ( len == 8){
        uint8_t *packet = new  uint8_t [len*3+3];
    }
    else if ( len == 3) {
         uint8_t *packet = new  uint8_t [len*3+4];
    }
    for (int i = 0; i < len*3; i++){
        new_data[i] = 0;
    }
    for (int i = 0; i < len; i++){
        get_data[i] = data[i];
    }
    for (int i = len-1; i > -1; i--){
        for (int j = 0 ;j < 3;  j++){
           new_data[size--] = get_data[i] % 10;
            get_data[i]/=10;
        }
    }
    delete[] get_data;
    for (int i = 0; i < len*3; i++){
        Serial.print(new_data[i]);
    }
}
//#else 
#include "Set_Serial.h"

Set_Serial::Set_Serial() {
    val = 0;
}

Set_Serial::~Set_Serial(){
    val = 0;
    if (tmp != nullptr) {delete[] tmp;}
}
int16_t Set_Serial::check_error_t(uint8_t *buf, bool num){
  val = 0;
  read_val(buf);
    if (val > 300 || val < -300){
       if (!num){  return INTERVAL_T_L;}
       else return INTERVAL_T_R;
    }
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_T_L;}
    else return SIGN_T_R;
    }
    return val;
}

int16_t Set_Serial::check_error_c(uint8_t *buf, bool num, uint8_t LR){
    val = 0;
    read_val(buf);
    if (buf[0] == '-')
    val*=-1;
    else if (buf[0] != '+') {
    if (!num){  return SIGN_C_P;}
       else return SIGN_C_D;
    }
    if (LR != 'L' && LR != 'l' && LR != 'R' && LR != 'r') return NOT_L_or_R;
    return val;
}

int16_t Set_Serial::check_error_f(uint8_t *buf){
    val = 0;
    read_val(buf);
    if (val >= 10000 ){
        return INTERVAL_F;
    }
   return val;
}

void Set_Serial::read_val(uint8_t *buf) {
    for(int i = 1; i < 6; ++i) {
        if(('0' <= buf[i]) && (buf[i] <= '9')) {
            val *= 10;
            val += buf[i] - '0';
        }
        else {
            val = NOT_NUM;
            return; 
        }
    }
}
int16_t Set_Serial::read_command(uint8_t *buf, uint8_t size) {
    if((buf[S] != 's') && (buf[S] != 'S')) {
        if((buf[S] != '\r') && (buf[S] != '\n')) {return START_SYMBOL;}
        return 0;
    }
   if(size < (SET_MAX)) {return LENGTH_BYTES;}
   if((buf[SET_MAX-1] != 'e') && (buf[SET_MAX-1] != 'E')) {return END_SYMBOL;}
   if (buf[CMD] == 't' || buf[CMD] == 'c' || buf[CMD] == 'f')
   {
       buf[CMD] -= 32;
   }
   switch(buf[CMD]){
        case 'T':
            return 1;
        case 'C':
            return 2;
        case 'F':
            return 3;
        default:
            return NOT_CMD;
    } 
}

void Set_Serial::print(uint8_t *data, uint32_t ID){
//void Set_Serial::print(){ 
    Serial.println();
    uint8_t size = 0;
  /*  uint32_t ID1 = 10;
    uint8_t data1[8];
    uint8_t value = 4;
    for (int i = 0; i<8; i++){
        data1[i] = value++;
    }*/
    if (ID == 9){ //rapidity
        size = R_MAX*3 + SERIAL_R;
        tmp = new char[size];
        format_byte(data, R_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'R';
        tmp[size - 1] = 'E';
        for (int i = 0; i < size; i++){
            Serial.print(tmp[i]);
            }
    }
    else if (ID == 10){
        size = IR_MAX*3 + SERIAL_IR;
        tmp = new char[size];
        format_byte(data, IR_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'I';
        tmp[size - 1] = 'E';
        for (int i = 0; i < size; i++){
            Serial.print(tmp[i]);
            }
    }
    else if (ID > 10) {
        size = US_MAX*3 + SERIAL_US;
        tmp = new char[size];
        format_byte(data, US_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'U';
        switch(ID){
            //US1
            case 11:
            tmp[2] = '1'; //forward
            break;
            //US2
            case 12:
            tmp[2] = '2'; //back
            break;
            //US3
            case 13:
            tmp[2] = '3'; //left
            break;
            //US4
            case 14:
            tmp[2] = '4'; //right
            break;
            default: Serial.print(ERR_ID);
        }
        tmp[size - 1] = 'E';
        for (int i = 0; i <  size; i++){
               Serial.print(tmp[i]);}
    }
}

void Set_Serial::print(int32_t data, uint32_t ID){
//void Set_Serial::print(){ 
    Serial.println();
    uint8_t size = 0;
  /*  uint32_t ID1 = 10;
    uint8_t data1[8];
    uint8_t value = 4;
    for (int i = 0; i<8; i++){
        data1[i] = value++;
    }*/
    if (ID == 9){ //rapidity
        size = R_MAX*3 + SERIAL_R;
        tmp = new char[size];
        format_byte(data, R_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'R';
        tmp[size - 1] = 'E';
        for (int i = 0; i < size; i++){
            Serial.print(tmp[i]);
            }
    }
    else if (ID == 10){
        size = IR_MAX*3 + SERIAL_IR;
        tmp = new char[size];
        format_byte(data, IR_MAX, size - 1) ;
        tmp[0] = 'S';
        tmp[1] = 'I';
        tmp[size - 1] = 'E';
        for (int i = 0; i < size; i++){
            Serial.print(tmp[i]);
            }
    }
    else if (ID > 10) {
        size = US_MAX*3 + SERIAL_US;
        tmp = new char[size];
        format_byte(data, US_MAX, size - 1);
        tmp[0] = 'S';
        tmp[1] = 'U';
        switch(ID){
            //US1
            case 11:
            tmp[2] = '1'; //forward
            break;
            //US2
            case 12:
            tmp[2] = '2'; //back
            break;
            //US3
            case 13:
            tmp[2] = '3'; //left
            break;
            //US4
            case 14:
            tmp[2] = '4'; //right
            break;
            default: Serial.print(ERR_ID);
        }
        tmp[size - 1] = 'E';
        for (int i = 0; i <  size; i++){
               Serial.print(tmp[i]);}
    }
}


void Set_Serial::format_byte(uint8_t *data, uint8_t len, uint8_t size){
    int32_t *get_data = new int32_t [len]; //current data
    for (int i = 2; i < size; i++){ 
        tmp[i] = 0;
    }
    for (int i = 0; i < len; i++){
        get_data[i] = data[i];
    }
    for (int i = len - 1; i > -1; i--){ //==0
        for (int j = 0 ; j < 3;  j++){
            tmp[--size] = get_data[i] % 10 + '0';
            get_data[i]/=10;
        }
    }
    delete[] get_data;
}

void Set_Serial::format_byte(int32_t data, uint8_t len, uint8_t size){
    int32_t *get_data = new int32_t [len]; //current data
    for (int i = 2; i < size; i++){ 
        tmp[i] = 0;
    }
    for (int i = 0; i < len; i++){
        get_data[i] = data;
    }
    for (int i = len - 1; i > -1; i--){ //==0
        for (int j = 0 ; j < 3;  j++){
            tmp[--size] = get_data[i] % 10 + '0';
            get_data[i]/=10;
        }
    }
    delete[] get_data;
}

#endif
