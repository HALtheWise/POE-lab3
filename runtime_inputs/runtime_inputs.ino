#define NOT_AN_INTERRUPT -1


boolean checkTime(long long* last_t, int thresh){
    long long t = millis();
    if(*last_t + thresh < t){
        *last_t = t;
        return true;
    }
    // otherwise ...
    return false;
}

// on-off toggle params
volatile boolean go = false;
long long int last_onoff = millis();
const int ONOFF_PIN = 2;

void onoff(){
    if(checkTime(&last_onoff, 1000)){
        go = !go;
    }
}


//calibration params
enum {
    CALIB_NONE, CALIB_START, CALIB_TAPE, CALIB_FLOOR};
const int CALIB_NOPTS = 4;
int calib_state = CALIB_NONE;
long long int last_calib = millis();

const int CALIB_PIN = 3;

void calibrate(){

    if(checkTime(&last_calib, 1000)){

        calib_state = (calib_state + 1) % CALIB_NOPTS;
        
        switch(calib_state){
        case CALIB_NONE:   
            // end of calibration
            break;
        case CALIB_START:
            // beginning calibration, cease action
            go = false;
            break;
        case CALIB_TAPE:
            // incoming analogRead(IR_PIN) is the value for tape ...
            go = false;
            break;
        case CALIB_FLOOR:
            // incoming analogRead(IR_PIN) is the value for floor ...
            go = false;
            break;
        }
    }

}

void setup(){
    Serial.begin(9600);
    pinMode(ONOFF_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ONOFF_PIN), onoff, FALLING);

    pinMode(CALIB_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CALIB_PIN), calibrate, FALLING);
}

void loop(){
    // other stuff
}

