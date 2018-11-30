#include <EEPROM.h>
#include <ServoMotor.h>
#include <TWI.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define COMMAND_NOP                 0b00000000 // 0b00000000 No operation
#define COMMAND_CONTROL             0b00010000 // 0b0001000E Where E is boolean enable/disable control angle get/set
#define COMMAND_CALIBRATION_MIN_GET 0b00100000 // 0b00100RXX Where X is channel index, R == 0
#define COMMAND_CALIBRATION_MIN_SET 0b00100100 // 0b00100WXX Where X is channel index, W == 1
#define COMMAND_CALIBRATION_MAX_GET 0b00101000 // 0b00101RXX Where X is channel index, R == 0
#define COMMAND_CALIBRATION_MAX_SET 0b00101100 // 0b00101WXX Where X is channel index, W == 1
#define COMMAND_CALIBRATION_SAVE    0b00100000 // 0b00100000 Store calibration values into EEPROM
#define COMMAND_ANGLE_GET           0b00110000 // 0b00110RXX Where X is channel index, R == 0
#define COMMAND_ANGLE_SET           0b00110100 // 0b00110WXX Where X is channel index, W == 1
#define COMMAND_MICROSECONDS_GET    0b00111000 // 0b00111RXX Where X is channel index, R == 0
#define COMMAND_MICROSECONDS_SET    0b00111100 // 0b00111WXX Where X is channel index, W == 1

uint16_t EEPROM_SERVO0_MIN EEMEM = 0;
uint16_t EEPROM_SERVO0_MAX EEMEM = 0;

uint16_t EEPROM_SERVO1_MIN EEMEM = 0;
uint16_t EEPROM_SERVO1_MAX EEMEM = 0;

uint16_t EEPROM_SERVO2_MIN EEMEM = 0;
uint16_t EEPROM_SERVO2_MAX EEMEM = 0;

uint16_t EEPROM_SERVO3_MIN EEMEM = 0;
uint16_t EEPROM_SERVO3_MAX EEMEM = 0;

typedef struct {
    uint8_t mask;
    uint16_t arg0;
    uint16_t arg1;
} Command_t;

static Command_t commandsBufferData[4];
static volatile uint8_t commandsBufferIndex  = 0;
static volatile uint8_t commandsBufferLength = 0;

static ServoMotor servos[4];
static volatile uint8_t command = COMMAND_NOP;
static volatile bool control = false;

void twiOnReceive();

void twiOnRequest();

int main() {
    // Initialize port for address configuration (set as input & enable internal pull-up resistors)
    DDRC  = 0x00;
    PORTC = 0xFF;

    // Initialize TWI
    TWI.setAddress((uint8_t) (0xF0 | ~(PINC & 0x0F)));// <-- read low 4 bits, use connected to ground as 1
    TWI.setOnReceiveHandler(twiOnReceive);
    TWI.setOnRequestHandler(twiOnRequest);

    // Read calibration from EEPROM & initialize servo motors
    servos[0].attach(&PORTB, PB0, EEPROM.readU16(EEPROM_SERVO0_MIN), EEPROM.readU16(EEPROM_SERVO0_MAX));
    servos[1].attach(&PORTB, PB1, EEPROM.readU16(EEPROM_SERVO1_MIN), EEPROM.readU16(EEPROM_SERVO1_MAX));
    servos[2].attach(&PORTB, PB2, EEPROM.readU16(EEPROM_SERVO2_MIN), EEPROM.readU16(EEPROM_SERVO2_MAX));
    servos[3].attach(&PORTB, PB3, EEPROM.readU16(EEPROM_SERVO3_MIN), EEPROM.readU16(EEPROM_SERVO3_MAX));

    // Global enable interrupts
    sei();

    while (true) {
        if (commandsBufferIndex < commandsBufferLength) {
            //TODO dispatch single command
            Command_t command = commandsBufferData[commandsBufferIndex++];
            if (command.mask) {
                //TODO execute command depends on type
            }
        } else {
            commandsBufferIndex  = 0;
            commandsBufferLength = 0;
        }
    }
}

void twiOnReceive() {
    command = TWI.readU08();

    if (command & COMMAND_CONTROL) {
        control = (bool) (command & 0x01);
    }
    if (command & COMMAND_CALIBRATION_MIN_SET) {
        servos[(command & 0x3)].setMIN(TWI.readU16());
    }
    if (command & COMMAND_CALIBRATION_MAX_SET) {
        servos[(command & 0x3)].setMAX(TWI.readU16());
    }
    if (command & COMMAND_CALIBRATION_SAVE) {
        EEPROM.start();

        EEPROM.writeU16(EEPROM_SERVO0_MIN, servos[0].getMIN());
        EEPROM.writeU16(EEPROM_SERVO0_MAX, servos[0].getMAX());

        EEPROM.writeU16(EEPROM_SERVO1_MIN, servos[1].getMIN());
        EEPROM.writeU16(EEPROM_SERVO1_MAX, servos[1].getMAX());

        EEPROM.writeU16(EEPROM_SERVO2_MIN, servos[2].getMIN());
        EEPROM.writeU16(EEPROM_SERVO2_MAX, servos[2].getMAX());

        EEPROM.writeU16(EEPROM_SERVO3_MIN, servos[3].getMIN());
        EEPROM.writeU16(EEPROM_SERVO3_MAX, servos[3].getMAX());

        EEPROM.flush();
    }
    if (control && command & COMMAND_ANGLE_SET) {
        servos[(command & 0x3)].setAngle(TWI.readU16());
    }
    if (control && command & COMMAND_MICROSECONDS_SET) {
        servos[(command & 0x3)].setMicroseconds(TWI.readU16());
    }
}

void twiOnRequest() {
    if (command & COMMAND_CALIBRATION_MIN_GET) {
        TWI.writeU16(servos[(command & 0x3)].getMIN());
    }
    if (command & COMMAND_CALIBRATION_MAX_GET) {
        TWI.writeU16(servos[(command & 0x3)].getMAX());
    }
    if (command & COMMAND_ANGLE_GET) {
        TWI.writeU16(servos[(command & 0x3)].getAngle());
    }
    if (command & COMMAND_MICROSECONDS_GET) {
        TWI.writeU16(servos[(command & 0x3)].getMicroseconds());
    }
}