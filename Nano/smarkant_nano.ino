// #############################################
/*
 * This file is part of the HA_smarkant project
 *
 * (c) 2021 Leon Schneider, www.leon-schneider.de
 *
 * Based on the Smarkant project from Dirk Grappendorf
 * https://github.com/grappendorf/smarkant
 */

#include <Arduino.h>
#include <hardware_clock.h>
#include <system_clock.h>
#include <Wire.h>
#include <EEPROM.h>
#include <lin_processor.h>
#include <PinButton.h>

const uint8_t BUTTON_UP_PIN = 3;
const uint8_t BUTTON_DOWN_PIN = 4;
const uint8_t TABLE_UP_PIN = 8;
const uint8_t TABLE_DOWN_PIN = 5;

const int NUM_POSITION_BUTTONS = 2;
const uint16_t HEIGHT_READ_MIN = 100;
const uint16_t HEIGHT_READ_MAX = 8000;
const uint16_t HEIGHT_MIN = 700;
const uint16_t HEIGHT_MAX = 4500;
const uint16_t HEIGHT_MIN_CM = 76;
const uint16_t HEIGHT_MAX_CM = 103;
const uint16_t HEIGHT_THRESHOLD_MIN = 10;
const uint16_t HEIGHT_THRESHOLD_MAX = 200;
const uint16_t HEIGHT_THRESHOLD_DEFAULT = 50;
const uint8_t I2C_ADDRESS = 0x10;
const unsigned long SERIAL_BAUD_RATE = 115200;
const uint16_t EEPROM_ADDR_HEIGHT_THRESHOLD = 0;
const uint16_t EEPROM_ADDR_POSITIONS = sizeof(uint16_t);
const unsigned long WATCHDOG_INTERVAL_MS = 20 * 1000;
const uint8_t LIN_HEIGHT_FRAME_ID = 0x92;

enum Movement {
  STOP,
  UP,
  DOWN,
  TARGET
};

enum I2CCommand {
  I2C_CMD_NOOP,
  I2C_CMD_MOVE_STOP,
  I2C_CMD_MOVE_UP,
  I2C_CMD_MOVE_DOWN,
  I2C_CMD_MOVE_HEIGHT,
  I2C_CMD_MOVE_POSITION,
  I2C_CMD_STORE_POSITION,
  I2C_CMD_STORE_CURRENT_POSITION,
  I2C_CMD_STORE_THRESHOLD,
  I2C_CMD_READ_HEIGHT,
  I2C_CMD_READ_HEIGHT_THRESHOLD,
  I2C_CMD_READ_POSITIONS
};

PinButton btn_up(BUTTON_UP_PIN);
PinButton btn_down(BUTTON_DOWN_PIN);

uint16_t positions[] = {0, 0};
uint16_t currentHeight = 0;
uint16_t targetHeight = 0;
uint16_t heightThreshold = 0;
Movement currentMovement = STOP;
unsigned long watchdogTimeout = 0;
unsigned long positionButtonPressTime = 0;
uint8_t i2cReadCommand = I2C_CMD_NOOP;

void watchdogCheck();
void moveTable(Movement move);
void moveTableToHeight(uint16_t height);
void processLINFrame(LinFrame frame);
void storePosition(int index, uint16_t height);
uint16_t recallPosition(int index);
void storeHeightThreshold(uint16_t threshold);
void handleI2CRequest();
void handleI2CReceive(int numBytes);
void loop();
void setup();
void log(const char *str, ...);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  log("Smarkant ready...");

  hardware_clock::setup();
  lin_processor::setup();

  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  

  EEPROM.get(EEPROM_ADDR_HEIGHT_THRESHOLD, heightThreshold);
  if (heightThreshold < HEIGHT_THRESHOLD_MIN || heightThreshold > HEIGHT_THRESHOLD_MAX) {
    log("Storing default height threshold");
    storeHeightThreshold(HEIGHT_THRESHOLD_DEFAULT);
  }

  pinMode(TABLE_UP_PIN, INPUT);
  pinMode(TABLE_DOWN_PIN, INPUT);
  digitalWrite(TABLE_UP_PIN, LOW);
  digitalWrite(TABLE_DOWN_PIN, LOW);

  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(handleI2CRequest);
  Wire.onReceive(handleI2CReceive);
}

void loop() {
  watchdogCheck();

  btn_up.update();
  btn_down.update();

  if ( btn_up.isSingleClick() ) moveTableToHeight(recallPosition(0));
  if ( btn_up.isDoubleClick() ) storePosition(0, currentHeight);
  if ( btn_up.isLongClick() ) moveTable(currentMovement == STOP ? UP : STOP);

  if ( btn_down.isSingleClick() ) moveTableToHeight(recallPosition(1));
  if ( btn_down.isDoubleClick() ) storePosition(1, currentHeight);
  if ( btn_down.isLongClick() ) moveTable(currentMovement == STOP ? DOWN : STOP);
  if ( btn_down.isReleased() or btn_up.isReleased() ) moveTable(STOP);
  system_clock::loop();
  LinFrame frame;
  if (lin_processor::readNextFrame(&frame)) {
    processLINFrame(frame);
  }
}

void watchdogCheck() {
  if (currentMovement != STOP && millis() > watchdogTimeout) {
    log("Watchdog timeout");
    moveTable(STOP);
  }
}

void moveTable(Movement move) {
  if (move != STOP) {
    watchdogTimeout = millis() + WATCHDOG_INTERVAL_MS;
  }
  currentMovement = move;
  switch (move) {
    case STOP:
      targetHeight = 0;
      pinMode(TABLE_UP_PIN, INPUT);
      pinMode(TABLE_DOWN_PIN, INPUT);
      break;
    case UP:
      targetHeight = 0;
      pinMode(TABLE_UP_PIN, OUTPUT);
      pinMode(TABLE_DOWN_PIN, INPUT);
      break;
    case DOWN:
      targetHeight = 0;
      pinMode(TABLE_UP_PIN, INPUT);
      pinMode(TABLE_DOWN_PIN, OUTPUT);
      break;
    case TARGET:
      if (targetHeight != 0) {
        if (currentHeight < targetHeight - heightThreshold) {
          pinMode(TABLE_UP_PIN, OUTPUT);
          pinMode(TABLE_DOWN_PIN, INPUT);
        }
        if (currentHeight > targetHeight + heightThreshold) {
          pinMode(TABLE_UP_PIN, INPUT);
          pinMode(TABLE_DOWN_PIN, OUTPUT);
        }
      } else {
        moveTable(STOP);
      }
  }
}

void moveTableToHeight(uint16_t height) {
  if (height < HEIGHT_MIN || height > HEIGHT_MAX) {
    return;
  }
  targetHeight = height;
  moveTable(TARGET);
  log("Moving table to height %d", targetHeight);
}

void processLINFrame(LinFrame frame) {
  uint8_t id = frame.get_byte(0);
  if (id == LIN_HEIGHT_FRAME_ID) {
    uint16_t position = frame.get_byte(2);
    position <<= 8;
    position |= frame.get_byte(1);
    if (position != currentHeight && position >= HEIGHT_READ_MIN && position <= HEIGHT_READ_MAX) {
      currentHeight = position;
      log("Table height: %d", position);
    }

    if (currentMovement == TARGET) {
        if (currentHeight >= targetHeight - heightThreshold &&
            currentHeight <= targetHeight + heightThreshold) {
          moveTable(STOP);
        }
    } else if (currentMovement == UP && currentHeight >= HEIGHT_MAX) {
      moveTable(STOP);
    } else if (currentMovement == DOWN && currentHeight <= HEIGHT_MIN) {
      moveTable(STOP);
    }
  }
}

int getHightincm(int currentHeight) {
  return map(currentHeight, HEIGHT_MIN, HEIGHT_MAX, HEIGHT_MIN_CM, HEIGHT_MAX_CM);
}

void storePosition(int index, uint16_t height) {
  if (height >= HEIGHT_MIN || height <= HEIGHT_MAX) {
    log("Store position %d <= %d", index, height);
    EEPROM.put(EEPROM_ADDR_POSITIONS + (index * sizeof(uint16_t)), height);
    positions[index] = height;
  }
}

uint16_t recallPosition(int index) {
  uint16_t height = positions[index];
  log("Recall position %d => %d", index, height);
  return height;
}

void storeHeightThreshold(uint16_t threshold) {
  if (heightThreshold >= HEIGHT_THRESHOLD_MIN || heightThreshold <= HEIGHT_THRESHOLD_MAX) {
    log("Storing height threshold %d", threshold);
    heightThreshold = threshold;
    EEPROM.put(EEPROM_ADDR_HEIGHT_THRESHOLD, threshold);
  }
}

void handleI2CRequest() {
  switch (i2cReadCommand) {
    case I2C_CMD_READ_HEIGHT:
      Wire.write((const uint8_t *) & currentHeight, 2);
      break;
    case I2C_CMD_READ_HEIGHT_THRESHOLD:
      Wire.write((const uint8_t *) & heightThreshold, 2);
      break;
    case I2C_CMD_READ_POSITIONS:
      Wire.write((const uint8_t *) positions, NUM_POSITION_BUTTONS * 2);
      break;
    default:
      break;
  }
}

void handleI2CReceive(int numBytes) {
  uint8_t command = Wire.read();
  switch (command) {
    case I2C_CMD_MOVE_STOP:
      moveTable(STOP);
      break;
    case I2C_CMD_MOVE_UP:
      moveTable(UP);
      break;
    case I2C_CMD_MOVE_DOWN:
      moveTable(DOWN);
      break;
    case I2C_CMD_MOVE_HEIGHT:
      if (numBytes == 3) {
        uint16_t position = Wire.read() + (Wire.read() << 8);
        moveTableToHeight(position);
      }
      break;
    case I2C_CMD_MOVE_POSITION:
      if (numBytes == 2) {
        uint8_t index = Wire.read();
        if (index < NUM_POSITION_BUTTONS) {
          moveTableToHeight(recallPosition(index));
        }
      }
      break;
    case I2C_CMD_STORE_POSITION:
      if (numBytes == 4) {
        uint8_t index = Wire.read();
        if (index < NUM_POSITION_BUTTONS) {
          uint16_t position = Wire.read() + (Wire.read() << 8);
          storePosition(index, position);
        }
      }
      break;
    case I2C_CMD_STORE_CURRENT_POSITION:
      if (numBytes == 2) {
        uint8_t index = Wire.read();
        if (index < NUM_POSITION_BUTTONS) {
          storePosition(index, currentHeight);
        }
      }
      break;
    case I2C_CMD_STORE_THRESHOLD:
      if (numBytes == 3) {
        uint16_t threshold = Wire.read() + (Wire.read() << 8);
        storeHeightThreshold(threshold);
      }
      break;
    case I2C_CMD_READ_HEIGHT:
    case I2C_CMD_READ_HEIGHT_THRESHOLD:
    case I2C_CMD_READ_POSITIONS:
      i2cReadCommand = command;
      break;
    default:
      break;
  }
}

/**
 * https://gist.github.com/asheeshr/9004783
 */
void log(const char *str, ...)
{
  const int LOG_BUFFER_LENGTH = 32;
  int i;
  int j;
  int count = 0;
  for(i = 0; str[i] != '\0'; i++) {
    if(str[i]=='%') {
      count++;
    }
  }
  if (count == 0) {
    Serial.println (str);
    return;
  }
  char temp[LOG_BUFFER_LENGTH + 1];
  va_list argv;
  va_start(argv, str);
  for(i = 0, j = 0; str[i] != '\0'; i++)
  {
    if(str[i] == '%')
    {
      temp[j] = '\0';
      Serial.print(temp);
      j = 0;
      temp[0] = '\0';
      switch(str[++i])
      {
        case 'd':
          Serial.print(va_arg(argv, int));
          break;
        case 'l':
          Serial.print(va_arg(argv, long));
          break;
        case 'f':
          Serial.print(va_arg(argv, double));
          break;
        case 'c':
          Serial.print((char)va_arg(argv, int));
          break;
        case 's':
          Serial.print(va_arg(argv, char *));
          break;
        default:
          break;
      };
    }
    else
    {
      temp[j] = str[i];
      j = (j+1) % LOG_BUFFER_LENGTH;
      if(j==0)
      {
        temp[LOG_BUFFER_LENGTH] = '\0';
        Serial.print(temp);
        temp[0] = '\0';
      }
    }
  };
  Serial.println();
}
