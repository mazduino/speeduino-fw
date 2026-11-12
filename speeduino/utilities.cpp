/*
  Speeduino - Simple engine management for the Arduino Mega 2560 platform
  Copyright (C) Josh Stewart
  A full copy of the license may be found in the projects root directory
*/
/** @file
 * Custom Programmable I/O.
 * The config related to Programmable I/O is found on configPage13 (of type @ref config13).
 */
#include <avr/pgmspace.h>
#include "globals.h"
#include "utilities.h"
#include "decoders.h"
#include "comms.h"
#include "logger.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "speeduino.h"

uint8_t ioDelay[sizeof(configPage13.outputPin)];
uint8_t ioOutDelay[sizeof(configPage13.outputPin)];
uint8_t pinIsValid = 0;
uint8_t currentRuleStatus = 0;


/** Translate between the pin list that appears in TS and the actual pin numbers.
For the **digital IO**, this will simply return the same number as the rawPin value as those are mapped directly.
For **analog pins**, it will translate them into the correct internal pin number.
* @param rawPin - High level pin number
* @return Translated / usable pin number
*/
#if defined(MEGA100_LAYOUT)
// Mega100: an STM32F407 on the Arduino Mega footprint (STM32F407 Mega converter v1.3).
// Board mappings and TunerStudio pin lists use Arduino Mega pin numbers; this table turns
// Mega D0..D53 into the STM32 pin on that header position. D21 and D46 are not wired, and
// D27 is PB2 (BOOT1), which the black_F407VX variant of the STM32 core does not expose; these
// three are MEGA100_NOT_WIRED, which the STM32 core treats as no pin (NC).
// A0..A15 need no table: the converter wires them to PA0-PA7, PB0, PB1, PC0-PC5, which
// is the order of the STM32 core's A0..A15.
#define MEGA100_NOT_WIRED 0xFF
volatile uint32_t mega100UnwiredPort = 0; //Port register for MEGA100_NOT_WIRED pins, see board_stm32_official.h
static const byte MEGA100_DIGITAL_PINS[54] = {
  PA10, PA9,  PD7,  PB6,  PB12, PB13, PB14, PB15, // D0-D7
  PD8,  PD9,  PD10, PD11, PA8,  PB3,  PD5,  PD6,  // D8-D15
  PB10, PB11, PD4,  PD3,  PB4,  MEGA100_NOT_WIRED, PC9, PC8, // D16-D23
  PA15, PB5,  PC7,  MEGA100_NOT_WIRED, PC6, PE1, PD14, PD15, // D24-D31
  PD12, PD13, PE5,  PE15, PE4,  PE14, PE3,  PE13, // D32-D39
  PE2,  PE12, PE10, PB7,  PE0,  PE11, MEGA100_NOT_WIRED, PE9, // D40-D47
  PE6,  PE8,  PE7,  PB8,  PC13, PB9,              // D48-D53
};
static const byte MEGA_MAX_DIGITAL_PINS = 54U; // As BOARD_MAX_DIGITAL_PINS on the Mega 2560

byte mega100Pin(byte megaPin)
{
  if (megaPin < MEGA_MAX_DIGITAL_PINS) { return MEGA100_DIGITAL_PINS[megaPin]; }
  return megaPin; // Ax references are already STM32 pin numbers
}

byte pinTranslate(byte rawPin)
{
  // Same numbering as the Mega: 0-53 digital, then A8 onwards.
  if(rawPin > MEGA_MAX_DIGITAL_PINS) { return A8 + (rawPin - MEGA_MAX_DIGITAL_PINS - 1); }
  return mega100Pin(rawPin);
}
#else
byte pinTranslate(byte rawPin)
{
  byte outputPin = rawPin;
  if(rawPin > BOARD_MAX_DIGITAL_PINS) { outputPin = A8 + (outputPin - BOARD_MAX_DIGITAL_PINS - 1); }

  return outputPin;
}
#endif
/** Translate a pin number (0 - 22) to the relevant Ax (analog) pin reference.
* This is required as some ARM chips do not have all analog pins in order (EG pin A15 != A14 + 1).
* */
byte pinTranslateAnalog(byte rawPin)
{
  byte outputPin = rawPin;
  switch(rawPin)
  {
    case 0: outputPin = A0; break;
    case 1: outputPin = A1; break;
    case 2: outputPin = A2; break;
    case 3: outputPin = A3; break;
    case 4: outputPin = A4; break;
    case 5: outputPin = A5; break;
    case 6: outputPin = A6; break;
    case 7: outputPin = A7; break;
    case 8: outputPin = A8; break;
    case 9: outputPin = A9; break;
    case 10: outputPin = A10; break;
    case 11: outputPin = A11; break;
    case 12: outputPin = A12; break;
    case 13: outputPin = A13; break;
  #if BOARD_MAX_ADC_PINS >= 14
      case 14: outputPin = A14; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 15
      case 15: outputPin = A15; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 16
      case 16: outputPin = A16; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 17
      case 17: outputPin = A17; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 18
      case 18: outputPin = A18; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 19
      case 19: outputPin = A19; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 20
      case 20: outputPin = A20; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 21
      case 21: outputPin = A21; break;
    #endif
    #if BOARD_MAX_ADC_PINS >= 22
      case 22: outputPin = A22; break;
    #endif
  }

  return outputPin;
}


void setResetControlPinState(void)
{
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);

  /* Setup reset control initial state */
  switch (resetControl)
  {
    case RESET_CONTROL_PREVENT_WHEN_RUNNING:
      /* Set the reset control pin LOW and change it to HIGH later when we get sync. */
      digitalWrite(pinResetControl, LOW);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    case RESET_CONTROL_PREVENT_ALWAYS:
      /* Set the reset control pin HIGH and never touch it again. */
      digitalWrite(pinResetControl, HIGH);
      BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    case RESET_CONTROL_SERIAL_COMMAND:
      /* Set the reset control pin HIGH. There currently isn't any practical difference
         between this and PREVENT_ALWAYS but it doesn't hurt anything to have them separate. */
      digitalWrite(pinResetControl, HIGH);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      break;
    default:
      // Do nothing - keep MISRA happy
      break;
  }
}

//*********************************************************************************************************************************************************************************
void initialiseProgrammableIO(void)
{
  uint8_t outputPin;
  for (uint8_t y = 0; y < sizeof(configPage13.outputPin); y++)
  {
    ioDelay[y] = 0;
    ioOutDelay[y] = 0;
    outputPin = configPage13.outputPin[y];
#if defined(MEGA100_LAYOUT)
    if ( (outputPin > 0) && (outputPin < 128) ) { outputPin = pinTranslate(outputPin); } //Tune holds Mega pin numbers
#endif
    if (outputPin > 0)
    {
      if ( outputPin >= 128 ) //Cascate rule usage
      {
        BIT_WRITE(currentStatus.outputsStatus, y, BIT_CHECK(configPage13.outputInverted, y));
        BIT_SET(pinIsValid, y);
      }
      else if ( !pinIsUsed(outputPin) )
      {
        pinMode(outputPin, OUTPUT);
        digitalWrite(outputPin, BIT_CHECK(configPage13.outputInverted, y));
        BIT_WRITE(currentStatus.outputsStatus, y, BIT_CHECK(configPage13.outputInverted, y));
        BIT_SET(pinIsValid, y);
      }
      else { BIT_CLEAR(pinIsValid, y); }
    }
  }
}
/** Check all (8) programmable I/O:s and carry out action on output pin as needed.
 * Compare 2 (16 bit) vars in a way configured by @ref cmpOperation (see also @ref config13.operation).
 * Use ProgrammableIOGetData() to get 2 vars to compare.
 * Skip all programmable I/O:s where output pin is set 0 (meaning: not programmed).
 */
#if defined(MEGA100_LAYOUT)
  #define PROGRAMMABLE_OUTPUT_PIN(y) pinTranslate(configPage13.outputPin[(y)])
#else
  #define PROGRAMMABLE_OUTPUT_PIN(y) (configPage13.outputPin[(y)])
#endif
void checkProgrammableIO(void)
{
  int16_t data, data2;
  uint8_t dataRequested;
  bool firstCheck, secondCheck;

  for (uint8_t y = 0; y < sizeof(configPage13.outputPin); y++)
  {
    firstCheck = false;
    secondCheck = false;
    if ( BIT_CHECK(pinIsValid, y) ) //if outputPin == 0 it is disabled
    {
      dataRequested = configPage13.firstDataIn[y];
      if ( dataRequested > 239U ) //Somehow using 239 uses 9 bytes of RAM, why??
      {
        dataRequested -= REUSE_RULES;
        if ( dataRequested <= sizeof(configPage13.outputPin) ) { data = BIT_CHECK(currentRuleStatus, dataRequested); }
        else { data = 0; }
      }
      else { data = ProgrammableIOGetData(dataRequested); }
      data2 = configPage13.firstTarget[y];

      if ( (configPage13.operation[y].firstCompType == COMPARATOR_EQUAL) && (data == data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_NOT_EQUAL) && (data != data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_GREATER) && (data > data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_GREATER_EQUAL) && (data >= data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_LESS) && (data < data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_LESS_EQUAL) && (data <= data2) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_AND) && ((data & data2) != 0) ) { firstCheck = true; }
      else if ( (configPage13.operation[y].firstCompType == COMPARATOR_XOR) && ((data ^ data2) != 0) ) { firstCheck = true; }

      if (configPage13.operation[y].bitwise != BITWISE_DISABLED)
      {
        dataRequested = configPage13.secondDataIn[y];
        if ( dataRequested <= (REUSE_RULES + sizeof(configPage13.outputPin)) ) //Failsafe check
        {
          if ( dataRequested > 239U ) //Somehow using 239 uses 9 bytes of RAM, why??
          {
            dataRequested -= REUSE_RULES;
            data = BIT_CHECK(currentRuleStatus, dataRequested);
          }
          else { data = ProgrammableIOGetData(dataRequested); }
          data2 = configPage13.secondTarget[y];
          
          if ( (configPage13.operation[y].secondCompType == COMPARATOR_EQUAL) && (data == data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_NOT_EQUAL) && (data != data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_GREATER) && (data > data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_GREATER_EQUAL) && (data >= data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_LESS) && (data < data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_LESS_EQUAL) && (data <= data2) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_AND) && ((data & data2) != 0) ) { secondCheck = true; }
          else if ( (configPage13.operation[y].secondCompType == COMPARATOR_XOR) && ((data ^ data2) != 0) ) { secondCheck = true; }

          if (configPage13.operation[y].bitwise == BITWISE_AND) { firstCheck &= secondCheck; }
          if (configPage13.operation[y].bitwise == BITWISE_OR) { firstCheck |= secondCheck; }
          if (configPage13.operation[y].bitwise == BITWISE_XOR) { firstCheck ^= secondCheck; }
        }
      }

      //If the limiting time is active(>0) and using maximum time
      if (BIT_CHECK(configPage13.kindOfLimiting, y))
      {
        if(firstCheck)
        {
          if ((configPage13.outputTimeLimit[y] != 0) && (ioOutDelay[y] >= configPage13.outputTimeLimit[y])) { firstCheck = false; } //Time has counted, disable the output
        }
        else
        {
          //Released before Maximum time, set delay to maximum to flip the output next
          if(BIT_CHECK(currentStatus.outputsStatus, y)) { ioOutDelay[y] = configPage13.outputTimeLimit[y]; }
          else { ioOutDelay[y] = 0; } //Reset the counter for next time
        }
      }

      if ( (firstCheck == true) && (configPage13.outputDelay[y] < UINT8_MAX) )
      {
        if (ioDelay[y] >= configPage13.outputDelay[y])
        {
          bool bitStatus = BIT_CHECK(configPage13.outputInverted, y) ^ firstCheck;
          if (BIT_CHECK(currentStatus.outputsStatus, y) && (ioOutDelay[y] < configPage13.outputTimeLimit[y])) { ioOutDelay[y]++; }
          if (configPage13.outputPin[y] < 128) { digitalWrite(PROGRAMMABLE_OUTPUT_PIN(y), bitStatus); }
          else { BIT_WRITE(currentRuleStatus, y, bitStatus); }
          BIT_WRITE(currentStatus.outputsStatus, y, bitStatus);
        }
        else { ioDelay[y]++; }
      }
      else
      {
        if (ioOutDelay[y] >= configPage13.outputTimeLimit[y])
        {
          bool bitStatus = BIT_CHECK(configPage13.outputInverted, y) ^ firstCheck;
          if (configPage13.outputPin[y] < 128) { digitalWrite(PROGRAMMABLE_OUTPUT_PIN(y), bitStatus); }
          else { BIT_WRITE(currentRuleStatus, y, bitStatus); }
          BIT_WRITE(currentStatus.outputsStatus, y, bitStatus);
          if(!BIT_CHECK(configPage13.kindOfLimiting, y)) { ioOutDelay[y] = 0; }
        }
        else { ioOutDelay[y]++; }

        ioDelay[y] = 0;
      }
    }
  }
}
/** Get single I/O data var (from currentStatus) for comparison.
 * @param index - Field index/number (?)
 * @return 16 bit (int) result
 */
int16_t ProgrammableIOGetData(uint16_t index)
{
  int16_t result;
  if ( index < LOG_ENTRY_SIZE )
  {
    if(is2ByteEntry(index)) { result = word(getTSLogEntry(index+1), getTSLogEntry(index)); }
    else { result = getTSLogEntry(index); }
    
    //Special cases for temperatures
    if( (index == 6) || (index == 7) ) { result -= CALIBRATION_TEMPERATURE_OFFSET; }
  }
  else if ( index == 239U ) { result = (int16_t)max((uint32_t)runSecsX10, (uint32_t)32768); } //STM32 used std lib
  else { result = -1; } //Index is bigger than fullStatus array
  return result;
}
