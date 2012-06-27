#include <SPI.h>
#include <ctype.h>
#include <stdlib.h>

/* LADY GAGA VERSION 2 VALVE CONTROL SKETCH
 * (C) 2012 Marc Gershow
 * Requires arduino v 1.01 or higher & teensyduino add-on
 * This work is licensed under the Creative Commons Attribution-* NonCommercial-ShareAlike 3.0 Unported License. To view a copy
* of this license, visit http://creativecommons.org/licenses/by-
* nc-sa/3.0/ or send a letter to Creative Commons, 444 Castro
* Street, Suite 900, Mountain View, California, 94041, USA.
 */

/* Serial Commands
 * all commands on one line, terminate with \n or \r
 * T xx - Time per cycle step (in ms) - minimum time valve is open or closed during gradient creation (default 15 ms; minimum 5 ms, max 4 seconds); xx is decimal float
 * N xx - Num valves - set number of valves to xx; xx can be dec, hex (leading 0x - zero x), or oct (leading 0 - zero) [default 40 valves]
 * F - Forward - turn gradient on forward
 * R - Reverse - turn gradient on reverse
 * M - Mixing - turn all valves on 50% of the time
 * O - all open - turn all valves on [note valves are 3-way so active/inactive is better terminology]
 * C - all close - turn all valves off [note valves are 3-way so active/inactive is better terminology]
 * V xx xx xx - set Valves to - each xx is one byte; should be one xx for each valve board, arranged MSB to LSB; xx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * S - query valve state - returns hex representation of current valve state
 * Q - quiet - only respond to errors [default loud]
 * L - loud - acknowledge all commands [default]
 * H - help - right, good luck with that guy
 */
 
 /* arrangement of valves - valve 0 is valve labeled 1 on board containing MCU
 *               valve (NUM_BOARDS*8)-1 is valve labeled 8 on board farthest from MCU
 *               valvestates[0] is state of valves farthest from MCU
 *               valvestates[NUM_BOARDS-1] is state of valves on MCU board
 * thus valvestates looks like
 *    (last valve on far board) VALVESTATES (first valve on mcu board)
 */

//to do: implement detection and sleep on PE2 to avoid conflicts if more than one board is populated with MCU

const int MAX_BOARDS = 32;
const int SERIAL_CHARS_TO_BUFFER = 256;
const int ssPin = PIN_B0;
const byte writeValveCode = 0b11000111;//0xC7; //high byte to write to valves = 11000111

//MIN_VALVE_TIME_IN_MILLIS is the minimum length of time a valve will be open or closed for during
//gradient creation. to assure linearity, should be larger than sum of open&close times
const float MIN_VALVE_TIME_IN_MILLIS = 5.0; 
const float MAX_VALVE_TIME_IN_MILLIS = 4000;
volatile float valveTimeInMillis = 15.0;
const boolean debug = false;

byte valveStates[MAX_BOARDS];
volatile int numBoards = 5; //defaults for lady gaga
volatile int numValves = 40;
volatile int cycleNumber = 0;

enum gradientState {grad_forward, grad_reverse, grad_mixed, grad_none};
volatile gradientState gradientDirection = grad_forward;
volatile boolean verbose = true;



void setup() {
  //first, turn off the clock prescaler, to get 16MHz operation
  noInterrupts();
  CLKPR = _BV(CLKPCE);
  CLKPR = 0;
  
  // set the slaveSelefctPin as an output:
  pinMode (ssPin, OUTPUT);
  digitalWrite(ssPin, HIGH);
  SPI.begin(); 
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  for (int j = 0; j < MAX_BOARDS; ++j) {
    valveStates[j] = 0;
  }
  interrupts();
  
  Serial.begin(9600);
  setupTimerCounter();
  
  for (int j = 0; j < 3; ++j) {
    setAllValves(true); updateValves();
    delay(500);
    setAllValves(false); updateValves();
    delay(500);
  }
  
  
}

void setNumValves(int nv) {
  numValves = nv < MAX_BOARDS*8 ? nv : MAX_BOARDS*8;
  numBoards = (numValves+7)/8;
}
void setValveTime(float time) {
  time = time < MIN_VALVE_TIME_IN_MILLIS ? MIN_VALVE_TIME_IN_MILLIS : time;
  time = time > MAX_VALVE_TIME_IN_MILLIS ? MAX_VALVE_TIME_IN_MILLIS : time;
  valveTimeInMillis = time;
  setupTimerCounter();
}
  

void loop() {
  serialPoll();
}

void serialPoll() {
  static char buffer[SERIAL_CHARS_TO_BUFFER+1];
  buffer[SERIAL_CHARS_TO_BUFFER] = (char) '\0';
  static int len = 0;
  while (Serial.available() && len < (SERIAL_CHARS_TO_BUFFER)) {
    char c = Serial.read();
    if (isspace(c) && len <= 0) { //trim any leading whitespace
      continue;
    }
    if ('\n' == c || '\r' == c) {
     buffer[len] = '\0';
     len = 0;
    
     if (executeSerialCommand(buffer)) {
       Serial.print("Error: could not parse/execute your command: ");
       Serial.println(buffer);
     }
     continue;
    }
    buffer[len++] = c;
  }
  if (len >= SERIAL_CHARS_TO_BUFFER) {
    Serial.print("E: Serial Buffer Overrun: ");
    buffer[SERIAL_CHARS_TO_BUFFER] = '\0';
    Serial.println(buffer);
    len = 0;
  }
}

void printHelp() {
  Serial.println ("look in the sketch for help!");
}


/* Serial Commands
 * all commands on one line, terminate with \n or \r
 * T xx - Time per cycle step (in ms) - minimum time valve is open or closed during gradient creation (default 15 ms; minimum 5 ms, max 4 seconds); xx is decimal float
 * N xx - Num valves - set number of valves to xx; xx can be dec, hex (leading 0x - zero x), or oct (leading 0 - zero) [default 40 valves]
 * F - Forward - turn gradient on forward
 * R - Reverse - turn gradient on reverse
 * M - Mixing - turn all valves on 50% of the time
 * O - all open - turn all valves on [note valves are 3-way so active/inactive is better terminology]
 * C - all close - turn all valves off [note valves are 3-way so active/inactive is better terminology]
 * V xx xx xx - set Valves to - each xx is one byte; should be one xx for each valve board, arranged MSB to LSB; xx can be dec, hex (0x - zero x), or oct (leading 0 - zero)
 * S - query valve state - returns hex representation of current valve state
 * Q - quiet - only respond to errors [default loud]
 * L - loud - acknowledge all commands [default]
 * H - help - right, good luck with that guy
 */


boolean executeSerialCommand(char *command) {
  boolean err = false;
  while (isspace(command[0])) {
    ++command;
  }
  unsigned char sreg = SREG;
  noInterrupts();
  switch(toupper(command[0])) {
    case 'N': 
      err = setNumValvesFromString(command+1);
      if (!err && verbose) {
        Serial.print("OK: Set up for "); Serial.print(numValves); Serial.print(" valves on "); Serial.print(numBoards); Serial.println (" boards"); 
      }
      break;
    case 'T': 
      err = setValveTimeFromString(command+1);
      if (!err && verbose) {
        Serial.print("OK: Set valve time to "); Serial.print(valveTimeInMillis);Serial.println (" ms"); 
      }
      break;
    case 'F':
      gradientDirection = grad_forward;
      enableGradientInterrupt();
      if (verbose) Serial.println("OK: gradient forward");
      break;
    case 'R':
      gradientDirection = grad_reverse;
      enableGradientInterrupt();
      if (verbose) Serial.println("OK: gradient reverse");
      break;
    case 'M':
      gradientDirection = grad_mixed;
      enableGradientInterrupt();
      if (verbose) Serial.println("OK: mixing");
      break;
    case 'O':
      disableGradientInterrupt(); gradientDirection = grad_none;
      setAllValves(true); 
      updateValves();
      if (verbose) Serial.println("OK: all valves activated");
      break;
    case 'C':
      disableGradientInterrupt(); gradientDirection = grad_none;
      setAllValves(false);
      updateValves();
      if (verbose) Serial.println("OK: all valves inactivated");
      break;
    case 'Q':
      verbose = false;
      break;
    case 'L':
      verbose = true;
      Serial.println ("OK: I'll be chatty");
      break;
    case '?': case 'H':
      printHelp();
      break;
    case 'S':
      echoValveState();
      break;
    case 'V':
      disableGradientInterrupt(); gradientDirection = grad_none;
      err = setValvesFromString(command+1);
      if (!err) {
        updateValves();
      }
      if (!err && verbose) {
        Serial.print("OK: ");
        echoValveState();
      }
      break;
    default:
      Serial.println("unrecognized command");
      err = true;
      break;
  }
  SREG = sreg;
  return err;
}
      
void echoValveState() {
    Serial.print("Valves set to:");
    for  (int j = 0; j < numBoards; ++j) {
      Serial.print(" ");
      Serial.print(valveStates[j], HEX);  
    }
    Serial.println();
}


void setupTimerCounter() {
  float timercts =  (valveTimeInMillis*F_CPU/1000.0);
  int prescalerValues[] = {1,8,64,256,1024};
  byte prescalerBytes[] = {1,2,3,4,5};
  int ps;
  for (ps = 0; ps < 5; ps++) {
    if (timercts/prescalerValues[ps] <= 0xFFFF) {
      break;
    }
  } 
  if (debug) {
    Serial.print("prescaler value = "); Serial.println(prescalerValues[ps]);
    Serial.print("prescaler bytes = "); Serial.println(prescalerBytes[ps]);
  }
  unsigned char sreg = SREG;
  noInterrupts();

  TCCR3A = 0;
  TCCR3C = 0;
  TCCR3B = _BV(WGM32); //set to CTC mode; stop counter
  
  //set timer counter max value
  //max valve time is therefore 65,535*1024/16MHZ = 4 seconds 

  unsigned int timerval = (timercts/prescalerValues[ps] + 0.5) < 0xFFFF ? (unsigned int) (timercts/prescalerValues[ps] + 0.5) : 0xFFFF;  
  if (debug) {
    Serial.print("timercts = "); Serial.println(timercts);
    Serial.print("timerval = "); Serial.println(timerval);
  }
  OCR3A = timerval;
  
  if (debug) {
    ICR3 = timerval; //not used, but a test for 16 bit register working
    unsigned int ocr3aval = OCR3A;
    unsigned int icr3val = ICR3;
    Serial.print("TCCR3B = "); Serial.println(TCCR3B, BIN);
    Serial.print("ICR3 = "); Serial.println(icr3val);
    
    Serial.print("OCR3A = "); Serial.println(ocr3aval);
    Serial.print("OCR3AL = "); Serial.println(OCR3AL);
    Serial.print("OCR3AH = "); Serial.println(OCR3AH);
  }

  TCCR3B |= prescalerBytes[ps]; //start clock
 
  if (debug) {
    Serial.print("TCCR3B = "); Serial.println(TCCR3B, BIN);
    Serial.print("OCR3AL = "); Serial.println(OCR3AL);
    Serial.print("OCR3AH = "); Serial.println(OCR3AH);
  
  }

  
  SREG = sreg;
}

void enableGradientInterrupt() {
  TIMSK3 |= _BV(OCIE3A ); // Enable CTC interrupt
  interrupts(); // make sure global interrupts are on
}
void disableGradientInterrupt() {
  TIMSK3 &= ~_BV(OCIE3A ); // disable CTC interrupt
}

void gradientStep() {
  updateValves(); //send previously computed valve code
  if (0 && debug) {
    Serial.print("cycle number = "); Serial.println(cycleNumber);
    echoValveState();
  }
  setGradientForNextCycle (); //compute valve code for next cycle
  if (debug) {
    unsigned int tcnt3 = TCNT3;
    unsigned int ocr3 = OCR3A;
    Serial.print("tcnt3 = "); Serial.print(tcnt3); Serial.print(" / "); Serial.println(ocr3);
  }
}

ISR ( TIMER3_COMPA_vect ) {
  gradientStep();
}



void sendValveCode (byte valveCode[], int numbytes) {
  digitalWrite(ssPin, LOW);
  for (int j = 0; j < numbytes; ++j) {
    SPI.transfer(writeValveCode);
    SPI.transfer(valveCode[j]);
   
  }
  digitalWrite(ssPin, HIGH);
  
}
void setAllValves (boolean on) {
  byte blankstate = on ? 0xFF : 0;
  for (int j = 0; j < numBoards; ++j) {
    valveStates[j] = blankstate;
  }
  
}
void setGradientForNextCycle () {
  //recalculate the valve display from scratch, using algorithm developed in lady gaga paper
  int j;
  int numCycles = numValves - 1;
  cycleNumber = (cycleNumber+1)%(numCycles);
  if (grad_none == gradientDirection) {
    return;
  }
  if (grad_mixed == gradientDirection) {
    static byte b = 0b01010101;
    //switch 3 times per cycle; each state gets equal time no matter what factorization of numCycles is
    if (cycleNumber == 0 || cycleNumber == numCycles/3 || cycleNumber == 2*numCycles/3) {
      b = ~b;
    }
    for (j = 0; j < numBoards; ++j) {
      valveStates[j] = b;
    } 
    return;
  }
      
    
  boolean forward;
  forward = (grad_forward == gradientDirection);
  setAllValves(!forward);
  if (false && debug) {
    Serial.print("forward = "); Serial.println(forward);
  }
  int startcycle = 0;
  int endcycle = 0;
  for (int vn = 1; vn < numValves; ++vn) {
    endcycle = (startcycle + vn)%numCycles;
    
    if (endcycle > startcycle) {
       if (cycleNumber >= startcycle && cycleNumber < endcycle) {
         setValveState(vn, forward);
         if (false && debug) {
           Serial.print(vn); Serial.print(" ");
         }
       }
    } else {
      if (cycleNumber >= startcycle || cycleNumber < endcycle) {
        setValveState(vn, forward);
        if (false && debug) {
           Serial.print(vn); Serial.print(" ");
         }
      }       
    }
    startcycle = (endcycle)%numCycles;
  }
  if (false && debug) {
    Serial.println();
  }
}


boolean setValveTimeFromString (char *valvestring) {
  char *vs = valvestring;
  int numread = 0;
  double d = strtod(valvestring, &valvestring);
  if (valvestring == vs) {
      return true;
  }
  setValveTime (d);
  return false;
}

boolean setNumValvesFromString (char *valvestring) {
  char *vs = valvestring;
  int numread = 0;
  long l = strtol(valvestring, &valvestring, 0);
  if (valvestring == vs) {
      return true;
  }
  setNumValves (l);
  return false;
}
    

boolean setValvesFromString (char *valvestring) {
  char *vs = valvestring;
  int numread;
  numread = 0;
  while (vs[0] != 0) {
    long l = strtol(valvestring, &valvestring, 0);
    if (valvestring == vs) {
      //nothing was read
      break;
    }
    vs = valvestring;
    if (l > 0xFF) {
      Serial.print("Error: value out of range - send bytes individually separated by spaces (problem value = ");
      Serial.print(l);
      Serial.println(" )");
      return true;
    }
    if (numread > numBoards) {
      Serial.println("Warning: you sent more bytes than there are boards");
      return true;
    }
    valveStates[numread++] = l;
  }
  if (numread < numBoards) {
    Serial.println("Warning: you send fewer bytes than there are boards");
    return true;
  }
  return false;
}

void setValveState(int valveNumber, boolean state) {
  /*
  int whichBank;
  int whichValve;
  if (valveNumber < 0 || valveNumber >= numValves) {
    return;
  }
  whichBank = numBoards - 1 - valveNumber/8;
  whichValve = valveNumber%8;
  bitWrite(valveStates[whichBank], whichValve, state);
  */
  if (valveNumber < 0 || valveNumber >= numValves) {
    return;
  }
  bitWrite(valveStates[numBoards - 1 - valveNumber/8], valveNumber%8, state);
  
  /*
  if (state) {
    valveStates[whichBank] |= _BV(whichValve);
  } else {
    valveStates[whichBank] &= ~_BV(whichValve);
  }
  */
}

void updateValves() {
  sendValveCode(valveStates, numBoards);
}



