
/*
*            WARNING!! This code is very rough, but it works.
*
*
*            Please feel free to fix any bugs or weird glitches that you find. 
*            
*            I would love to hear any improvements and or fixes to this existing code!! Thanks!
*
*
*            Using Arduino Nano with MCP4728 DAC
*/


#include <Wire.h>
#include <MIDI.h>
#include <Adafruit_MCP4728.h>

Adafruit_MCP4728 dac;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);

const float V_PER_OCTAVE = 1.0 / 12.0;
const uint8_t GATE_PINS[4] = {6, 7, 8, 9};      //Gate output pins
const uint8_t TRIGGER_PINS[4] = {2, 4, 5, 13};  //Trigger output pins
const int MODE_SWITCH_PIN = 12;                 //Midi Channel Assign Switch
const int SWITCH_PIN = 10;                      //Mono or Polyphonic Mode Switch
const uint8_t PITCH_BEND_OUT_PIN = 11;          // Pitch bend PWM pin
const uint8_t MOD_OUT_PIN = 3;                  // Modulation PWM pin

bool unisonMode = false;
bool channelAssignMode = false;

float pitchBendAmount = 0.0;          
float modDepth = 0.0;                   
float currentModVoltage = 0.0;
float targetModVoltage = 0.0;

const unsigned long MOD_UPDATE_INTERVAL = 5; 
const float modStep = 0.05;                   
unsigned long lastModUpdate = 0;

// ---- Trigger handling ----
const unsigned long TRIGGER_PULSE_MS = 10;
unsigned long triggerStart[4] = {0, 0, 0, 0};
bool triggerActive[4] = {false, false, false, false};

void startTrigger(uint8_t index) {
  digitalWrite(TRIGGER_PINS[index], HIGH);
  triggerStart[index] = millis();
  triggerActive[index] = true;
}

void updateTriggers() {
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (triggerActive[i] && (now - triggerStart[i] >= TRIGGER_PULSE_MS)) {
      digitalWrite(TRIGGER_PINS[i], LOW);
      triggerActive[i] = false;
    }
  }
}
// ---------------------------

struct NoteSlot {
  bool active;
  uint8_t note;
  uint8_t velocity;
};

NoteSlot activeNotes[4] = {{false,0,0},{false,0,0},{false,0,0},{false,0,0}};
float lastVoltages[4] = {0,0,0,0};

int findFreeSlot() {
  for (int i=0; i<4; i++) {
    if (!activeNotes[i].active) return i;
  }
  return -1;
}

int findNoteSlot(uint8_t note) {
  for (int i=0; i<4; i++) {
    if (activeNotes[i].active && activeNotes[i].note == note) return i;
  }
  return -1;
}

void updateCV() {
  for (int i=0; i<4; i++) {
    if (activeNotes[i].active) {
      float voltage = (activeNotes[i].note - 36) * V_PER_OCTAVE;
      lastVoltages[i] = voltage;
      dac.setChannelValue((MCP4728_channel_t)i, voltage * 4095 / 5.0);
    } else {
      dac.setChannelValue((MCP4728_channel_t)i, lastVoltages[i] * 4095 / 5.0);
    }
  }
}

void handleNoteOn(byte channel, byte note, byte velocity) {
  float voltage = (note - 36) * V_PER_OCTAVE;

  if (!channelAssignMode) {
    if (unisonMode) {
      for (int i=0; i<4; i++) {
        dac.setChannelValue((MCP4728_channel_t)i, voltage * 4095 / 5.0);
        digitalWrite(GATE_PINS[i], HIGH);
        startTrigger(i);
      }
    } else {
      int slot = findFreeSlot();
      if (slot != -1) {
        activeNotes[slot] = {true, note, velocity};
        updateCV();
        digitalWrite(GATE_PINS[slot], HIGH);
        startTrigger(slot);
      }
    }
  } else {
    int index = channel - 1;
    if (index >= 0 && index < 4) {
      activeNotes[index] = {true, note, velocity};
      float modVoltage = voltage;
      if (modVoltage < 0) modVoltage = 0;
      if (modVoltage > 5.0) modVoltage = 5.0;
      dac.setChannelValue((MCP4728_channel_t)index, modVoltage * 4095 / 5.0);
      digitalWrite(GATE_PINS[index], HIGH);
      startTrigger(index);
    }
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  if (!channelAssignMode) {
    if (unisonMode) {
      for (int i=0; i<4; i++) {
        digitalWrite(GATE_PINS[i], LOW);
      }
    } else {
      int slot = findNoteSlot(note);
      if (slot != -1) {
        activeNotes[slot] = {false,0,0};
        digitalWrite(GATE_PINS[slot], LOW);
      }
      updateCV();
    }
  } else {
    int index = channel - 1;
    if (index >= 0 && index < 4) {
      activeNotes[index].active = false;
      digitalWrite(GATE_PINS[index], LOW);
    }
  }
}

void handlePitchBend(byte channel, int bend) {
  const int CENTER = 8192;
  const float SEMITONE_RANGE = 5.0;

  pitchBendAmount = ((float)bend - CENTER) / CENTER * SEMITONE_RANGE * V_PER_OCTAVE;
  updatePitchBendOutput();
}

void updatePitchBendOutput() {
  float voltage = 1.0 + pitchBendAmount;  

  if (voltage < 0) voltage = 0;
  if (voltage > 5.0) voltage = 5.0;

  uint8_t pwmValue = (uint8_t)(voltage * 255 / 5.0);
  analogWrite(PITCH_BEND_OUT_PIN, pwmValue);
}

void handleControlChange(byte channel, byte control, byte value) {
  if (control == 1) {  
    modDepth = value / 127.0;
    targetModVoltage = modDepth * 5.0;
  }
}

void updateModWheelOutputSmooth() {
  if (fabs(currentModVoltage - targetModVoltage) < modStep) {
    currentModVoltage = targetModVoltage;
  } else if (currentModVoltage < targetModVoltage) {
    currentModVoltage += modStep;
  } else {
    currentModVoltage -= modStep;
  }

  if (currentModVoltage < 0) currentModVoltage = 0;
  if (currentModVoltage > 5.0) currentModVoltage = 5.0;

  uint8_t pwmValue = (uint8_t)(currentModVoltage * 255 / 5.0);
  analogWrite(MOD_OUT_PIN, pwmValue);
}

void setup() {
  Serial.begin(31250);
  MIDI.begin(MIDI_CHANNEL_OMNI);

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);

  Wire.begin();
  if (!dac.begin()) {
    Serial.println("Failed to initialize MCP4728!");
    while (1);
  }

  for (uint8_t i=0; i<4; i++) {
    pinMode(GATE_PINS[i], OUTPUT);
    pinMode(TRIGGER_PINS[i], OUTPUT);
    digitalWrite(GATE_PINS[i], LOW);
    digitalWrite(TRIGGER_PINS[i], LOW);
  }

  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  pinMode(PITCH_BEND_OUT_PIN, OUTPUT);
  pinMode(MOD_OUT_PIN, OUTPUT);

  analogWrite(PITCH_BEND_OUT_PIN, 0);
  analogWrite(MOD_OUT_PIN, 0);
}

void loop() {
  channelAssignMode = digitalRead(MODE_SWITCH_PIN) == LOW;
  unisonMode = digitalRead(SWITCH_PIN) == LOW;

  MIDI.read();

  unsigned long now = millis();
  if (now - lastModUpdate >= MOD_UPDATE_INTERVAL) {
    lastModUpdate = now;
    updateModWheelOutputSmooth();
  }

  updateTriggers();
}
