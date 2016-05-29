#include <TimerOne.h>                                 // Header file for TimerOne library
#include <MIDIUSB.h>

// Timer part

#define trigPin 12                                    // Pin 12 trigger output
#define echoPin 2                                     // Pin 2 Echo input
#define onBoardLED 13                                 // Pin 13 onboard LED
#define echo_int 0                                    // Interrupt id for echo pulse

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 500                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = TICK_COUNTS;        // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0;              // Count down counter for flashing distance LED



float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve);

//User definition
#define NOTE_TOTAL_NUMBER 12           //min 1 max 14
#define POTSCALE_TOTAL_NUMBER 6       //min max
#define POTNOTE_TOTAL_NUMBER 36       //firstnote pot
#define DIST_MIN 1                    //2cm*58.2 of a bottom dead bound
#define DIST_MAX 50                   //50cm*58.2 of the upper limit
#define IST 2                         //accuracy of the isteresys cycle on the 'distance to note interval'

//MIDI Definition
void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

//MUSIC NOTES ARRAY
int firstNote = 0;
int musicNotesArray[NOTE_TOTAL_NUMBER];

int minor_pentatonic[]  = {0, 3, 5, 7, 10, 12, 15, 17, 19, 22, 24, 27, 29, 31};
int major_pentatonic[]  = {0, 2, 4, 7, 9, 12, 14, 16, 19, 21, 24, 26, 28, 31};
int minor_blues[]       = {0, 3, 5, 6, 7, 10, 12, 15, 17, 18, 19, 22, 24, 27};
int major_blues[]       = {0, 2, 3, 4, 7, 9, 12, 14, 15, 16, 19, 21, 24, 26};
int sol_arm_min[]       = {0, 1, 4, 5, 7, 8, 10, 12, 13, 16, 17, 19, 20, 22, 24};
int major[]             = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 17, 19, 21, 23, 24};

int* multiScales[POTSCALE_TOTAL_NUMBER] = {minor_pentatonic, major_pentatonic, minor_blues, major_blues, sol_arm_min, major};

bool play = true;


//Definition of the 3 analog measure we need, as interval, in such a way we can apply the isteresys function instead of the MAP function
int intervalDistance = 0;
int intervalDistance_o = 0;
int DistanceOld = 0;

int intervalPotNote = 0;
int intervalPotNote_o = 0;
bool PotNotePlay = false;
long PotNoteTime = 0;
int PotNoteOld = 0;

int intervalPotScale = 0;
int intervalPotScale_o = 0;
bool PotScalePlay = false;

int thresholdDistance[NOTE_TOTAL_NUMBER];
int thresholdPotNote[POTNOTE_TOTAL_NUMBER];
int thresholdPotScale[POTSCALE_TOTAL_NUMBER];

int istRead(int interval, int inputValue, int thresholdArray[], int inputMax, int inputMin, int ist, int intervalTotal) {

  for (int z = 0; z < intervalTotal; z++) {
    thresholdArray[z] = (z + 1) * (inputMax - inputMin) / intervalTotal + inputMin;
    if (z == interval) {
      thresholdArray[z] += ist;
    }
  }

  if ((inputValue <= inputMax) && (inputValue >= inputMin)) {
    if (inputValue < thresholdArray[0]) {
      interval = 0;
    }
    else {
      for (int k = 1; k < intervalTotal; k++) {
        if ((inputValue >= thresholdArray[k - 1]) && (inputValue < thresholdArray[k])) {
          interval = k;
        }
      }
    }
  }
  else {
    interval = -1;
  }
  return interval;

}


// ----------------------------------
// setup() routine called first.
// A one time routine executed at power up or reset time.
// Used to initialise hardware.
// ----------------------------------
void setup()
{
  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  pinMode(onBoardLED, OUTPUT);                        // Onboard LED pin set to output

  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input

  //isteresys part
  for (int i = 0; i < NOTE_TOTAL_NUMBER; i++) {
    thresholdDistance[i] = 0;
  }
  for (int ii = 0; ii < POTNOTE_TOTAL_NUMBER; ii++) {
    thresholdPotNote[ii] = 0;
  }
  for (int iii = 0; iii < POTSCALE_TOTAL_NUMBER; iii++) {
    thresholdPotScale[iii] = 0;
  }

  Serial.begin(9600);
}



// ----------------------------------
// loop() Runs continuously in a loop.
// This is the background routine where most of the processing usualy takes place.
// Non time critical tasks should be run from here.
// ----------------------------------

volatile long c = 0;

void loop()
{

  intervalDistance_o = intervalDistance;
  intervalDistance = istRead(intervalDistance, (echo_duration / 58.2), thresholdDistance, DIST_MAX, DIST_MIN, 2, NOTE_TOTAL_NUMBER);

  intervalPotNote_o = intervalPotNote;
  intervalPotNote = istRead(intervalPotNote, analogRead(1), thresholdPotNote, 678, 0, 2, POTNOTE_TOTAL_NUMBER);

  intervalPotScale_o = intervalPotScale;
  intervalPotScale = istRead(intervalPotScale, analogRead(0), thresholdPotScale, 678, 0, 5, POTSCALE_TOTAL_NUMBER);

  //intervals to values
  firstNote = 48 + intervalPotNote;

  for (int c = 0; c < NOTE_TOTAL_NUMBER; c++) {
    musicNotesArray[c] = firstNote + multiScales[intervalPotScale][c];
  }

  //values to midi
  //  POT NOTES
  if (intervalPotNote_o != intervalPotNote) {
    if (!PotNotePlay) {
      noteOff(1, DistanceOld, 64);
      PotNoteOld = firstNote;
      noteOn(1, PotNoteOld, 64);
      PotNotePlay = true;
      PotNoteTime = millis();
      intervalPotNote = intervalPotNote_o;
    }
    else {
      if ((millis() - PotNoteTime) > 50) {
        noteOff(1, PotNoteOld, 64);
        PotNotePlay = false;
      }
      else {
        intervalPotNote = intervalPotNote_o;
      }
    }
  }
  //  POT SCALES
  else if (intervalPotScale_o != intervalPotScale) {
    noteOff(1, DistanceOld, 64);
    PotScalePlay = true;
    while (PotScalePlay) {
      noteOn(1, musicNotesArray[0], 64);
      MidiUSB.flush();
      delay(100);
      for (int p = 1; p < NOTE_TOTAL_NUMBER; p++) {
        noteOff(1, musicNotesArray[p - 1], 64);
        noteOn(1, musicNotesArray[p], 64);
        MidiUSB.flush();
        delay(100);
      }
      noteOff(1, musicNotesArray[NOTE_TOTAL_NUMBER - 1], 64);
      MidiUSB.flush();
      delay(100);
      PotScalePlay = false;
    }
  }

  else if (intervalDistance_o != intervalDistance) {
    if (intervalDistance_o == -1) {
      noteOn(1, musicNotesArray[intervalDistance], 64);
      DistanceOld = musicNotesArray[intervalDistance];
    }
    else if (intervalDistance == -1) {
      noteOff(1, musicNotesArray[intervalDistance_o], 64);
    }
    else {
      noteOn(1, musicNotesArray[intervalDistance], 64);
      MidiUSB.flush(); //just for legato
      noteOff(1, musicNotesArray[intervalDistance_o], 64);
      DistanceOld = musicNotesArray[intervalDistance];
    }
  }

  MidiUSB.flush();

  
}

// --------------------------
// timerIsr() 50uS second interrupt ISR()
// Called every time the hardware timer 1 times out.
// --------------------------
void timerIsr()
{
  trigger_pulse();                                 // Schedule the trigger pulses
}

// --------------------------
// trigger_pulse() called every 50 uS to schedule trigger pulses.
// Generates a pulse one timer tick long.
// Minimum trigger pulse width for the HC-SR04 is 10 us. This system
// delivers a 50 uS pulse.
// --------------------------
void trigger_pulse()
{
  static volatile int state = 0;                 // State machine variable
  trigger_time_count = trigger_time_count - 1;
  if (trigger_time_count == 0)                   // Count to 200mS
  { // Time out - Initiate trigger pulse
    trigger_time_count = TICK_COUNTS;           // Reload
    state = 1;                                  // Changing to state 1 initiates a pulse
  }

  switch (state)                                 // State machine handles delivery of trigger pulse
  {
    case 0:                                      // Normal state does nothing
      break;

    case 1:                                      // Initiate pulse
      digitalWrite(trigPin, HIGH);              // Set the trigger output high
      state = 2;                                // and set state to 2
      break;

    case 2:                                      // Complete the pulse
      digitalWrite(trigPin, LOW);               // Set the trigger output low
      state = 0;                                // and return state to normal 0
      break;
  }
}

// --------------------------
// echo_interrupt() External interrupt from HC-SR04 echo signal.
// Called every time the echo signal changes state.
//
// Note: this routine does not handle the case where the timer
//       counter overflows which will result in the occassional error.
// --------------------------
void echo_interrupt()
{

  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;

    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      echo_duration = echo_end - echo_start;        // Calculate the pulse duration
      break;
  }
}

