#include <MIDIUSB.h>
#include <microsmooth.h>

float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve);

//Arduino definition
#define trigPin  5          // Trigger Pin
#define echoPin 2           // Echo Pin

//User definition
#define NOTE_TOTAL_NUMBER 13           //min 1 max 14
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

//Definition of the measurement from the sensor
volatile float vect = -2;

void read_distance(byte sensor_trig_pin, byte sensor_echo_pin) {

  digitalWrite(sensor_trig_pin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensor_trig_pin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sensor_trig_pin, LOW);
  vect = pulseIn(sensor_echo_pin, HIGH, 100000) / 58.2;

}

ISR(TIMER1_COMPA_vect)
{
  read_distance(trigPin, echoPin);
}

//Microsmooth
uint16_t *histDistance;
int sma_vect;


void setup()
{
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

  //distance sensor part
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  cli();                          // disable global interrupts
  TCCR1A = 0;                     // set entire TCCR1A register to 0
  TCCR1B = 0;                     // set entire TCCR1B register to 0
  OCR1A = 1024;                   // Timer 1 Output Compare Register A set to 1 second
  TCCR1B |= (1 << WGM12);         // turn on CTC mode on Timer1
  TCCR1B |= (1 << CS10);          // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);          // Set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);        // enable timer compare interrupt
  sei();                          // enable global interrupts

  //microsmooth part
  histDistance = ms_init(SMA);
}


void loop()
{

  //read intervals
  sma_vect = sma_filter(vect, histDistance);
  intervalDistance_o = intervalDistance;
  intervalDistance = istRead(intervalDistance, sma_vect, thresholdDistance, DIST_MAX, DIST_MIN, 2, NOTE_TOTAL_NUMBER);

  intervalPotNote_o = intervalPotNote;
  intervalPotNote = istRead(intervalPotNote, analogRead(1), thresholdPotNote, 678, 0, 2, POTNOTE_TOTAL_NUMBER);

  intervalPotScale_o = intervalPotScale;
  intervalPotScale = istRead(intervalPotScale, analogRead(0), thresholdPotScale, 678, 0, 5, POTSCALE_TOTAL_NUMBER);

  //intervals to values
  firstNote = 24 + intervalPotNote;

  for (int c = 0; c < NOTE_TOTAL_NUMBER; c++) {
    musicNotesArray[c] = firstNote + multiScales[intervalPotScale][c];
  }

  //values to midi
    //  POT NOTES
    if (intervalPotNote_o != intervalPotNote) {
      if (!PotNotePlay){
        noteOff(1, DistanceOld, 64);
        PotNoteOld = firstNote;
        noteOn(1, PotNoteOld, 64);
        PotNotePlay = true;
        PotNoteTime = millis();
        intervalPotNote = intervalPotNote_o;
      }
      else {
        if ((millis() - PotNoteTime) > 100) {
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
      if (intervalDistance_o == -1){
        noteOn(1, musicNotesArray[intervalDistance], 64);
        DistanceOld = musicNotesArray[intervalDistance];
      }
      else if (intervalDistance == -1){
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
