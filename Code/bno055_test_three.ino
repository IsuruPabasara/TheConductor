#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <Adafruit_VS1053.h>
#include "VS1053EffectsTeensy41.hpp"

#define VS1053_RX  20 // This is the pin that connects to the RX pin on VS1053
#define VS1053_RESET 9 // This is the pin that connects to the RESET pin on VS1053

#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0xFF
#define MIDI_CHAN_PROGRAM 0xC0

#define FLEX_INS_PIN A14
#define FLEX_VUP_PIN A15
#define FLEX_VDN_PIN A17
#define FLEX_REP_PIN A16

#define BUTTON_GUITAR_PIN 29
#define BUTTON_PIANO_PIN 30
#define BUTTON_XYLOPHONE_PIN 31
#define BUTTON_X_PIN 32

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #include <SoftwareSerial.h>
  SoftwareSerial VS1053_MIDI(0, 2); // TX only, do not use the 'rx' side
#else
  // on a Mega/Leonardo you may have to change the pin to one that 
  // software serial support uses OR use a hardware serial port!
  #define VS1053_MIDI Serial5
#endif

uint8_t pitch_value = 60;
uint8_t velocity_value = 127; 
uint8_t channel_value = 0;
float tot_angle = 0;

uint8_t pitch_value_old = 60;
uint8_t velocity_value_old = 127; 
uint8_t channel_value_old = 0;
float tot_angle_old = 0;

int systime;

uint8_t instruments[4] = {25,2,13,12} ; 
int instrument_index = 3;
uint8_t volumes[4] = {5,31,63,127} ;
int volumes_index = 3;
int flex_threshold = 500;
bool flex_ins_status = 0;
bool flex_vup_status = 0;
bool flex_vdn_status = 0;
bool flex_rep_status = 0;

int base_flex_ins = 50;
int base_flex_up = -100;
int base_flex_dn = -130;
int base_flex_rep = 100;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


#define VS1053_MISO 12
#define VS1053_MOSI 11
#define VS1053_SCLK 13
#define VS1053_CS 10

// Adafruit_VS1053 vs1053_chip =  
//   Adafruit_VS1053(VS1053_RESET, VS1053_CS, -1, -1);

void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  VS1053_MIDI.write(inst);
}

void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max _value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min _value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  // Serial.println("Midi note on");
  // Serial.print("Channel is ");
  // Serial.println(chan);
  Serial.print("Note is ");
  Serial.println(n);
  // Serial.print("Velocity is ");
  // Serial.println(vel);
  
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  // Serial.println("Midi note off");
  // Serial.print("Channel is ");
  // Serial.println(chan);
  Serial.print("Note is ");
  Serial.println(n);
  // Serial.print("Velocity is ");
  // Serial.println(vel);
  
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void instrument_change(){
  instrument_index = (instrument_index + 1) % 4;
  midiSetInstrument(0, instruments[instrument_index]);
}

void volume_up(){
  volumes_index = (volumes_index + 1) % 4;
  midiSetChannelVolume(0, volumes[volumes_index]);
}

void volume_dn(){
  volumes_index = (volumes_index - 1) % 4;
  midiSetChannelVolume(0, volumes[volumes_index]);
}

void note_repeat(){
  midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
  Serial.println("Note repeated");
}

void persist_note(){
  midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
  Serial.println("Note persit");
}

void flex_check(){
  int flex_ins_read = analogRead(FLEX_INS_PIN) + base_flex_ins;
  int flex_vup_read = analogRead(FLEX_VUP_PIN) + base_flex_up;
  int flex_vdn_read = analogRead(FLEX_VDN_PIN) + base_flex_dn;
  int flex_rep_read = analogRead(FLEX_REP_PIN) + base_flex_rep;
  Serial.println(flex_ins_read);
  Serial.println(flex_vup_read);
  Serial.println(flex_vdn_read);
  Serial.println(flex_rep_read);
  if(flex_ins_read>flex_threshold){
    persist_note();
  }
  // if(flex_ins_read>flex_threshold && flex_ins_status==0 || flex_ins_read<flex_threshold && flex_ins_status==1){
  //   int flex_ins_status_new = flex_ins_read>flex_threshold;
  //   if(flex_ins_status_new = 1 && flex_ins_status ==0){
  //     instrument_change();
  //   }
  //   flex_ins_status = flex_ins_status_new;
  // } else if(flex_vup_read>flex_threshold && flex_vup_status==0 || flex_vup_read<flex_threshold && flex_vup_status==1){
  //   int flex_vup_status_new = flex_vup_read>flex_threshold;
  //   if(flex_vup_status_new = 1 && flex_vup_status ==0){
  //     instrument_change();
  //   }
  //   flex_vup_status = flex_vup_status_new;
  // } else if(flex_vdn_read>flex_threshold && flex_vdn_status==0 || flex_vdn_read<flex_threshold && flex_vdn_status==1){
  //   int flex_vdn_status_new = flex_vdn_read>flex_threshold;
  //   if(flex_vdn_status_new = 1 && flex_vdn_status ==0){
  //     midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
  //   }
  //   flex_vdn_status = flex_vdn_status_new;
  // } else if(flex_rep_read>flex_threshold && flex_rep_status==0 || flex_rep_read<flex_threshold && flex_rep_status==1){
  //   int flex_rep_status_new = flex_rep_read>flex_threshold;
  //   if(flex_rep_status_new = 1 && flex_rep_status ==0){
  //     note_repeat();
  //   }
  //   flex_rep_status = flex_rep_status_new;
  // } 
}

void flex_setup(void){
  int flex_ins_read = analogRead(FLEX_INS_PIN);
  int flex_vup_read = analogRead(FLEX_VUP_PIN);
  int flex_vdn_read = analogRead(FLEX_VDN_PIN) + base_flex_dn;
  int flex_rep_read = analogRead(FLEX_REP_PIN) + base_flex_rep;
  Serial.println(flex_ins_read);
  Serial.println(flex_vup_read);
  Serial.println(flex_vdn_read);
  Serial.println(flex_rep_read);
}

void setup(void)
{
  // vs1053_chip.begin();
  // vs1053_chip.setVolume(255, 255);

  SPI.begin();
  delay(3000);

  Serial.begin(115200);
  Serial.println(volumes[volumes_index]);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  Serial.println("VS1053 MIDI test");
  delay(1000);  

  vs1053Reset();
  // Serial.println("111");
  delay(1000);
  vs1053Begin();
  // Serial.println("222"); 
  delay(5);
  vs1053SetVolume(5,5);
  // Serial.println("333");

  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'

  // pinMode(FLEX_INS_PIN, INPUT);

  pinMode(BUTTON_GUITAR_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_PIANO_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_XYLOPHONE_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_X_PIN, INPUT_PULLDOWN);

  // pinMode(VS1053_RESET, OUTPUT);
  // digitalWrite(VS1053_RESET, LOW);
  // delay(10);
  // digitalWrite(VS1053_RESET, HIGH);
  // delay(10);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
  systime = millis();
  
  midiSetChannelBank(0, VS1053_BANK_MELODY);
  midiSetInstrument(0, instruments[instrument_index]); // 25,2,110. instruments[instrument_index]
  midiSetChannelVolume(0, volumes[volumes_index]);
}

String inString = ""; 

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);               

  flex_check();

  // if (digitalRead(BUTTON_GUITAR_PIN) == HIGH) {
  //   midiSetInstrument(0, 25);
  //   Serial.println("Set To Guitar");
  // }
  // else if(digitalRead(BUTTON_PIANO_PIN) == HIGH){
  //   midiSetInstrument(0, 2);
  //   Serial.println("Set To Piano");
  // }
  // else if(digitalRead(BUTTON_XYLOPHONE_PIN) == HIGH){
  //   midiSetInstrument(0, 13);
  //   Serial.println("Set To Xylophone");
  // }
  // else if(digitalRead(BUTTON_X_PIN) == HIGH){
  //   midiSetInstrument(0, 12);
  //   Serial.println("Set To Marimba");
  // }

  // for (int i = 0; i< 50;i++){
  //   midiNoteOn(channel_value, pitch_value + i, velocity_value); // Channel 0, middle C, normal velocity
  //   delay(500);
  // }

  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      channel_value = inString.toInt(); 
      Serial.println(channel_value);
      // clear the string for new input:
      inString = "";
    }
  }

  /* The processing sketch expects data as roll, pitch, heading */
  float roll_angle = (float)event.orientation.x;
  float pitch_angle = (float)event.orientation.y;
  float heading_angle = (float)event.orientation.z;
  tot_angle = roll_angle + pitch_angle + heading_angle;
  pitch_value = (uint8_t) ((tot_angle / 10) + 30);
  if((millis()-systime)>500) {
    midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity
  }

  if(abs(tot_angle - tot_angle_old)>5) {
    midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity

    midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity

    tot_angle_old = tot_angle;
    channel_value_old = channel_value;
    pitch_value_old = pitch_value;
    velocity_value_old = velocity_value;
    systime = millis();
  }
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
