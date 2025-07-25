#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "VS1053EffectsTeensy41.hpp"

#define VS1053_RESET 9 // This is the pin that connects to the RESET pin on VS1053
#define VS1053_BANK_MELODY 0x79

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0xFF
#define MIDI_CHAN_PROGRAM 0xC0

#define FLEX_MAJ_PIN A14
#define FLEX_MIN_PIN A15
#define FLEX_REP_PIN A17
#define FLEX_SUS_PIN A16

#define BUTTON_INSTRUMENT 29
#define BUTTON_VOLUME 30
#define BUTTON_SENSITIVITY 31
#define BUTTON_MODE 32

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #include <SoftwareSerial.h>
  SoftwareSerial VS1053_MIDI(0, 2); // TX only, do not use the 'rx' side
#else
  // on a Mega/Leonardo you may have to change the pin to one that 
  // software serial support uses OR use a hardware serial port!
  #define VS1053_MIDI Serial5
#endif

int systime;

uint8_t pitch_value = 60;
uint8_t velocity_value = 127; 
uint8_t channel_value = 0;
float tot_angle = 0;

uint8_t pitch_value_old = 60;
uint8_t velocity_value_old = 127; 
uint8_t channel_value_old = 0;
float tot_angle_old = 0;

uint8_t instruments[3] = {1,25,14} ; 
int instrument_index = 0;
uint8_t instruments_sus[3] = {22,42,53} ; 
int instrument_sus_index = 0;
uint8_t sensitivities[2] = {10,20} ; 
int sensitivity_index = 0;
int sensitivity = sensitivities[sensitivity_index];
uint8_t volumes[4] = {32,64,98,127} ;
int volumes_index = 0;
bool mode_sus = 0;

bool flex_rep_status = 0;
bool flex_maj_status = 0;
bool flex_min_status = 0;
bool flex_sus_status = 0;

bool btn_inst_status = 0;
bool btn_vol_status = 0;
bool btn_sen_status = 0;
bool btn_mode_status = 0;

int flex_threshold = 50;
int base_flex_rep = 0;
int base_flex_maj = 0;
int base_flex_min = 0;
int base_flex_sus = 0;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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

void displaySensorDetails(void){
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

  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n+1);
  VS1053_MIDI.write(vel);

  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n+3);
  VS1053_MIDI.write(vel);

  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n+4);
  VS1053_MIDI.write(vel);

  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n+7);
  VS1053_MIDI.write(vel);
}

void refresh_note(){
  if(mode_sus==0){
    midiSetInstrument(0, instruments[instrument_index]);
  }else{
    midiSetInstrument(0, instruments_sus[instrument_sus_index]);
  }
  velocity_value = volumes[volumes_index];
  sensitivity = sensitivities[sensitivity_index];
  midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity   
  channel_value_old = channel_value;
  pitch_value_old = pitch_value;
  velocity_value_old = velocity_value;
  systime = millis();
}

void instrument_change(){
  if(mode_sus==0){
    instrument_index = (instrument_index + 1) % 3;
    midiSetInstrument(0, instruments[instrument_index]);
  }else{
    instrument_sus_index = (instrument_sus_index + 1) %3;
    midiSetInstrument(0, instruments_sus[instrument_sus_index]);
  }
  refresh_note();
}

void volume_change(){
  volumes_index = (volumes_index + 1) % 4;
  velocity_value = volumes[volumes_index];
  //midiSetChannelVolume(0, volumes[volumes_index]);
  refresh_note();
}

void sensitivity_change(){
  sensitivity_index = (sensitivity_index + 1) % 2;
  sensitivity = sensitivities[sensitivity_index];
  refresh_note();
}

void mode_change(){
  mode_sus = !mode_sus;
  refresh_note();
}

void button_check(){
  int btn_inst_read = digitalRead(BUTTON_INSTRUMENT);
  int btn_vol_read = digitalRead(BUTTON_VOLUME);
  int btn_sen_read = digitalRead(BUTTON_SENSITIVITY);
  int btn_mode_read = digitalRead(BUTTON_MODE);
  
  if(btn_inst_read==HIGH && btn_inst_status==0){
    btn_inst_status = 1;
    instrument_change();
    Serial.println("Instrument changed");
  } else if(btn_inst_read==LOW){
    btn_inst_status = 0;
  }

  if(btn_vol_read==HIGH && btn_vol_status==0){
    btn_vol_status = 1;
    volume_change();
    Serial.println("Volume changed");
  } else if(btn_vol_read==LOW){
    btn_vol_status = 0;
  }

  if(btn_sen_read==HIGH && btn_sen_status==0){
    btn_sen_status = 1;
    sensitivity_change();
    Serial.println("Sensitivity changed");
  } else if(btn_sen_read==LOW){
    btn_sen_status = 0;
  }

  if(btn_mode_read==HIGH && btn_mode_status==0){
    btn_mode_status = 1;
    mode_change();
    Serial.println("Mode changed");
  } else if(btn_mode_read==LOW){
    btn_mode_status = 0;
  }
}

void note_maj(){
  midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value+4, velocity_value);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value+7, velocity_value);   // Channel 0, middle C, normal velocity
  channel_value_old = channel_value;
  pitch_value_old = pitch_value;
  velocity_value_old = velocity_value;
  systime = millis();
  Serial.println("Note major");
}

void note_min(){
  midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value+3, velocity_value);   // Channel 0, middle C, normal velocity
  midiNoteOn(channel_value, pitch_value+7, velocity_value);   // Channel 0, middle C, normal velocity
  channel_value_old = channel_value;
  pitch_value_old = pitch_value;
  velocity_value_old = velocity_value;
  systime = millis();
  Serial.println("Note minor");
}

void flex_check(){
  int flex_rep_read = analogRead(FLEX_REP_PIN) - base_flex_rep;
  int flex_maj_read = analogRead(FLEX_MAJ_PIN) - base_flex_maj;
  int flex_min_read = analogRead(FLEX_MIN_PIN) - base_flex_min;
  int flex_sus_read = analogRead(FLEX_SUS_PIN) - base_flex_sus;
  Serial.println(flex_rep_read);
  Serial.println(flex_maj_read);
  Serial.println(flex_min_read);
  Serial.println(flex_sus_read);

  if(flex_maj_read>flex_threshold && flex_maj_status==0 || flex_maj_read<flex_threshold && flex_maj_status==1){
    int flex_maj_status_new = flex_maj_read>flex_threshold;
    if(flex_maj_status_new = 1 && flex_maj_status ==0){
      Serial.println("Major");
      note_maj();
    }
    flex_maj_status = flex_maj_status_new;
  }

  if(flex_min_read>flex_threshold && flex_min_status==0 || flex_min_read<flex_threshold && flex_min_status==1){
    int flex_min_status_new = flex_min_read>flex_threshold;
    if(flex_min_status_new = 1 && flex_min_status ==0){
      Serial.println("Minor");
      note_min();
    }
    flex_min_status = flex_min_status_new;
  }

  if(flex_rep_read>flex_threshold && flex_rep_status==0 || flex_rep_read<flex_threshold && flex_rep_status==1){
    int flex_rep_status_new = flex_rep_read>flex_threshold;
    if(flex_rep_status_new = 1 && flex_rep_status ==0){
      Serial.println("Repeat instrument");
      refresh_note();
    }
    flex_rep_status = flex_rep_status_new;
  }

  if((flex_min_read + flex_maj_read+ flex_rep_read+flex_sus_read) > (flex_threshold*6)){
    midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);   // Channel 0, middle C, normal velocity
  }

}

void flex_setup(void){
  int flex_maj_read = analogRead(FLEX_MAJ_PIN);
  int flex_min_read = analogRead(FLEX_MIN_PIN);
  int flex_rep_read = analogRead(FLEX_REP_PIN);
  int flex_sus_read = analogRead(FLEX_SUS_PIN);

  delay(5);

  for (int i = 1; i< 50;i++){
    flex_maj_read += analogRead(FLEX_MAJ_PIN);
    flex_min_read += analogRead(FLEX_MIN_PIN);
    flex_rep_read += analogRead(FLEX_REP_PIN);
    flex_sus_read += analogRead(FLEX_SUS_PIN);
    delay(5);
  }

  base_flex_maj = flex_maj_read/50;
  base_flex_min = flex_min_read/50;
  base_flex_rep = flex_rep_read/50;
  base_flex_sus = flex_sus_read/50;

  Serial.println(base_flex_maj);
  Serial.println(base_flex_min);
  Serial.println(base_flex_rep);
  Serial.println(base_flex_sus);
}

void setup(void){

  SPI.begin();
  delay(3000);

  Serial.begin(115200);
  Serial.println(volumes[volumes_index]);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  Serial.println("VS1053 MIDI test");
  delay(1000);  

  vs1053Reset();
  delay(1000);
  vs1053Begin();
  delay(5);
  vs1053SetVolume(5,5);
  
  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'

  pinMode(BUTTON_INSTRUMENT, INPUT);
  pinMode(BUTTON_SENSITIVITY, INPUT);
  pinMode(BUTTON_VOLUME, INPUT);
  pinMode(BUTTON_MODE, INPUT);

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

  flex_setup();
}

void loop(void){
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);               

  flex_check();
  button_check();

  /* The processing sketch expects data as roll, pitch, heading */
  float roll_angle = (float)event.orientation.x;
  float pitch_angle = (float)event.orientation.y;
  float heading_angle = (float)event.orientation.z;
  tot_angle = roll_angle + pitch_angle + heading_angle;
  pitch_value = (uint8_t) ((tot_angle / sensitivity) + 50);

  if(mode_sus==0){
    if(((millis()-systime)>500)) {
      midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity
    }
    if(abs(tot_angle - tot_angle_old)>sensitivity/2) {
      midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity
      midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
      tot_angle_old = tot_angle;
      channel_value_old = channel_value;
      pitch_value_old = pitch_value;
      velocity_value_old = velocity_value;
      systime = millis();
    }
  }else{
    if(abs(tot_angle - tot_angle_old)>sensitivity/2) {
      midiNoteOff(channel_value_old, pitch_value_old, velocity_value_old);  // Channel 0, middle C, normal velocity
      tot_angle_old = tot_angle;
      channel_value_old = channel_value;
      pitch_value_old = pitch_value;
      velocity_value_old = velocity_value;
      midiNoteOn(channel_value, pitch_value, velocity_value);   // Channel 0, middle C, normal velocity
    }
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
