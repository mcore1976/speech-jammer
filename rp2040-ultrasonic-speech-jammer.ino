// DIRECTIONAL ULTRASONIC SPEECH JAMMER
// based on Raspberry Pi Pico - RP2040-zero board
// MAX4466 microphone module
// and TPA2116D2 module XH-M542
// plus ultrasonic transducers - at least 10
// please install PWM 2040 library https://github.com/khoih-prog/RP2040_PWM
// (C) Adam Loboda 2025

#include "RP2040_PWM.h"

// put desired ADCxx RP2040 pico pin here - I am using ADC1 which is GP27. 
// pin A1 = microphone input MAX 4466
const int analogPin = A1;

// global variable for reading ADC 
int audioreading = 0;
int pwmread = 0;
int adcresult = 0;
int adcoffset = 0;

// Array for audio readings - will be used as delay for the sound samples
#define AUDIOARRAYSIZE 75000
uint16_t audioarray[AUDIOARRAYSIZE];
// pointers to the array for ADC filling and PWM readings
uint16_t adcpointer = 0;
uint16_t pwmpointer = 0;

// How much to delay audio for speech jamming in number of ADC samples
#define AUDIODELAY  20000

// semaphore for putting data from first core into memory and reading by second core
bool semaphore = false;

// creates pwm instance
RP2040_PWM* PWM_Instance;
// setup the frequency of your ultrasonic transducers here
// it has to be resonance frequency of your transducers model f.ex. 40000 or 25000
float frequency = 25000;
float dutyCycle;
#define PWMpinToUse 1    // GP1 as PWM output in my case 

// FIRST RASPBERRY PI PICO CORE PROCEDURES

// First RP2040 core will only take care of PWM
void setup() {
  // initialize PWM
  PWM_Instance = new RP2040_PWM(PWMpinToUse, frequency, 0);

  // synchronize the second Core - ADC operation via semaphore
  semaphore = true;
  
}

// loop for the First core and PWM steering
void loop() {

  // set PWM dutyCycle according to microphone readings
  // binary shift left 7 bits aka multiply by 128, 
  // you may want to decrease it to 7 if the audio signal is high or increase it of signal is low
  // play with this value to achieve best volume and less distortion

  // calculating which audio frame to play
  pwmpointer = adcpointer - AUDIODELAY ;
  if (pwmpointer < 0)   pwmpointer = AUDIOARRAYSIZE - adcpointer ; 
  // reading the ADC sample
  pwmread = audioarray[pwmpointer];
  
  // PWM_Instance->setPWM_Int(PWMpinToUse, frequency, audioreading << 8);  
  // if (semaphore == 0 ) PWM_Instance->setPWM_Int(PWMpinToUse, frequency, audioreading << 6);
  PWM_Instance->setPWM_Int(PWMpinToUse, frequency, pwmread << 6);

  // increase pointer to audiodata and rewind to the beginning if needed
  pwmpointer++;
  if (pwmpointer == AUDIOARRAYSIZE)   pwmpointer = 0;

  // delay microseconds for the next sample 
  delayMicroseconds(2); // Some delay may be needed

}


// SECOND RASPBERRY PI PICO CORE PROCEDURES

// Second RP2040 core will only take care of sampling audio by ADC
void setup1() {
  
  // initialize serial port for debugging - enable ony if needed
  // Serial.begin(115200);
  
  // initialize ADC to sample at 10 for faster reading - max signal is 1024 reading value
  // or initialize ADC for 12 bits to get more precise and better sound - max is 4096 reading value
  // analogReadResolution(10); 
  analogReadResolution(12);
  // synchronize second CPU core for PWM readings 
  semaphore = false;
}

// loop for the Second core and ADC readings
void loop1() {

  // wait for the semaphore of PWM initialization to start sampling
  while (!semaphore); 
  
  // read signal value from microphone ADC input
  adcresult = analogRead(analogPin);
  
  // You may want to use dome offset to the signal depending how accuate is voltage divider
  adcresult = adcresult - adcoffset ;   
  
  // then compute middle value of this sinusoidal signal 
  // because it is shifted by the resistor voltage divider at the input of ADC
  // 512 is for 10bit ADC, 2048 is for 12 bit ADC
  // adcresult = adcresult - 512; 
     adcresult = adcresult - 2048; 

  // calculate ABSolute value of the signal amplitude now
  if (adcresult<0) audioreading = 0 - adcresult ;
  else  audioreading = adcresult;

  // put the final value into the audio array for further reading
  audioarray[adcpointer] = audioreading;
  // increase pointer to ADC audiodata and rewind to the beginning if needed
  adcpointer++;
  if (adcpointer == AUDIOARRAYSIZE)   adcpointer = 0;
  // delay microseconds for the next sample 
  delayMicroseconds(5);
  
}
