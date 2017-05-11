
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Bounce.h>
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

#define LOG_INTERVAL  1000 // mills between entries
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//input = microphone
const int myInput = AUDIO_INPUT_MIC;
const int chipSelect = BUILTIN_SDCARD;
int j = 0;
int k = 0;
int pot = 0;
int average = 0;
int averagecounter = 0;
int line = 0;
int counter = 0;
int looptime = 3000;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
int t0 = 0;
int t1 = 0;
int t2 = 0;
int data[16];
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

//button variables
const int redbuttonPin = 6;    // the number of the pushbutton pin
int redbuttonState;             // the current reading from the input pin
int lastredButtonState = LOW;   // the previous reading from the input pin
unsigned long lastredDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
//
AudioInputI2S          i2s2;         // audio shield: mic or line-in
AudioSynthWaveformSine sinewave;
AudioAnalyzeFFT1024    myFFT;
AudioOutputI2S         i2s1;        // audio shield: headphones & line-out
AudioAnalyzePeak         peak1;          //xy=278,108
AudioRecordQueue         queue1;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioConnection          patchCord2(i2s2, 0, peak1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// set mic as input
AudioConnection patchCord3(i2s2, 0, myFFT, 0);
AudioControlSGTL5000 audioShield;

// Bounce objects to easily and reliably read the buttons
Bounce buttonRecord = Bounce(0, 8);
Bounce buttonStop =   Bounce(1, 8);  // 8 = 8 ms debounce time
Bounce buttonPlay =   Bounce(2, 8);

int saveRate = 10000;
int readRate = 1000;
int count = 0;

//time variables
int secs = 0;
int mins = 0;
int hours = 0;

//decibel variable
float fake_db = 0;

//file names of FFT and db files
File myFile_FFT;
File myFile_DB;
File frec;
String FFT;
String DB;

elapsedMillis msecs;

void setup() {
  Serial.begin(9600);

  //set red button
  pinMode(redbuttonPin, INPUT_PULLUP);

  //initialize matrix array
  matrix.begin(0x70);  // pass in the address

  // Initialize the SD card
  Serial.print("Initializing SD card...");
  pinMode(BUILTIN_SDCARD, OUTPUT);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  }

  Serial.println("initialization done.");

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(60);

  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.volume(0.5);

  // Configure the window algorithm to use
  myFFT.windowFunction(AudioWindowHanning1024);

  // create a new file
  for (int i = 0; i < 10000; i++) {

    String stringFFT = "FFT";
    stringFFT.concat(i);
    stringFFT.concat(".CSV");
    char filename[stringFFT.length() + 1];
    FFT = stringFFT;
    stringFFT.toCharArray(filename, sizeof(filename));
    Serial.print("FFT print statement");
    Serial.println(filename);
    String FFT(filename);

    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      myFile_FFT = SD.open(filename, FILE_WRITE);
      myFile_FFT.close();
      break;  // leave the loop!
    }
  }

  if (! myFile_FFT) {
    Serial.println("couldnt create file_FFT");
  }

  for (int i = 0; i < 10000; i++) {
    String stringDB = "DB";
    stringDB.concat(i);
    stringDB.concat(".CSV");
    char filename1[stringDB.length() + 1];
    DB = stringDB;
    stringDB.toCharArray(filename1, sizeof(filename1));
    Serial.print("db print statement");
    Serial.println(filename1);
    if (! SD.exists(filename1)) {
      // only open a new file if it doesn't exist
      myFile_DB = SD.open(filename1, FILE_WRITE);
      myFile_DB.close();

      break;  // leave the loop!
    }
  }

  if (! myFile_DB) {
    Serial.println("couldnt create file_DB");
  }

  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  Teensy3Clock.set(now());
  //
  Serial.begin(115200);
  //while (!Serial);  // Wait for Arduino Serial Monitor to open
  //delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
}

/**************************************************************/
/**************************************************************/

void loop() {
  float n;
  int i;

  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  uint32_t m = millis();

  // delay for the amount of time we want between readings
  //delay((1000);

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(0);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax) {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin) {
        signalMin = sample;  // save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 5.0) ;  // convert to volts

  //map volts to db values
  fake_db = map(volts, 500, 4000, 32, 113);

  //matrix code
  for (int i = 0; i < k ; i++)
  {
    matrix.drawLine(0, i, data[i], i, LED_ON);
  }

  //add volts calculation here


  line = map(volts, 0, 2600, 0, 8);
  average = average + line;
  matrix.drawLine(0, k, line, k, LED_ON);
  matrix.writeDisplay();
  matrix.clear();
  counter++;
  //Serial.println(counter);

  if (counter == looptime)
  {
    average = average / looptime;
    data[k] = average;
    k++;
    if (average > 6)
    {
      matrix.setTextSize(1);
      matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
      matrix.setTextColor(LED_ON);
      matrix.setRotation(3);
      for (int8_t x = 7; x >= -36; x--) {
        matrix.clear();
        matrix.setCursor(x, 0);
        matrix.print("Too Loud");
        matrix.writeDisplay();
        delay(100);
      }
      matrix.setRotation(0);
    }


    if (k == 8)
    {
      k = 0;
      for (int i = 0 ; i < 9 ; i++)
      {
        data[i] = 0;
      }
      matrix.clear();
    }
    counter = 0;
    average = 0;
  }


  int readingred = digitalRead(redbuttonPin);
  if (readingred != lastredButtonState) {
    lastredDebounceTime = millis();
  }

  if ((millis() - lastredDebounceTime) > debounceDelay) {
    if (readingred != redbuttonState) {
      redbuttonState = readingred;
      if (redbuttonState == HIGH) {
        Serial.println("red button press");
        myFile_DB.println("red button press");
        myFile_FFT.println("red button press");
      }
    }
  }

  lastredButtonState = readingred;

  if ((millis() - t0) > 1000)
  {

    if (Serial.available()) {
      time_t t = processSyncMessage();
      if (t != 0) {
        Teensy3Clock.set(t); // set the RTC
        setTime(Teensy3Clock.get());
      }
    }
    digitalClockDisplay();
    Serial.print(fake_db);
    Serial.print(" ");
    myFile_DB.print(fake_db);
    myFile_DB.println(" ");
    t0 = millis();


    if (myFFT.available()) {
      Serial.print("FFT: ");
      myFile_FFT.print("FFT: ");

      for (i = 0; i < 40; i++) {
        n = myFFT.read(i);

        if (n >= 0.024) {
          Serial.print(n);
          Serial.print(" ");
          myFile_FFT.print(n);
          myFile_FFT.print(" ");
        }
        else {
          Serial.print("  -  "); // don't print "0.00"
          myFile_FFT.print("  -  ");
        }
      }

      Serial.println();
      myFile_FFT.println(" ");
      myFile_DB.println(" ");

      //    if ((millis() - t1) > 1000)
      //    {
      //      Serial.println();
      //      myFile_FFT.println();
      //      t1 = millis();
      //    }
    }
    secs = millis() / 1000;
    mins = secs / 60;
    hours = mins / 60;

    if ((millis() - t2) > 10000)
    {
      t2 = millis();
      //Serial.println("file closed fft");
      myFile_DB.close();
      myFile_FFT.close();

      int str_len_FFT = FFT.length() + 1;
      char char_array_fft[str_len_FFT];
      FFT.toCharArray(char_array_fft, str_len_FFT);

      int str_len_db = DB.length() + 1;
      char char_array_db[str_len_db];
      DB.toCharArray(char_array_db, str_len_db);

      myFile_DB = SD.open(char_array_db, FILE_WRITE);
      myFile_FFT = SD.open(char_array_fft, FILE_WRITE);
    }





  }
}

void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.print(" ");

  myFile_FFT.print(hour());
  myFile_FFT.print(":");
  myFile_FFT.print(minute());
  myFile_FFT.print(":");
  myFile_FFT.print(second());
  myFile_FFT.print(" ");
  myFile_FFT.print(day());
  myFile_FFT.print(" ");
  myFile_FFT.print(month());
  myFile_FFT.print(" ");
  myFile_FFT.print(year());
  myFile_FFT.print(" ");

  myFile_DB.print(hour());
  myFile_DB.print(":");
  myFile_DB.print(minute());
  myFile_DB.print(":");
  myFile_DB.print(second());
  myFile_DB.print(" ");
  myFile_DB.print(day());
  myFile_DB.print(" ");
  myFile_DB.print(month());
  myFile_DB.print(" ");
  myFile_DB.print(year());
  myFile_DB.print(" ");

}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
