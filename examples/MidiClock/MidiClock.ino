/*
   Midi clock generator based on UNO and uClock with Sparkfun midi shield latest version (softserail AND serial at the same time
*/

#include "Arduino.h"
#include <uClock.h>
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
#include <SoftwareSerial.h>

//// MIDI clock, start and stop byte definitions - based on MIDI 1.0 Standards.
//#define MIDI_CLOCK 0xF8
//#define MIDI_START 0xFA
//#define MIDI_STOP  0xFC

// midi soft serial for DIN5 sockets
SoftwareSerial mSerial (8, 9);
MIDI_CREATE_INSTANCE(SoftwareSerial, mSerial, MIDI);

// sparkfun midi shield UI :
#define PIN_LED_PLAYING 6 // D6
#define PIN_LED_TEMPO 7 // D7
#define PIN_PLAY_INPUT 2 // D2
#define PIN_CONTINUE_INPUT 3 // D3
#define PIN_TEMPO_POT 1 // A1

static const uint16_t DEBOUNCE_COUNT = 50;


void check_buttons()
{
  uint8_t val;
  static uint16_t play_debounce = 0;
  static uint16_t cont_debounce = 0;

  // First the PLAY/STOP button
  val = digitalRead(PIN_PLAY_INPUT);

  if (val == LOW)
  {
    play_debounce++;
    if (play_debounce == DEBOUNCE_COUNT)
    {
      play_button_event();
    }
  }
  else
  {
    play_debounce = 0;
  }
  // Then the continue button
  val = digitalRead(PIN_CONTINUE_INPUT);
  if (val == LOW)
  {
    cont_debounce++;
    if (cont_debounce == DEBOUNCE_COUNT)
    {
      cont_button_event();
    }
  }
  else
  {
    cont_debounce = 0;
  }
}

void play_button_event()
{
  // toggle running state, 
  // send corresponding responses
  running = !running;
    
    if(running)
    {
      send_start = true;
      digitalWrite(PIN_LED_PLAYING, LOW);
    }
    else
    {
      send_stop = true;
      digitalWrite(PIN_LED_PLAYING, HIGH);
    }
}

void cont_button_event()
{
  // ignore continue if running
  if(!running)
  {
    send_continue = true;
    running = true;
    digitalWrite(PIN_LED_PLAYING, LOW);
  }
}

void timer_callback()
{
  send_tick = true;
}

void check_pots()
{
  uint32_t pot_val;
  uint32_t calc;
  
  pot_val = 1023 - analogRead(PIN_TEMPO_POT);
  
  // Result is 10 bits
  calc = (((0x3ff - pot_val) * 75)/1023) + 8;
  
  tempo_delay = calc  ;//* 5;
}
// The callback function wich will be called by Clock each Pulse of 96PPQN clock resolution.
void ClockOut96PPQN(uint32_t * tick)
{
  // Send MIDI_CLOCK to external gears
  MIDI.sendRealTime(MIDI_NAMESPACE::Clock);
}

// The callback function wich will be called when clock starts by using Clock.start() method.
void onClockStart()
{
  //Serial.write(MIDI_START);
  MIDI.sendRealTime(MIDI_NAMESPACE::Start);

}

// The callback function wich will be called when clock stops by using Clock.stop() method.
void onClockStop()
{
  //Serial.write(MIDI_STOP);
  MIDI.sendRealTime(MIDI_NAMESPACE::Stop);
  // MIDI.sendNoteOn(42, 127, 1); //

}

/*
  void enableMidiCallbacks(void)
  {
  MIDI.setHandleNoteOff(ZoneNoteOff);
  MIDI.setHandleNoteOn(ZoneNoteOn);
  MIDI.setHandleControlChange(HandleControlChange);
  MIDI.setHandleProgramChange(HandleProgramChange);
  MIDI.setHandleAfterTouchChannel(HandleAfterTouchChannel);
  MIDI.setHandlePitchBend(HandlePitchBend);
  MIDI.setHandleSystemExclusive(HandleSystemExclusive);
  MIDI.setHandleClock(HandleClock);
  MIDI.setHandleStart(HandleStart);
  MIDI.setHandleContinue(HandleContinue);
  MIDI.setHandleStop(HandleStop);

  }
*/

void setup()
{

  // Initialize serial communication at 31250 bits per second, the default MIDI serial speed communication:
  Serial.begin(115200);
  
  // LED outputs
  pinMode(PIN_LED_PLAYING, OUTPUT);
  pinMode(PIN_LED_TEMPO, OUTPUT);
  digitalWrite(PIN_LED_PLAYING, HIGH);
  digitalWrite(PIN_LED_TEMPO, HIGH);
  
  // button inputs
  pinMode(PIN_PLAY_INPUT, INPUT_PULLUP);
  pinMode(PIN_CONTINUE_INPUT, INPUT_PULLUP);
  
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();

  // Connect the midi callbacks to the library,
  //enableMidiCallbacks();

  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setClock96PPQNOutput(ClockOut96PPQN);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);
  uClock.setOnClockStopOutput(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(126);

  // Starts the clock, tick-tac-tick-tac...
  uClock.start();

}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop()
{
  MIDI.read();
    // Check buttons
  check_buttons();
      check_pots();
}
