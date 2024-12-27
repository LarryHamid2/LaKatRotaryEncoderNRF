
#ifndef LAKAT_ROTARY_ENCODER_NRF_H
#define LAKAT_ROTARY_ENCODER_NRF_H

//
// Class for operating a rotary encoder with an integrated switch.
// Specifically these products by Bourns:
// PEC11 Series - 12 mm Incremental Encoder
//
// Adafruit sells them as PRODUCT ID: 377
//
// Note that this implementation also assumes an nRF52840 (Adafruit ItsyBitsy nRF)
// for the pin assignments.
//

#include <Arduino.h>
#include <RotaryEncoder.h>

// Wiring (Arduino) PIN assignments
#define ROTARY_ENCODER_PIN_A    A0
#define ROTARY_ENCODER_PIN_B    2
#define ROTARY_ENCODER_PIN_SW   A1   // This is the built in push switch

#define ENCODER_DEBOUNCE_DELAY		5		// 5 milliseconds is supposed to be all we need according to Bourns
#define ENCODER_SWITCH_DEBOUNCE_DELAY   15	// 5 milliseconds is supposed to be all we need according to Bourns
#define N_TURN_SAMPLES              5       // # of samples in our FIFO to calculate turn rate.

class LaKatRotaryEncoderNRF /* : public LaKatButtonCallbacks */
{
private:
  	int                	m_lastPosition;
  	int                	m_encoderBounces;       // Keeps track of number of bounces we think we had
    int                 m_PIN_A;
    int                 m_PIN_B;
    int                 m_PIN_SW;
    int                 m_didPress;
    int                 m_currentButtonState;
    int                 m_lastButtonState;;
    int                 m_turnFIFO[ N_TURN_SAMPLES ];
    int                 m_FIFO_position;

    HwRotaryEncoder     m_encoder;              // Using Arduino Library

    unsigned long       m_lastSWInterruptTime;
    unsigned long       m_lastDebounceTime;;

public:
  	LaKatRotaryEncoderNRF( void );
    int begin( void (*callback_switch)() ); // Callers must pass our public isrs
  	void rotary_encoder_switch_isr();
  	int didTurn( void );
    int didPress( void );
  	int getTurnRate( void );
  	int getPosition( void );
  	int getButtonPin( void );

protected:
};

#endif
