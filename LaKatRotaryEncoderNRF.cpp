
/*
 * This sketch demonstrate how to pass ISR_DEFFERED as additional parameter
 * to defer callback from ISR context with attachInterrupt
 */
#include "LaKatRotaryEncoderNRF.h"
#include <nrf_rtc.h>

static uint32_t lakat_millis( void )
{
    return nrf_rtc_counter_get( NRF_RTC1 );
}

LaKatRotaryEncoderNRF::LaKatRotaryEncoderNRF( void )
{
    m_PIN_A             = ROTARY_ENCODER_PIN_A;
    m_PIN_B             = ROTARY_ENCODER_PIN_B;
    m_PIN_SW            = ROTARY_ENCODER_PIN_SW;

    m_lastSWInterruptTime = 0;
    m_lastPosition      = 0;
    m_encoderBounces    = 0;
    m_didPress          = 0;

    m_encoder           = HwRotaryEncoder();

    m_lastDebounceTime      = 0;
    m_lastButtonState       = -1;
    m_currentButtonState    = -1;

    memset( m_turnFIFO, 0, N_TURN_SAMPLES * sizeof( int ) );
    m_FIFO_position = 0;
}

int LaKatRotaryEncoderNRF::begin( void (*callback_switch)() )
{
    m_encoder.begin( ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B );
    pinMode( m_PIN_SW, INPUT_PULLUP );

    // ISR_DEFERRED flag cause the callback to be deferred from ISR context
    // and invoked within a callback thread.
    // It is required to use ISR_DEFERRED if callback function take long time 
    // to run e.g Serial.print() or using any of Bluefruit API() which will
    // potentially call rtos API

    m_lastButtonState     = HIGH; // We're using pullup
    m_currentButtonState  = HIGH; // Used when polling
    m_lastDebounceTime    = millis();
    m_lastSWInterruptTime = millis();
    // PIN A will be used as the interrupt trigger for the encoder.
    //int pin = digitalPinToInterrupt( m_PIN_A );
    //attachInterrupt( pin, callback_rotary, CHANGE );

#ifdef USE_INTERRUPTS
#error "I thought we weren't using intterrupts"
    int pin = digitalPinToInterrupt( m_PIN_SW );
    attachInterrupt( pin, callback_switch, CHANGE );
#endif

    // TODO: Get the return status of attachInterrupt and use it.
    m_encoder.start();
    m_lastPosition = m_encoder.readAbs()/2;

    return 0;
}

int LaKatRotaryEncoderNRF::getButtonPin( void )
{
    return m_PIN_SW;
}

int LaKatRotaryEncoderNRF::getTurnRate( void )
{
    int total = 0;
    for ( int i = 0; i < N_TURN_SAMPLES; i++ ) {
        total += m_turnFIFO[ i ];
    }
    return abs(total);
}

//
// Returns 0 if no change occurred.
// Returns 1 if movement clockwise.
// Returns -1 if movement counterClockwise
int LaKatRotaryEncoderNRF::didTurn( void )
{
    int32_t position = m_encoder.readAbs()/2;
    int32_t delta = position - m_lastPosition;
    // We want to keep track of the turn rate...
    m_turnFIFO[ m_FIFO_position++ ] = (int)delta;
    if ( N_TURN_SAMPLES == m_FIFO_position ) m_FIFO_position = 0;
    m_lastPosition = position;
    return delta;
}

#ifndef USE_INTERRUPTS
// Interrupts are hard to get right.  Trying polling.
// Only caveat is that the application needs to call this frequently enough
int LaKatRotaryEncoderNRF::didPress( void )
{
    int reading = digitalRead( m_PIN_SW );
    if ( reading != m_lastButtonState ) {
        // There's been a change of state.  Get the time of this.
        m_lastDebounceTime = millis();
    }
    if ( millis() - m_lastDebounceTime > ENCODER_SWITCH_DEBOUNCE_DELAY ) {
        // We're past the debounce period.
        if ( reading != m_currentButtonState ) {
            // We have a debounced change of state.
            //Serial.print( "Debounced state: " );
            //Serial.println( reading );
            m_currentButtonState = reading;
            if ( LOW == m_currentButtonState ) {
                // We have a debounced event
                return 1;
            }
        }
    }
    m_lastButtonState = reading;
    return 0;
}

#else
#error "I though interrupts were disabled for switch"
// Interrupt driven.
// Here we just pass back the state of any detected event.
int LaKatRotaryEncoderNRF::didPress( void )
{
    int press = m_didPress;
    m_didPress = 0;  // Clear it
    return press;
}

#endif

int LaKatRotaryEncoderNRF::getPosition( void )
{
    return m_lastPosition;
}

void LaKatRotaryEncoderNRF::rotary_encoder_switch_isr()
{
    // We are falling, 
    unsigned long interruptTime = millis();
    unsigned long deltaT = interruptTime - m_lastSWInterruptTime;

    int a = digitalRead( m_PIN_SW );

    if ( LOW == a ) {
        //Serial.print( "L dT:" );
        //Serial.println( deltaT );
        if ( deltaT > ENCODER_SWITCH_DEBOUNCE_DELAY ) {
            m_lastSWInterruptTime = interruptTime;  // Reset interrupt time (assume it is a fresh press)
        }
    }
    if ( HIGH == a ) {
        //Serial.print( "H dT:" );
        //Serial.println( deltaT );
        // We assume a maximum bounce time of 5 milliseconds
        if ( deltaT > ENCODER_SWITCH_DEBOUNCE_DELAY ) {
            //Serial.print( "Press dT: " );
            //Serial.println( deltaT );
            m_didPress = 1;
        }
    }
}

