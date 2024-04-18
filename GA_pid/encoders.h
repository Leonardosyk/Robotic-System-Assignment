#ifndef _ENCODERS_H
#define _ENCODERS_H

#define ENCODER_0_A_PIN  7
#define ENCODER_0_B_PIN  23
#define ENCODER_1_A_PIN  26
//#define ENCODER_1_B_PIN Non-standard pin!

// Volatile Global variables used by Encoder ISR.
volatile long count_leftEncoder; // used by encoder to count the rotation
volatile byte state_leftEncoder;
volatile long count_rightEncoder;
volatile byte state_rightEncoder;

// This ISR now handles Encoder 1 (previously handled just Encoder 0)
ISR( INT6_vect ) {
    // Read in the new state of the encoder pins.
    boolean rightEncoder_B = digitalRead( ENCODER_0_B_PIN );
    boolean rightEncoder_A = digitalRead( ENCODER_0_A_PIN ); // XORed value

    rightEncoder_A = rightEncoder_A ^ rightEncoder_B;

    state_rightEncoder = state_rightEncoder | ( rightEncoder_B  << 3 );
    state_rightEncoder = state_rightEncoder | ( rightEncoder_A  << 2 );

    // The logic within the ISR remains the same, we are just changing which
    // encoder the ISR is associated with
    if (state_rightEncoder == 0 || state_rightEncoder == 5 || state_rightEncoder == 10 || state_rightEncoder == 15) {
        // No change
    } else if (state_rightEncoder == 1 || state_rightEncoder == 7 || state_rightEncoder == 8 || state_rightEncoder == 14) {
        count_rightEncoder -= 1;  // Clockwise
    } else if (state_rightEncoder == 2 || state_rightEncoder == 4 || state_rightEncoder == 11 || state_rightEncoder == 13) {
        count_rightEncoder += 1;  // Counterclockwise
    } else {
        // Invalid Move - may want to add some error handling or ignore the
    }
    state_rightEncoder = state_rightEncoder >> 2;
}

// This ISR now handles Encoder 0 (previously handled just Encoder 1)
ISR( PCINT0_vect ) {
    boolean leftEncoder_B = PINE & (1<<PINE2);
    boolean leftEncoder_A = digitalRead( ENCODER_1_A_PIN );

    leftEncoder_A = leftEncoder_A ^ leftEncoder_B;

    state_leftEncoder = state_leftEncoder | ( leftEncoder_B  << 3 );
    state_leftEncoder = state_leftEncoder | ( leftEncoder_A  << 2 );

    // The logic within the ISR remains the same, we are just changing which
    // encoder the ISR is associated with
    if (state_leftEncoder == 0 || state_leftEncoder == 5 || state_leftEncoder == 10 || state_leftEncoder == 15) {
        // No change
    } else if (state_leftEncoder == 1 || state_leftEncoder == 7 || state_leftEncoder == 8 || state_leftEncoder == 14) {
        count_leftEncoder -= 1;   // Clockwise
    } else if (state_leftEncoder == 2 || state_leftEncoder == 4 || state_leftEncoder == 11 || state_leftEncoder == 13) {
        count_leftEncoder += 1;  // Counterclockwise
    } else {
        // Invalid Move - may want to add some error handling or ignore the
    }
    state_leftEncoder = state_leftEncoder >> 2;
}



void setupEncoder1() 
{
    count_leftEncoder = 0;

    // Setup pins for right encoder 
    pinMode( ENCODER_0_A_PIN, INPUT );
    pinMode( ENCODER_0_B_PIN, INPUT );

    // initialise the recorded state of e0 encoder.
    state_leftEncoder = 0;

    // Get initial state of encoder pins A + B
    boolean e0_A = digitalRead( ENCODER_0_A_PIN );
    boolean e0_B = digitalRead( ENCODER_0_B_PIN );
    e0_A = e0_A ^ e0_B;

    // Shift values into correct place in state.
    // Bits 1 and 0  are prior states.
    state_leftEncoder = state_leftEncoder | ( e0_B << 1 );
    state_leftEncoder = state_leftEncoder | ( e0_A << 0 );


    // Now to set up PE6 as an external interupt (INT6), which means it can
    // have its own dedicated ISR vector INT6_vector

    // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
    // Disable external interrupts for INT6 first
    // Set INT6 bit low, preserve other bits
    EIMSK = EIMSK & ~(1<<INT6);
    //EIMSK = EIMSK & B1011111; // Same as above.
  
    // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
    // Used to set up INT6 interrupt
    EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
    //EICRB |= B00010000; // does same as above

    // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
    // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
    EIFR |= ( 1 << INTF6 );
    //EIFR |= B01000000;  // same as above

    // Now that we have set INT6 interrupt up, we can enable
    // the interrupt to happen
    // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
    // Disable external interrupts for INT6 first
    // Set INT6 bit high, preserve other bits
    EIMSK |= ( 1 << INT6 );
    //EIMSK |= B01000000; // Same as above

}

void setupEncoder0() 
{

    count_rightEncoder = 0;

    // Setting up left encoder:
    // The Romi board uses the pin PE2 (port E, pin 2) which is
    // very unconventional.  It doesn't have a standard
    // arduino alias (like d6, or a5, for example).
    // We set it up here with direct register access
    // Writing a 0 to a DDR sets as input
    // DDRE = Data Direction Register (Port)E
    // We want pin PE2, which means bit 2 (counting from 0)
    // PE Register bits [ 7  6  5  4  3  2  1  0 ]
    // Binary mask      [ 1  1  1  1  1  0  1  1 ]
    //    
    // By performing an & here, the 0 sets low, all 1's preserve
    // any previous state.
    DDRE = DDRE & ~(1<<DDE6);
    //DDRE = DDRE & B11111011; // Same as above. 

    // We need to enable the pull up resistor for the pin
    // To do this, once a pin is set to input (as above)
    // You write a 1 to the bit in the output register
    PORTE = PORTE | (1 << PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Encoder0 uses conventional pin 26
    pinMode( ENCODER_1_A_PIN, INPUT );
    digitalWrite( ENCODER_1_A_PIN, HIGH ); // Encoder 1 xor

    // initialise the recorded state of e1 encoder.
    state_rightEncoder = 0;
    
    // Get initial state of encoder.
    boolean e1_B = PINE & (1<<PINE2);
    //boolean e1_B = PINE & B00000100;  // Does same as above.

    // Standard read from the other pin.
    boolean e1_A = digitalRead( ENCODER_1_A_PIN ); // 26 the same as A8

    // Some clever electronics combines the
    // signals and this XOR restores the 
    // true value.
    e1_A = e1_A ^ e1_B;

    // Shift values into correct place in state.
    // Bits 1 and 0  are prior states.
    state_rightEncoder = state_rightEncoder | ( e1_B << 1 );
    state_rightEncoder = state_rightEncoder | ( e1_A << 0 );

    // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
    // pin-change interrupts.
    // Note, this register will normally create an interrupt a change to any pins
    // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
    // When we set these registers, the compiler will now look for a routine called
    // ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
    // mismatch to PCINT4, however there is only the one vector servicing a change
    // to all PCINT0->7 pins.
    // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR
    
    // Page 91, 11.1.5, Pin Change Interrupt Control Register 
    // Disable interrupt first
    PCICR = PCICR & ~( 1 << PCIE0 );
    // PCICR &= B11111110;  // Same as above
    
    // 11.1.7 Pin Change Mask Register 0 – PCMSK0
    PCMSK0 |= (1 << PCINT4);
    
    // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

    // Enable
    PCICR |= (1 << PCIE0);
}

#endif
