// Arduino (Abra) Nano control of Si5351 clockgen chip on a MIKROE-4113 eval board
// Reads drone's X8R radio PWM output signal to control chopper state (steady off, chopping, steady on)
// Eval board uses 3.3 V power
// I2C SDA on pin A4
// I2C SCL on pin A5
// ATMega328p timer/counter 1 overflow output on pins D9 and D10
// GPIO output from chopper routine on pin D6 and built in LED
// X8R PWM input as a timer/counter 1 input capture interrupt on pin D8

#include "si5351.h"
#include "Wire.h"
#include <util/atomic.h>

Si5351 si5351;

const int test_lead = 6; // toggle GPIO according to the mode of the chopper, could be useful for a LED or something.
const uint16_t count_top = 12508; // counter timer overflow value, must be less than 32k for the input capture overflow method to work, 12508 is for 10 Hz chopping with a 1/64 prescaler (Arduino clock speed 16 MHz)

// variables and flag used in the timer/counter input capture interrupt ISR (PWM reading) (I know 'interrupt ISR' is a bit redundant but wanted to be clear):
uint8_t lowval; // used to read lowest 8 bits of input capture register
uint8_t highval; // used to read highest 8 bits of input capture register
uint16_t falling; // timer count at the falling edge of the PWM signal
uint16_t rising; // timer count at the rising edge of the PWM signal
volatile uint16_t interval; // number of counts between rise and fall of PWM signal ie the pulse width in timer counts
boolean dataflag; // flag that indicates a PWM pulse has been measured

// flags used in the timer/counter compare match interrupt ISR (10 Hz chopping):
boolean toggle0; // used in the chopping routine to toggle the output off and on
boolean chop_flag; // sets chopper state to chopping if true
boolean on_flag; // sets chopper state to steady on if true
boolean on_init; // flag so that the on state is only set when the Taranis switch is first flipped and not continuously reset at 20 Hz
boolean off_flag; // sets chopper state to steady off if true
boolean off_init; // flag so that the off state is only set when the Taranis switch is first flipped and not continuously reset at 20 Hz

// variables used in the main loop
const uint16_t up_pwm = 247; // timer/counter interval (PWM pulse width) of Taranis switch up position
const uint16_t middle_pwm = 375; // timer/counter interval (PWM pulse width) of Taranis switch middle position
const uint16_t down_pwm = 503; // timer/counter interval (PWM pusle width) of Taranis switch down position
const uint16_t pwm_range = 5; // range around detected PWM intervals to determine a match
uint16_t oldPWM; // used in main loop to detect a change in PWM pulse width ie mode of the chopper
uint16_t PWM; // same as above

void setup() {
  // configure GPIO
  pinMode(test_lead, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // configure Si5351 clockgen
  bool i2c_found;
  Serial.begin(9600);
    /* Initialise the sensor */
  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found)
  {
    Serial.println("Device not found on I2C bus!");
  }
  Serial.println("OK!");
  // Set CLK0 to output 50 MHz, all outputs disabled for now
  si5351.set_freq(5000000000ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);

  // configure timer/counter 1 and set interrupts
  TCCR1B = B00000000; // Stop Timer/Counter1 clock by setting the clock source to none.
  TCCR1A = B00000000; // Set Timer/Counter1 to normal mode.
  TCNT1  = 0; // Set Timer/Counter1 to 0
  OCR1A = count_top; // Set the Output Compare A for Timer/Counter1
  TCCR1A = B01010000; // Set Timer/Counter1 to CTC mode. Set OC1A and OC1B to toggle.
  TCCR1B = B00001011; // Start Timer/Counter1 clock by setting the source to CPU source. Set prescalar to 1/64.
  TIMSK1 = B00100010; // enable compare match interrupt and input capture interrupt
  DDRB = 6; // set data direction to output in case we want to get the raw clock output. Output on pins 9 and 10 (set in TCCR1A).
}

ISR(TIMER1_COMPA_vect) { // timer/counter 1 overflow interrupt, 10 Hz chopping freq or 20 Hz interrupt rate
  sei();
  if (chop_flag) { // chopping
    on_init = 0; // reset the init flags of the other modes
    off_init = 0;
    if (toggle0) {
      si5351.output_enable(SI5351_CLK0, 1); // enable signal output
      digitalWrite(test_lead, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      toggle0 = 0; // reset chopper toggle
    } else {
      si5351.output_enable(SI5351_CLK0, 0); // disable signal output
      digitalWrite(test_lead, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      toggle0 = 1; // set chopper toggle
    }    
  } else if (on_flag) { // steady on
    if (on_init == 0) { // only set this config on the initial Taranis switch setting and not re-set it at 20 Hz
      on_init = 1; // set init flags
      off_init = 0;
      si5351.output_enable(SI5351_CLK0, 1); // enable signal output
      digitalWrite(test_lead, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
    }    
  } else if (off_flag) { // steady off
    if (off_init == 0) { // only set this config on the initial Taranis switch setting and not re-set it at 20 Hz
      off_init = 1; // set init flags
      on_init = 0;
      si5351.output_enable(SI5351_CLK0, 0); // disable signal output
      digitalWrite(test_lead, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }    
  }
  cli();
}

ISR(TIMER1_CAPT_vect) {// timer/counter 1 input capture interrupt to read the PWM output of the X8R radio (channel 6 right now)
  if (not bit_is_set(TCCR1B, ICES1)) { // falling edge
    lowval = ICR1L; // get counter value
    highval = ICR1H;
    falling = lowval | highval << 8;
    if (rising > falling) { // if an overflow happened between rising and falling
      falling += count_top; // wrapping;
    }
    interval = falling - rising; // calculate the interval
    dataflag = 1; // tell the loop there's data
  }
  else { // rising edge
    lowval = ICR1L; // get counter value
    highval = ICR1H;
    rising = lowval | highval << 8;
    dataflag = 0; // need to wait for falling edge
  }
  TCCR1B ^= bit(ICES1); // in either case, flip the rising/falling edge detect bit
}

void loop() {
  // put your main code here, to run repeatedly:
  // read chopper state from X8R PWM output
  if (dataflag) { // a PWM pulse has been detected and pulse width measured
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // atomic read from the input capture ISR to avoid any register update errors
      PWM = interval;
    }
  } 
  if((PWM>oldPWM?PWM-oldPWM:oldPWM-PWM) > 2*pwm_range) { // subtract the smaller value from the larger value of PWM vs oldPWM. If the difference is more than a few counts then the PWM value changed - print it
    oldPWM = PWM;
    Serial.println(PWM);
  }

  // set chopper state flags according to X8R PWM output
  if((PWM > down_pwm - pwm_range) && (PWM < down_pwm + pwm_range)) { // down position of Taranis switch D
    // steady OFF
    off_flag = 1;
    on_flag = 0;
    chop_flag = 0;
  } else if ((PWM > middle_pwm - pwm_range) && (PWM < middle_pwm + pwm_range)) { // middle position of Taranis switch D
    // ON, Chopping
    chop_flag = 1;
    on_flag = 0;
    off_flag = 0;
  } else if ((PWM > up_pwm - pwm_range) && (PWM < up_pwm + pwm_range)) { // up position of Taranis switch D
    // steady ON
    on_flag = 1;
    off_flag = 0;
    chop_flag = 0;
  }
}
