#include <Arduino.h> // Standard Arduino library
#include <kinetis.h> // Provides access to Kinetis K20 registers

// --- Comparator Reference Voltage Settings ---
// The Kinetis K20 has an internal 6-bit Digital-to-Analog Converter (DAC)
// that can be used as a reference for the comparator.
// DAC output voltage = VREF_EXT_HIGH * (VOSEL / 63)
// For Teensy 3.2, VREF_EXT_HIGH is 3.3V (VDDA).
// A VOSEL value of 31 (approx mid-range) will give ~1.65V reference.
#define DAC_VOSEL_VALUE 0 // 0-63, sets the reference voltage for the comparator

// --- Serial Report Interval ---
#define SERIAL_REPORT_INTERVAL_US 8000 // Report every x microseconds
#define MOTOR_CONTROL_LOOP_INTERVAL 10000

//analog inputs from poteniometers
#define PIN_VERT_ANALOG 22
#define PIN_HORIZ_ANALOG 23


#define PIN_DEBUG 15
#define PIN_COMPARATOR_OUTPUT 13 // not used explicitly -- pin 13 is always the output of the comparator when setup via registers

//PWM control of motors
#define MOTOR_PWM_BITS 10
#define MOTOR_PWM_RANGE ( 1 << MOTOR_PWM_BITS)
#define MOTOR_PWM_FREQUENCY 18000

#define PIN_MOTOR_VERT_H 5
#define PIN_MOTOR_VERT_L 6
#define PIN_MOTOR_VERT_EN 8

#define PIN_MOTOR_HORIZ_H 3
#define PIN_MOTOR_HORIZ_L 4
#define PIN_MOTOR_HORIZ_EN 7


#define HORIZ_START 34000
#define HORIZ_END 48000

#define VERT_START 20000  //2000
#define VERT_END 55000


IntervalTimer sendDataTimer;
IntervalTimer motorLoopTimer;

unsigned long lastReportTime = 0;
unsigned long currentPulseCount = 0; // Variable to store the LPTMR pulse count
volatile unsigned long lptmrOverflowCount = 0; // Counts 16-bit LPTMR overflows
unsigned long previousPulseCount =0;

volatile unsigned long current_pos_vert, current_pos_horiz;
volatile float set_pos_vert, set_pos_horiz;
volatile int beam_flyback = 0;

void setup() {
  // Initialize USB Serial communication
  Serial.begin(115200);  // Speed over USB is always fast-as-possible. Baudrate makes no difference
  Serial1.begin(9600,SERIAL_8N1_RXINV_TXINV);  // Hardware serial to the X-ray controller.  Baud and inversion is critical
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (up to 5 seconds)

  sendDataTimer.begin(sendUSBData, SERIAL_REPORT_INTERVAL_US);
  motorLoopTimer.begin(motor_control_loop, MOTOR_CONTROL_LOOP_INTERVAL);

  analogReadResolution(16);
  analogReadAveraging(32);  // Conversion time for 16 bit, 32 averages is about 100 microseconds

  pinMode(PIN_DEBUG, OUTPUT);



  pinMode(PIN_MOTOR_VERT_H, OUTPUT);
  pinMode(PIN_MOTOR_VERT_L, OUTPUT);
  pinMode(PIN_MOTOR_VERT_EN, OUTPUT);

  pinMode(PIN_MOTOR_HORIZ_H, OUTPUT);
  pinMode(PIN_MOTOR_HORIZ_L, OUTPUT);
  pinMode(PIN_MOTOR_HORIZ_EN, OUTPUT);

  analogWriteFrequency(PIN_MOTOR_VERT_H, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_MOTOR_VERT_L, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_MOTOR_HORIZ_L, MOTOR_PWM_FREQUENCY);
  analogWriteFrequency(PIN_MOTOR_HORIZ_H, MOTOR_PWM_FREQUENCY);

  analogWriteResolution(MOTOR_PWM_BITS);
  analogWrite(PIN_MOTOR_VERT_H, 0);
  analogWrite(PIN_MOTOR_VERT_L, 0);
  analogWrite(PIN_MOTOR_HORIZ_H, 0);
  analogWrite(PIN_MOTOR_HORIZ_L, 0);
  
  digitalWrite(PIN_MOTOR_VERT_EN, 1);
  digitalWrite(PIN_MOTOR_HORIZ_EN, 1);

  set_pos_vert = VERT_START;
  set_pos_horiz = HORIZ_START;

  // --- 1. Enable Clocks for Peripherals ---
  // Enable clock to Comparator 0 (CMP0)
  SIM_SCGC4 |= SIM_SCGC4_CMP;
  // Enable clock to Port C (where CMP0_IN0 and CMP0_OUT are located)
  SIM_SCGC5 |= SIM_SCGC5_PORTC;
  // Enable clock to Low Power Timer (LPTMR)
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER;

  // --- 2. Configure Pin Muxing for CMP Input and Output ---

  // Configure PTC6 (Teensy pin 11) for CMP0_IN0 (Analog function)
  // Set MUX to 0 for Analog mode.
  // Pull-up/down resistors are typically disabled for analog inputs.
  PORTC_PCR6 = PORT_PCR_MUX(0); // MUX field 000 for analog

  // Configure PTC5 (Teensy Pin 13) for CMP0_OUT function
  // Refer to K20 manual for MUX value for CMP0_OUT (usually MUX 6 or 7)
  // For PTC5, MUX 6 routes CMP0_OUT.
  PORTC_PCR5 = PORT_PCR_MUX(6); // MUX field 110 for CMP0_OUT

  // --- 3. Configure Analog Comparator 0 (CMP0) ---

  // CMP0_CR1 (Control Register 1)
  // - CMP_CR1_EN: Enable Comparator (Bit 0)
  // - CMP_CR1_OPE: Comparator Output Pin Enable (Bit 1) - routes COUT to a physical pin
  // - CMP_CR1_PMODE: Power Mode Select (Bit 3) - 0 for low-speed/low-power, 1 for high-speed
  // - CMP_CR1_INV: Invert Comparator Output (Bit 2) - 0 for non-inverted
  CMP0_CR1 = CMP_CR1_EN | // Enable CMP0
             CMP_CR1_OPE | // Enable output pin functionality
             CMP_CR1_PMODE; // High speed mode (optional, for faster response)

  // CMP0_CR0 (Control Register 0)
  // - HYST (Bits 1-0): Hysteresis Control - 00 for 0mV, 01 for 6mV, 10 for 12mV, 11 for 24mV
  // Hysteresis helps prevent oscillations when input is near reference.
  // Setting HYST to 0x02 for 12mV hysteresis.
  CMP0_CR0 = 0x02; // Set 12mV hysteresis (optional, directly setting bits 1:0 to 01b)

  // CMP0_MUXCR (Mux Control Register)
  // - CMP_MUXCR_PSEL(x): Positive Input Select (Bits 5-3) - 000 selects CMP0_IN0
  // - CMP_MUXCR_MSEL(x): Minus Input Select (Bits 2-0) - 111 selects the internal DAC output
  CMP0_MUXCR = CMP_MUXCR_PSEL(0) | // Select CMP0_IN0 (PTC0) as positive input
               CMP_MUXCR_MSEL(7); // Select internal DAC as negative input

  // CMP0_DACCR (DAC Control Register)
  // - CMP_DACCR_DACEN: DAC Enable (Bit 7) - enable internal 6-bit DAC
  // - CMP_DACCR_VRSEL: Voltage Reference Select (Bit 6) - 0 for VREF_OUT, 1 for VDDA
  // - CMP_DACCR_VOSEL(x): DAC Output Voltage Select (Bits 5-0) - 0-63 for voltage division
  CMP0_DACCR = CMP_DACCR_DACEN | // Enable the internal DAC
               CMP_DACCR_VRSEL | // Use VDDA as DAC reference (Teensy 3.2 uses 3.3V supply)
               CMP_DACCR_VOSEL(DAC_VOSEL_VALUE); // Set DAC output voltage based on VOSEL_VALUE


  // --- 4. Configure Low-Power Timer (LPTMR) for Pulse Counting ---

  // Disable LPTMR before configuration
  LPTMR0_CSR &= ~LPTMR_CSR_TEN;

  // LPTMR0_PSR (Prescale and Clock Select Register)
  // - PBYP: Prescaler Bypass Enable (Bit 2) - 1 to bypass the prescaler, count every pulse
  // - PCS (Bits 1-0): Prescaler Clock Select (Not strictly used for pulse counting when PBYP=1,
  //                   but setting to 00b (Peripheral Clock) as a safe default).
  LPTMR0_PSR = LPTMR_PSR_PBYP | // Bypass the prescaler
               LPTMR_PSR_PCS(0); // Peripheral clock (default, not used if PBYP is enabled)

  // LPTMR0_CMR (Compare Register)
  // Set compare value to 0. In free-running mode, this is not critical for counting
  // but explicitly setting it to 0 ensures the counter starts from 0 or rolls over.
  LPTMR0_CMR = 0xFFFF; // Not critical for free-running pulse counting, but good practice.

  // LPTMR0_CSR (Control Status Register)
  // - TMS: Timer Mode Select (Bit 1) - 1 for Pulse Counter mode
  // - TFC: Timer Free-Running Counter (Bit 2) - 1 to allow counter to free-run (don't stop at CMR)
  // - TPS (Bits 5-4): Timer Pin Select - 10b (2) selects CMP0 output as pulse input
  // - TIE: Timer Interrupt Enable (Bit 6) - 0 for no interrupt (we'll poll)
  // - TCF: Timer Compare Flag (Bit 7) - Clear by writing 1 (important before enabling)
  LPTMR0_CSR = LPTMR_CSR_TMS |    // Pulse Counter mode
               LPTMR_CSR_TFC |    // Free-running counter
               LPTMR_CSR_TPS(0) | // Select CMP0 output (Pulse input 0) as source
               LPTMR_CSR_TIE |    // Enable Timer Interrupt (allows TCF to set)
               LPTMR_CSR_TCF;     // Clear Timer Compare Flag (write 1 to clear)


  LPTMR0_CSR |= LPTMR_CSR_TEN; // Enable the LPTMR
  NVIC_ENABLE_IRQ(IRQ_LPTMR);



 Serial1.print("CPA11111100");
  Serial1.print('\r');
  delay(100);

 Serial1.print("RESPA0");
  Serial1.print('\r');
  delay(100);
 Serial1.print("RESPA1");
  Serial1.print('\r');
  delay(100);

 Serial1.print("VA3600");  // Should work up to 4095 maximum, but there was a lot of arcing and instability
  Serial1.print('\r');
  delay(100);

Serial1.print("VB4095");
  Serial1.print('\r');
  delay(100);

Serial1.print("SETPA1");
  Serial1.print('\r');
  delay(100);

Serial1.print("RESPA1");
  Serial1.print('\r');
  delay(100);

Serial1.print("RPA2");
  Serial1.print('\r');
  delay(100);

if (Serial1.available()) {
    if(Serial1.read()=='0')
      {
        Serial.println("X-ray ready");
      }
      else
      {
        Serial.println("X-ray not ready");
      }
   }

}


void lptmr_isr(void) 
{
  // Clear the Timer Compare Flag (TCF) by writing a 1 to it.
  // This is crucial to acknowledge the interrupt and allow it to fire again.
  LPTMR0_CSR |= LPTMR_CSR_TCF;

  // Increment the global overflow counter
  lptmrOverflowCount++;
}


void motor_control_loop()
{
  static unsigned int prev_vert_position, prev_horiz_position; 
  static float vert_integrated_error, horiz_integrated_error;

  //float cur_horiz_speed = (current_pos_horiz - prev_horiz_position) / MOTOR_CONTROL_LOOP_INTERVAL;
  motor_vert_control(((float)set_pos_vert - (float)current_pos_vert)/1100.0);
  horiz_integrated_error = horiz_integrated_error + (((float)set_pos_horiz - (float)current_pos_horiz) * .00000001);
 horiz_integrated_error = constrain(horiz_integrated_error, -.2, .2);
  motor_horiz_control((((float)set_pos_horiz - (float)current_pos_horiz)/2500.0) + 0.001*((float)prev_horiz_position - (float)current_pos_horiz) );
 
 prev_horiz_position = current_pos_horiz;
 prev_vert_position = current_pos_vert;



}

void motor_vert_control(float control_value)
{
  control_value = constrain(control_value, -1, 1);
  if(control_value > 0)
    {
      analogWrite(PIN_MOTOR_VERT_H, (abs(control_value) * MOTOR_PWM_RANGE));
      analogWrite(PIN_MOTOR_VERT_L, 0);
    }
  else if(control_value < 0)
    {
      analogWrite(PIN_MOTOR_VERT_L, (abs(control_value) * MOTOR_PWM_RANGE));
      analogWrite(PIN_MOTOR_VERT_H, 0);
    }
  else
  {
    analogWrite(PIN_MOTOR_VERT_L, 0);
    analogWrite(PIN_MOTOR_VERT_H, 0);
  }
}

void motor_horiz_control(float control_value)
{
  control_value = constrain(control_value, -1, 1);
  if(control_value < 0)
    {
      analogWrite(PIN_MOTOR_HORIZ_H, (abs(control_value) * MOTOR_PWM_RANGE));
      analogWrite(PIN_MOTOR_HORIZ_L, 0);
    }
  else if(control_value > 0)
    {
      analogWrite(PIN_MOTOR_HORIZ_L, (abs(control_value) * MOTOR_PWM_RANGE));
      analogWrite(PIN_MOTOR_HORIZ_H, 0);
    }
  else
  {
    analogWrite(PIN_MOTOR_HORIZ_L, 0);
    analogWrite(PIN_MOTOR_HORIZ_H, 0);
  }
}





void sendUSBData()
{
digitalWriteFast(PIN_DEBUG, HIGH);
current_pos_vert = analogRead(PIN_VERT_ANALOG);
current_pos_horiz = analogRead(PIN_HORIZ_ANALOG);

LPTMR0_CNR = 0;  //MUST BE WRITTEN BEFORE READING
currentPulseCount = LPTMR0_CNR + (lptmrOverflowCount<<16) ;

if (beam_flyback == 0)
{
  Serial.print(current_pos_horiz);
  Serial.print(',');
  Serial.print(current_pos_vert);
  Serial.print(',');
  Serial.flush();
  Serial.println(currentPulseCount - previousPulseCount);
}
previousPulseCount = currentPulseCount;
digitalWriteFast(PIN_DEBUG, LOW);
}

void loop() {
  unsigned long currentTime = millis();
  static int scan_running = 0;
  static int horiz_dir = 0;
  static unsigned long prevTime = 0;

if (scan_running == 1 && ((currentTime - prevTime) > 10))
  {
    if (current_pos_horiz > (HORIZ_END-600))
      {
        horiz_dir = 0;
        beam_flyback = 0;
      }
    if(current_pos_horiz < (HORIZ_START+600))
      {
        horiz_dir = 1;
        beam_flyback = 1;
      }

    if(current_pos_vert > VERT_END - 100)
      {
        scan_running = 0;
      }

    if(horiz_dir == 1)
      {
        set_pos_horiz = set_pos_horiz + 150;  // 80 = 2.5 seconds per sweep 
      }
    if(horiz_dir == 0)
      {
        set_pos_horiz = set_pos_horiz - 35;   // was 25
      }
    set_pos_vert = set_pos_vert + 0.3;  //   1 = 8 minutes per scan
    

      set_pos_horiz = constrain(set_pos_horiz, HORIZ_START -1, HORIZ_END + 1);
      set_pos_vert = constrain(set_pos_vert, VERT_START -1, VERT_END + 1);
  prevTime = currentTime;
  }



  if (Serial.available()) {       //USB serial
    switch (Serial.read()) {
        case '1':
          Serial1.print("SETPA0");
          Serial1.print('\r');
          Serial.println("X-ray on");
          break;

        case '0':
          Serial1.print("RESPA0");
          Serial1.print('\r');
          Serial.println("X-ray off");
        break;
      

        case '3':
          set_pos_vert = VERT_START;
          set_pos_horiz = HORIZ_START;
          scan_running = 0;
          break;
        
        case '4':
          scan_running = 1;
          break;

       }
   }
  
}
