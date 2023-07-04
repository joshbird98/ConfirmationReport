/*Floating Picoammeter rev 6 with digital filter Software
*/

// INCLUDES
#include <SPI.h>
#include <SoftwareSerial.h>
#include <IIRFilter.h>

// PIN DEFINES
#define ADC_CS A0
#define REED_RELAY_CONTROL A1
#define FIBRE_TRANSMIT A2
#define DEBUG_LED 4
#define SEL_RNG_1 8
#define SEL_RNG_2 7
#define SEL_RNG_3 10
#define SEL_RNG_4 9

// CONSTANTS

// GLOBAL VARIABLES, ARRAYS & OBJECTS
uint32_t adc_tally;
uint32_t adc_readings;
uint32_t large_tally;
uint16_t large_tally_length;
uint8_t range;
uint16_t last_adc_value;

uint8_t t1_flag;
uint32_t i;
long int lastTime;

unsigned long calibration_time; // time in millis() of most recent calibration
#define CALIBRATION_PERIOD 1000*60*2 //2 minutes
const double ADC_samplefreq  = 1600;  // this seems to actually end up as 1577.5 Hz

SoftwareSerial fibreOptic (A4, FIBRE_TRANSMIT, 1);

// FUNCTIONS

// sets the relays for the desired range, ensuring only one relay coil is driven
// includes small delay to ensure relay settles

void set_range(uint8_t rangeSelection)
{
  pinMode(REED_RELAY_CONTROL, OUTPUT);
  pinMode(SEL_RNG_1, OUTPUT);
  pinMode(SEL_RNG_2, OUTPUT);
  pinMode(SEL_RNG_3, OUTPUT);
  pinMode(SEL_RNG_4, OUTPUT);
  digitalWrite(REED_RELAY_CONTROL, LOW);
  digitalWrite(SEL_RNG_1, LOW);
  digitalWrite(SEL_RNG_2, LOW);
  digitalWrite(SEL_RNG_3, LOW);
  digitalWrite(SEL_RNG_4, LOW);
  if (rangeSelection > 0) digitalWrite(REED_RELAY_CONTROL, HIGH);
  if (rangeSelection == 1) digitalWrite(SEL_RNG_1, HIGH);
  if (rangeSelection == 2) digitalWrite(SEL_RNG_2, HIGH);
  if (rangeSelection == 3) digitalWrite(SEL_RNG_3, HIGH);
  
}

void reset_counter()
{
  adc_tally = 0;
  adc_readings = 0;
}

// interface with the MCP33131, returns a 16-bit measaurement
uint16_t read_adc()
{
  digitalWrite(ADC_CS, LOW);
  uint8_t msb = SPI.transfer(0xFF);
  uint8_t lsb = SPI.transfer(0xFF);
  digitalWrite(ADC_CS, HIGH);

  uint16_t value = (((uint16_t)msb << 8) | lsb);
  return value;
}

//interface with the MCP33131, performs recalibration if needed, takes up to 650ms
void calibrate_adc(uint8_t force_calibrate)
{
  //check if enough time has passed to need recalibration
  unsigned long now = millis();
  unsigned long elapsed = now - calibration_time;
  
  if ((elapsed > CALIBRATION_PERIOD) or (force_calibrate == 1))
  {
    digitalWrite(ADC_CS, LOW);
    for (uint8_t i = 0; i < 129; i++) //1024 clock cycles
    {
      SPI.transfer(0xFF);
    }
    delay(650); // max delay needed for t_cal
    digitalWrite(ADC_CS, HIGH);
    set_range(0);        // set relays and delay
    calibration_time = now;
  }
}

void flash_led()
{
  digitalWrite(DEBUG_LED, HIGH);
  delay(100);
  digitalWrite(DEBUG_LED, LOW);
}

void check_range(uint32_t value)
{
  //Serial.print("RANGE :");
  //Serial.println(range);
  //Serial.println(value);
  uint8_t new_range = range;
  if ((value < 4096) or (value > 61439))
  {
    if (range < 3) new_range += 1;  // range should be increased
  }
  // if ((value > 32550) and (value < 33005)) good for range 1 going into range 0
  if ((value > 32550) and (value < 33005))
  {
    if (range > 0) new_range -= 1;  // range should be decreased
  }
  if (new_range != range)
  {
    //Serial.print("OLD RANGE: ");
    //Serial.print(range);
    //Serial.print(" NEW RANGE: ");
    //Serial.println(new_range);
    range = new_range;
    set_range(range);
    adc_tally = 0;
    adc_readings = 0;
  }
}

void setup() {
  last_adc_value = 0;
  i = 0;
  lastTime = millis();
  t1_flag = 0;
  adc_tally = 0;
  adc_readings = 0;
  range = 0;
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
  pinMode(ADC_CS, OUTPUT);
  pinMode(FIBRE_TRANSMIT, OUTPUT);
  fibreOptic.begin(74880);
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
  //handle_calibration();
  set_range(range);
  calibrate_adc(1); // forces recalibration, updates time
  flash_led();

  //Use Timer1 for 200Hz interrupts
  cli();                    // Stop interrupts
  TCCR1A = 0;               // Set entire TCCR1A register to 0
  TCCR1B = 0;               // Same for TCCR1B
  TCNT1  = 0;               // Initialize counter value to 0
  OCR1A = (8000000 / ADC_samplefreq) - 1;              // = (8*10^6) / (1000*1) - 1
  TCCR1B |= (1 << WGM12);   // Turn on CTC mode
  TCCR1B |= (1 << CS10);    // Set CS10 bit for x1 prescaler
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  sei();                    // Allow interrupts
}

ISR(TIMER1_COMPA_vect)
{
  t1_flag = 1;
}

void loop() {
  uint16_t adc_value = read_adc();
  if ((adc_value > 64000) and (last_adc_value < 10000))
  {
    adc_value = 0;
  }
  if ((adc_value < 1500) and (last_adc_value > 55353))
  {
    adc_value = 65535;
  }
  adc_tally += adc_value;
  adc_readings += 1;
  last_adc_value = adc_value;
  
  if (t1_flag == 1) // ~1/ADC_samplefreq has passed
  {
    t1_flag = 0;
    uint16_t adc_average = adc_tally / adc_readings;
    large_tally += adc_average;
    // Send the data in binary format for efficiency. Receiver will interpret gaps.
    uint8_t data[3] = {range, (uint8_t)(adc_average >> 8), (uint8_t)(adc_average & 0xFF)};
    fibreOptic.write(data, 3);
    adc_tally = 0;
    adc_readings = 0;
    i += 1;
  }

  if (i >= 100)
  {
    check_range(large_tally / 100);
    //fibreOptic.print(range);
    large_tally = 0;
    i = 0;
  }
}
