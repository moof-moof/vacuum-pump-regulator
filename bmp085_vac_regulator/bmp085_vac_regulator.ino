/* NOTE TO MYSELF:
 * The board used is the Arduino Pro Mini clone (Xino 5V/3V3 by Ciseco), which seems to have a
 * bootloader that make it necessary to identify the board as a UNO for it to be responding,,,
*/


#include <Wire.h>

#define I2C_ADDRESS 0x77
#define SDA_PIN     A4
#define SCL_PIN     A5
#define RELAY_PIN   12
#define BLINK_PIN   13
#define AMBIENT_KPA 101     // Standard athmospheric pressure is circa 101 kiloPascals (1013.25 mbar)
#define MAX_VAC     7       // Maximum permitted relative vacuum as kPa less than ambient pressure
#define MIN_VAC     3       // Minimum ditto

const unsigned char oversampling_setting = 3;
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 };

int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);

long pressure_ambient = 0;
bool pumping = true;
bool blinkState = false;


/// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY


void setup()
{
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BLINK_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(BLINK_PIN, LOW);
    
    Serial.begin(9600);
    Serial.println("Setting up BMP085");
    
    Wire.begin();
    digitalWrite(SDA_PIN, 0); // deactivate internal pullups just set for TWI in Wire library.
    digitalWrite(SCL_PIN, 0);
    bmp085_get_cal_data();
    int  temp_start = 0;
    bmp085_read_temperature_and_pressure( &temp_start, &pressure_ambient);

    Serial.print("Ambient pressure is ");
    Serial.print((pressure_ambient / 100), DEC); // Convert to kPa (truncated), normally 100-101 kPa
    Serial.println(" mbar (hPa)");
    Serial.println("***********************************");
    delay(1000);
}


void loop()
{
    int  temperature = 0;
    long pressure = 0;
    
    bmp085_read_temperature_and_pressure(&temperature, &pressure);

    pump_controller(pressure / 1000);
}

/// YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY


void pump_controller(uint8_t VacAbsKPa) {

    int8_t vac = AMBIENT_KPA - VacAbsKPa ;              // vac as a positive value (kPa)
    
    
    if(vac > MAX_VAC) {
        digitalWrite(RELAY_PIN, HIGH);                  // That is: break pump power circuit
        pumping = false;
    }
    else if(!pumping && (vac < MIN_VAC)) {
        digitalWrite(RELAY_PIN, LOW);                   // Pump up the vac!
        pumping = true;
    }
    Serial.print("  "); Serial.println(vac, DEC);

    blinkState = !blinkState;
    digitalWrite(BLINK_PIN, blinkState);
    delay(1000);
}

