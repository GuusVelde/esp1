// Auteur : Guus van der Velde
// Student: 1035940

//uart_init
const int BAUD = 9600;

//parse_data

#define C_ACTIVE 7 //length of active code word ("active:")
#define C_FREQ 5   //length of frequency code word ("freq:")

//NTC:

const float NTC = 3950; //adjust based on thermistor
const int NTC_OFFSET = 6; //adjust based on thermistor
const float NTC_OHM = 10000.0; //adjust based on thermistor
const int AVG_TMP = 25; // Average temperature. Adjust based on target area
const float NTC_VOLT = 3.3; //adjust based on microcontroller
const float ADC_MAX = 4095.0; //adjust based on microcontroller