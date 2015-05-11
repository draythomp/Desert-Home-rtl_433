#include "rtl_433.h"

// ** Acurite 5n1 functions **

char * acurite_winddirections[] = 
    {"NW",  // 0  315
     "WSW", // 1  247.5
     "WNW", // 2  292.5
     "W",   // 3  270
     "NNW", // 4  337.5
     "SW",  // 5  225
     "N",   // 6  0
     "SSW", // 7  202.5
     "ENE", // 8  67.5
     "SE",  // 9  135
     "E",   // 10 90
     "ESE", // 11 112.5
     "NE",  // 12 45
     "SSE", // 13 157.5
     "NNE", // 14 22.5
     "S"};  // 15 180

// this is a bitmapped byte to tell if the various styles of reports have
// come in.  Bit 0 is R1 first type, bit 2 is R1 type 2; the 5n1 sensor only
// sends two report types
uint8_t reportsSeen = 0;

// These are the sensors the the 5 in 1 weather head provides
struct {
    int     sensorId;
    time_t  siTime;
    char    channel;
    time_t  cTime;
    int     messageCaught;
    time_t  mcTime;
    int     battLevel;
    time_t  battTime;
    float   windSpeed;
    time_t  wsTime;
    char *  windDirection;
    time_t  wdTime;
    float   temperature;
    time_t  tTime;
    int     humidity;
    time_t  hTime;
    int     rainCounter;
    time_t  rcTime;
} weatherData;

void acuriteShowit(){

    // make sure enough reports have come in before reporting
    if( reportsSeen >= 3){  // Make sure both reports have come in
        fprintf(stdout, "{"
                        "\"sensorId\":{\"SID\":\"%d\",\"t\":\"%d\"},"
                        "\"channel\":{\"CH\":\"%c\",\"t\":\"%d\"},"
                        "\"messageCaught\":{\"MC\":\"%d\",\"t\":\"%d\"},"
                        "\"battLevel\":{\"BAT\":\"%d\",\"t\":\"%d\"},"
                        "\"windSpeed\":{\"WS\":\"%.1f\",\"t\":\"%d\"},"
                        "\"windDirection\":{\"WD\":\"%s\",\"t\":\"%d\"},"
                        "\"temperature\":{\"T\":\"%.1f\",\"t\":\"%d\"},"
                        "\"humidity\":{\"H\":\"%d\",\"t\":\"%d\"},"
                        "\"rainCounter\":{\"RC\":\"%d\",\"t\":\"%d\"}}\n",
                weatherData.sensorId, (int)weatherData.siTime,
                weatherData.channel, (int)weatherData.cTime,
                weatherData.messageCaught, (int)weatherData.mcTime,
                weatherData.battLevel, (int)weatherData.battTime,
                weatherData.windSpeed, (int)weatherData.wsTime,
                weatherData.windDirection, (int)weatherData.wdTime,
                weatherData.temperature,  (int)weatherData.tTime,
                weatherData.humidity,  (int)weatherData.hTime,
                weatherData.rainCounter,  (int)weatherData.rcTime
                );
        fflush(stdout);
    }
}
static int acuriteParity(uint8_t v){
    // returns 1 if parity odd, 0 if parity even
    v ^= v >> 4;
    v &= 0xf;
    return (0x6996 >> v) & 1;
}

static int acurite_crc(uint8_t row[BITBUF_COLS], int cols) {
    // sum of first n-1 bytes modulo 256 should equal nth byte
    int i;
    int sum = 0;
    // Do the checksum
    for ( i=0; i < cols; i++)
        sum += row[i];
    if ( sum % 256 != row[cols] )
        return 0; // Bail out, it didn't pass
    // Check the parity of bytes 2-6. The first and second bytes are
    // status and sensor ID, the last byte is the checksum.
    for (i=2; i<cols-1; i++){
        if(acuriteParity(row[i]) == 1){
            if (debug_output) {
                fprintf(stderr, "Parity error byte %d, %02X\n", i, row[i]);
            }
            return 0; // Bail out, it didn't pass parity
        }
    }
    return 1;
}

static int acurite_detect(uint8_t *pRow) {
    int i;
    if ( pRow[0] != 0x00 ) {
        // invert bits due to wierd issue
        for (i = 0; i < 8; i++)
            pRow[i] = ~pRow[i] & 0xFF;
        //pRow[0] |= pRow[8];  // fix first byte that has mashed leading bit

        if (acurite_crc(pRow, 7))
            return 1;  // passes crc check
    }
    return 0;
}

static float acurite_getTemp (uint8_t highbyte, uint8_t lowbyte) {
    // range -40 to 158 F
    int highbits = (highbyte & 0x0F) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp = (rawtemp - 400) / 10.0;
    return temp;
}

static float acurite_getWindSpeed (uint8_t highbyte, uint8_t lowbyte) {
    // This is in MPH, not KPH
    int highbits = ( highbyte & 0x1F) << 3;
    int lowbits = ( lowbyte & 0x70 ) >> 4;
    float speed = (highbits | lowbits) / 2.0;
    return speed;
}

static char *acurite_getWindDirection (uint8_t byte) {
    int direction = byte & 0x0F;
    return acurite_winddirections[direction];
}

static int acurite_getHumidity (uint8_t byte) {
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}

static int acurite_getRainfallCounter (uint8_t highbyte, uint8_t lowbyte) {
    // range: 0 to 99.99 in, 0.01 in incr., rolling counter? 
    int highbits = (highbyte & 0x3F) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int raincounter = highbits | lowbits;
    return raincounter;
}
//Sensor ID is in the second byte
static int acurite_getSensorId(uint8_t byte){
    return byte;
}
// The high 2 bits of byte zero are the channel (bits 7,6)
//  00 = C
//  10 = B
//  11 = A
static char chLetter[4] = {'C','E','B','A'}; // 'E' stands for error

static char acurite_getChannel(uint8_t byte){
    int channel = (byte & 0xC0) >> 6;
    return chLetter[channel];
}
// The sensor sends the same data three times, each of these have 
// an indicator of which one of the three it is. This means the
// checksum and first byte will be different for each one.
// The bits 5,4 of byte 0 indicate which copy of the 65 bit data string
//  00 = first copy
//  01 = second copy
//  10 = third copy
//  1100 xxxx  = channel A 1st copy
//  1101 xxxx  = channel A 2nd copy
//  1110 xxxx  = channel A 3rd copy  
static int acurite_getMessageCaught(uint8_t byte){
    return (byte & 0x30) >> 4;
}
// So far, all that's known about the battery is that the
// third byte, high nibble has two values. 0xb0=low and 0x70=OK
// so this routine just returns the nibble shifted to make a byte
// for more work as time goes by
static int acurite_getBatteryLevel(uint8_t byte){
    return (byte & 0xf0) >> 4;
}

static int acurite5n1_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS],int16_t bits_per_row[BITBUF_ROWS]) {
    // acurite 5n1 weather sensor decoding for rtl_433
    // Jens Jensen 2014
    int i;
    uint8_t *buf = NULL;
    // run through rows til we find one with good crc (brute force)
    for (i=0; i < BITBUF_ROWS; i++) {
        if (acurite_detect(bb[i])) {
            buf = bb[i];
            break; // done
        }
    }

    if (buf) {
        // decode packet here
        fprintf(stderr, "Detected Acurite 5n1 sensor, %d bits\n",bits_per_row[1]);
        if (debug_output) {
            for (i=0; i < 8; i++)
                fprintf(stderr, "%02X ", buf[i]);
            fprintf(stderr, "CRC OK\n");
        }

        time_t seconds = time (NULL);
        
        // both message types have the common items
        // including wind speed
        weatherData.sensorId = acurite_getSensorId(buf[1]);
        weatherData.siTime = seconds;
        weatherData.channel = acurite_getChannel(buf[0]);
        weatherData.cTime = seconds;
        weatherData.messageCaught = acurite_getMessageCaught(buf[0]);
        weatherData.mcTime = seconds;
        weatherData.battLevel = acurite_getBatteryLevel(buf[2]);
        weatherData.battTime = seconds;
        weatherData.windSpeed = acurite_getWindSpeed(buf[3], buf[4]);
        weatherData.wsTime = seconds;
        if ((buf[2] & 0x0F) == 1) {
            // has wind direction, rainfall
            weatherData.windDirection = acurite_getWindDirection(buf[4]);
            weatherData.wdTime = seconds;
            weatherData.rainCounter = acurite_getRainfallCounter(buf[5], buf[6]);
            weatherData.rcTime = seconds;
            if(debug_output){
                fprintf(stderr,"Wind Speed: %.1f, ",weatherData.windSpeed);
                fprintf(stderr,"Wind Direction: %s, ",weatherData.windDirection);
                fprintf(stderr,"Rain Counter: %d, ",weatherData.rainCounter);
                fprintf(stderr,"\n");
            }
            reportsSeen |= 0x01; //I've seen report type 2 now
        } else if ((buf[2] & 0x0F) == 8) {
            // temp, Relative Humidity
            weatherData.temperature = acurite_getTemp(buf[4], buf[5]);
            weatherData.tTime = seconds;
            weatherData.humidity = acurite_getHumidity(buf[6]);
            weatherData.hTime = seconds;
            if(debug_output){
                fprintf(stderr,"Wind Speed: %.1f, ",weatherData.windSpeed);
                fprintf(stderr,"Temperature: %.1f, ",weatherData.temperature);
                fprintf(stderr,"Humidity: %d, ", weatherData.humidity);
                fprintf(stderr,"\n");
            }
            reportsSeen |= 0x02;  // I've seen report 1 type 2 now
        }
    }
    acuriteShowit();
    //if (debug_output)
    //   debug_callback(bb);
    return 1;
}


static int acurite_rain_gauge_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS]) {
    // This needs more validation to positively identify correct sensor type, but it basically works if message is really from acurite raingauge and it doesn't have any errors
    if ((bb[0][0] != 0) && (bb[0][1] != 0) && (bb[0][2]!=0) && (bb[0][3] == 0) && (bb[0][4] == 0)) {
	    float total_rain = ((bb[0][1]&0xf)<<8)+ bb[0][2];
		total_rain /= 2; // Sensor reports number of bucket tips.  Each bucket tip is .5mm
        fprintf(stdout, "AcuRite Rain Gauge Total Rain is %2.1fmm\n", total_rain);
		fprintf(stdout, "Raw Message: %02x %02x %02x %02x %02x\n",bb[0][0],bb[0][1],bb[0][2],bb[0][3],bb[0][4]);
        return 1;
    }
    return 0;
}

static int acurite_th_detect(uint8_t *buf){
    if(buf[5] != 0) return 0;
    uint8_t sum = (buf[0] + buf[1] + buf[2] + buf[3]) & 0xff;
    if(sum == 0) return 0;
    return sum == buf[4];
}
static float acurite_th_temperature(uint8_t *s){
    uint16_t shifted = (((s[1] & 0x0f) << 8) | s[2]) << 4; // Logical left shift
    return (((int16_t)shifted) >> 4) / 10.0; // Arithmetic right shift
}
static int acurite_th_callback(uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS]) {
    uint8_t *buf = NULL;
    int i;
    for(i = 0; i < BITBUF_ROWS; i++){
	if(acurite_th_detect(bb[i])){
            buf = bb[i];
            break;
        }
    }
    if(buf){
        fprintf(stdout, "Temperature event:\n");
        fprintf(stdout, "protocol      = Acurite Temp&Humidity\n");
        fprintf(stdout, "temp          = %.1f°C\n", acurite_th_temperature(buf));
        fprintf(stdout, "humidity      = %d%%\n\n", buf[3]);
        return 1;
    }

    return 0;
}

r_device acurite5n1 = {
    /* .id             = */ 10,
    /* .name           = */ "Acurite 5n1 Weather Station",
    /* .modulation     = */ OOK_PWM_RAW,
    /* .short_limit    = */ 70,
    /* .long_limit     = */ 240,
    /* .reset_limit    = */ 21000,
    /* .json_callback  = */ &acurite5n1_callback,
};

r_device acurite_rain_gauge = {
    /* .id             = */ 10,
    /* .name           = */ "Acurite 896 Rain Gauge",
    /* .modulation     = */ OOK_PWM_D,
    /* .short_limit    = */ 1744/4,
    /* .long_limit     = */ 3500/4,
    /* .reset_limit    = */ 5000/4,
    /* .json_callback  = */ &acurite_rain_gauge_callback,
};

r_device acurite_th = {
    /* .id             = */ 11,
    /* .name           = */ "Acurite Temperature and Humidity Sensor",
    /* .modulation     = */ OOK_PWM_D,
    /* .short_limit    = */ 300,
    /* .long_limit     = */ 550,
    /* .reset_limit    = */ 2500,
    /* .json_callback  = */ &acurite_th_callback,
};
