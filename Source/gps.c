#include "mw.h"
#include "board.h"

#ifndef sq
#define sq(x) ((x)*(x))
#endif

#define POSHOLD_IMAX           20       // degrees
#define POSHOLD_RATE_IMAX      20       // degrees
#define NAV_IMAX               20       // degrees


static PID_PARAM posholdPID_PARAM;
static PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;
PID_PARAM altPID_PARAM;

typedef struct gpsData_t {
    uint8_t state;                  // GPS thread state. Used for detecting cable disconnects and configuring attached devices
    uint8_t baudrateIndex;          // index into auto-detecting or current baudrate
    int errors;                     // gps error counter - crc error/lost of data/sync etc. reset on each reinit.
    uint32_t lastMessage;           // last time valid GPS data was received (millis)
    uint32_t lastLastMessage;       // last-last valid GPS message. Used to calculate delta.

    int ubx_init_state;             // state of ublox initialization
    const uint8_t *init_ptr;        // pointer to init strings
    int init_length;                // length of data pointed to by init_ptr
    int state_position;             // incremental variable for loops
    int config_position;            // position in ubloxConfigList
    uint32_t state_ts;              // timestamp for last state_position increment
} gpsData_t;

gpsData_t gpsData;

static void gpsNewData(uint8_t c);
static bool gpsNewFrame(uint8_t c);
static bool gpsNewFrameNMEA(char c);
static bool gpsNewFrameUBLOX(uint8_t data);
uint32_t GPS_coord_to_degrees(char *s);
static uint32_t grab_fields(char *src, uint8_t mult);

// Get the relevant P I D values and set the PID controllers
void gpsSetPIDs(void)
{
    posholdPID_PARAM.kP = (float)cfg.P8[PIDPOS] / 100.0f;
    posholdPID_PARAM.kI = (float)cfg.I8[PIDPOS] / 100.0f;
    posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    poshold_ratePID_PARAM.kP = (float)cfg.P8[PIDPOSR] / 10.0f;
    poshold_ratePID_PARAM.kI = (float)cfg.I8[PIDPOSR] / 100.0f;
    poshold_ratePID_PARAM.kD = (float)cfg.D8[PIDPOSR] / 1000.0f;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP = (float)cfg.P8[PIDNAVR] / 10.0f;
    navPID_PARAM.kI = (float)cfg.I8[PIDNAVR] / 100.0f;
    navPID_PARAM.kD = (float)cfg.D8[PIDNAVR] / 1000.0f;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    if (f.FIXED_WING) {
        altPID_PARAM.kP   = (float)cfg.P8[PIDALT] / 10.0f;
        altPID_PARAM.kI   = (float)cfg.I8[PIDALT] / 100.0f;
        altPID_PARAM.kD   = (float)cfg.D8[PIDALT] / 1000.0f;
    }
}

void gpsThread(void)
{
	// read out available GPS bytes
    if (core.gpsport) {
        while (serialTotalBytesWaiting(core.gpsport)) {
            gpsNewData(serialRead(core.gpsport));
		}
    }
}

/****************** PI and PID controllers for GPS ********************///32938 -> 33160



static float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = { 0, 0 };
float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
static void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (abs((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cosf(rads);
}


////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
static void GPS_distance_cm_bearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2, int32_t *dist, int32_t *bearing)
{
    float dLat = *lat2 - *lat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * 1.113195f;

    *bearing = 9000.0f + atan2f(-dLat, dLon) * 5729.57795f;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

static void gpsNewData(uint8_t c)
{
	static uint32_t nav_loopTimer;
	int32_t dist;
    int32_t dir;
    int16_t speed;
	
	if (gpsNewFrame(c)) {
        // new data received and parsed, we're in business
		gpsData.lastLastMessage = gpsData.lastMessage;
        gpsData.lastMessage = millis();
        sensorsSet(SENSOR_GPS);
        if (GPS_update == 1) {
            GPS_update = 0;
		} else {
            GPS_update = 1;
		}
		
		if (f.GPS_FIX && GPS_numSat >= 5) {
            if (!f.ARMED && !f.FIXED_WING) {
                f.GPS_FIX_HOME = 0;
			}
            if (!f.GPS_FIX_HOME && f.ARMED) {
                GPS_reset_home_position();
			}
			// Apply moving average filter to GPS data
#if defined(GPS_FILTERING)

#endif			
			// dTnav calculation
            // Time for calculating x,y speed and navigation pids
            dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
            nav_loopTimer = millis();
            // prevent runup from bad GPS
            dTnav = min(dTnav, 1.0f);

            // calculate distance and bearings for gui and other stuff continously - From home to copter
            GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
            GPS_distanceToHome = dist / 100;
            GPS_directionToHome = dir / 100;

            if (!f.GPS_FIX_HOME) {      // If we don't have home set, do not display anything
                GPS_distanceToHome = 0;
                GPS_directionToHome = 0;
            }
			
			
			
			
		}
		
	}
}

void GPS_reset_home_position(void)
{
    if (f.GPS_FIX && GPS_numSat >= 5) {
        GPS_home[LAT] = GPS_coord[LAT];
        GPS_home[LON] = GPS_coord[LON];
        GPS_calc_longitude_scaling(GPS_coord[LAT]); // need an initial value for distance and bearing calc
        nav_takeoff_bearing = heading;              // save takeoff heading
        //Set ground altitude
        GPS_home[ALT] = GPS_altitude;
        f.GPS_FIX_HOME = 1;
    }
}

static bool gpsNewFrame(uint8_t c)
{	
	switch (mcfg.gps_type) {
        case GPS_NMEA:         			// NMEA
        case GPS_MTK_NMEA:      		// MTK in NMEA mode
            return gpsNewFrameNMEA(c);
        case GPS_UBLOX:         		// UBX binary
            return gpsNewFrameUBLOX(c);
        case GPS_MTK_BINARY:    		// MTK in BINARY mode (TODO)
            return false;
    }

    return false;
}

static bool gpsNewFrameUBLOX(uint8_t data)
{
	
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t err)
{
    if (err > 18000)
        err -= 36000;
    if (err < -18000)
        err += 36000;
    return err;
}

//#define NO_FRAME   0
//#define FRAME_GGA  1
//#define FRAME_RMC  2
enum {
	NO_FRAME=0,
	FRAME_GGA,
	FRAME_RMC
};

typedef struct gpsMessage_t {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    uint16_t altitude;
    uint16_t speed;
    uint16_t ground_course;
} gpsMessage_t;

static bool gpsNewFrameNMEA(char c)
{
	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, gps_frame = NO_FRAME;
	static gpsMessage_t gps_msg;
	
	switch (c) {
		case	'$':
			param = 0;
			offset = 0;
			parity = 0;			
			break;
		
		case	',':			
		case	'*':
			string[offset] = 0;
            if (param == 0) {       // frame identification
                gps_frame = NO_FRAME;
                if (string[0] == 'G' && string[1] == 'N' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') {
                    gps_frame = FRAME_GGA;
				}
                if (string[0] == 'G' && string[1] == 'N' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') {
                    gps_frame = FRAME_RMC;
				}
            }
			
			switch (gps_frame) {
                case FRAME_GGA:        // ************* GPGGA FRAME parsing
                    switch (param) {
                        case 2:
                            gps_msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_msg.latitude *= -1;
                            break;
                        case 4:
                            gps_msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_msg.longitude *= -1;
                            break;
                        case 6:
                            f.GPS_FIX = string[0] > '0'? true:false;
                            break;
                        case 7:
                            gps_msg.numSat = grab_fields(string, 0);
                            break;
                        case 9:
                            gps_msg.altitude = grab_fields(string, 0);     // altitude in meters added by Mis
                            break;
                    }
                    break;
			
				case FRAME_RMC:        // ************* GPRMC FRAME parsing
                    switch (param) {
                        case 7:
                            gps_msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                    }
                    break;
				}
				param++;
				offset = 0;
				if (c == '*')
					checksum_param = 1;
				else
					parity ^= c;
				break;
				
		case '\r':
        case '\n':
            if (checksum_param) {   //parity checksum
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    switch (gps_frame) {
                        case FRAME_GGA:
                            frameOK = 1;
							//GPS_numSat = gps_msg.numSat;
                            if (f.GPS_FIX) {
                                GPS_coord[LAT] = gps_msg.latitude;
                                GPS_coord[LON] = gps_msg.longitude;
                                GPS_numSat = gps_msg.numSat;
                                GPS_altitude = gps_msg.altitude;
                                if (!sensors(SENSOR_BARO) && f.FIXED_WING)
                                    EstAlt = (GPS_altitude - GPS_home[ALT]) * 100;    // Use values Based on GPS
                            }
                            break;

                        case FRAME_RMC:
                            GPS_speed = gps_msg.speed;
                            GPS_ground_course = gps_msg.ground_course;
                            if (!sensors(SENSOR_MAG) && GPS_speed > 100) {
                                GPS_ground_course = wrap_18000(GPS_ground_course * 10) / 10;
                                heading = GPS_ground_course / 10;    // Use values Based on GPS if we are moving.
                            }
                            break;
                    }
                }
            }
            checksum_param = 0;
					
			break;
		
		default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
            break;
	}
	
	return frameOK;
}

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char *s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    int i;

    // scan for decimal point or end of field
    for (p = s; isdigit((unsigned char)*p); p++) {
        if (p >= s + 15)
            return 0; // stop potential fail
    }
    q = s;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.') {
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit((unsigned char)*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{
    // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
        if (i >= 15)
            return 0; // out of bounds
    }
    return tmp;
}
