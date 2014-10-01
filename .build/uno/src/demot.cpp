#include <Arduino.h>
#include <stdio.h>
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#include "Time.h"
#include "TinyGPS.h"
static void say(byte level, char* msg);
static int uart_putchar (char c, FILE *stream);
void setup();
int readCompass();
void readGPS();
int mod(int value);
int get_hdg_diff(int heading1,int heading2);
void loop();
#line 1 "src/demot.ino"
///
/// Dermot the bottle boat
/// 
/// This is the control system for Dermot the bottle boat and is salvaged from one of Colin's moops. Not ideal as we
/// hacked this together one night but it seems to work so far.
///
/// This code has been specially commented for the people joining Aber Sailbot to get a idea about what is included in
/// a simple boat control system. Its for a motor boat not a sail boat. Some of this code may look a bit daughting to you
/// however its just because its a big ugly and was hacked together in a rush. I'm a bit too lazy to clear it up too. I 
/// should mention also that it really isn't complicated and Colin seems to of made it look more complicated, blame the 
/// lego man murderer!!!!
/// 
/// 
/// This has taught me Colin doesn't like super clean code
/// 
/// Commented, hacked and somewhat cleaned up by Jordan
///

// These are other libraries we are using
//#include <stdio.h>
//#include <Servo.h>
//#include <Wire.h>
//#include <SoftwareSerial.h>
//#include <math.h>
//#include "Time.h"
//#include "TinyGPS.h"

// For those of you that don't know what a #define is its a macro, kinda like a variable you can't change that saves
// you program memory space as the compiler will replace these with the actual value in the code. Hope that makes sense

#define HMC6343_ADDRESS         0x19    // I2C address
#define HMC6343_HEADING_REG     0x50    // Bearing, pitch and roll register

// IO pins
#define GPS_ENABLE_PIN          10

// Function macros
#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180

// Debug Levels
#define DEBUG_CRITICAL 1 // really important messages that we don't want to ignore and are prepared to sacrifice execution speed to see
#define DEBUG_IMPORTANT 2 // fairly important messages that we probably want to see, but might cause issues with execution speed
#define DEBUG_MINOR 3 // less important messages

#define DEBUG_THRESHOLD   DEBUG_MINOR // This is what level of debuging we want, 0 means no debuging information

#define WP_THRESHOLD 5 //how close (in metres) should we get before we change waypoint

// Variables
Servo rudderServo; // Servo object to control the rudder servo
SoftwareSerial gps_serial(11, 12);  // Creates a serial object which allows us to read serial data from pin 11 and write serial data 
                                    // using pin 12.

TinyGPS gps; // A object that will help us get GPS data

#define HEADING 0
#define WIND_DIR 1
#define ROLL 2
#define PITCH 3
#define RUDDER 4
#define SAIL 5
#define LAT 6
#define LON 7
#define TIME 8

// This struct contains all the data needed for the boat to decide what it wants to do and more. By using the struct we are grouping this 
// data together
struct Data{
  uint16_t heading;
  uint16_t wind_dir;
  int8_t roll;
  int8_t pitch;
  int8_t rudder;
  float lat;
  float lon;
  long unix_time;
} data;


//////////////////////////////////////////////////////////////////////////////////////////
/// All this code can really be ignored, its only purpose is so we can use the function
/// printf which makes it easier to print out data to the serial

// This function prints out debug information. Depending on the currently set debug level (DEBUG_THRESHOLD) some debug information may 
// not be printed.
static void say(byte level, char* msg)
{
  if(level<=DEBUG_THRESHOLD)
  {
    Serial.print("Debug");
    Serial.print(level);
    Serial.print(": [Ctrl] ");
    Serial.println(msg);
  }
}

//make printf work
static FILE uartout = {
  0} 
;

//debugging printf that prepends "Debug:" to everything and can be easily turned off
void dprintf(byte level, const char *fmt, ...)
{      
  
  if(level<=DEBUG_THRESHOLD)
  {
    printf("Debug%d: [Ctrl] ",level);
    va_list ap;
    va_start(ap, fmt);
    
    vprintf(fmt, ap);
    va_end(ap);
  }
}
// Required for the printf, essentially overwrites it.
static int uart_putchar (char c, FILE *stream)
{
  Serial.write(c) ;
  return 0 ;
}
//////////////////////////////////////////////////////////////////////////////////////////

//
// Ok stop ignoring code now!!
// 

//////////////////////////////////////////////////////////////////////////////////////////
/// This function is your typical arduino setup function
/// 
void setup() 
{
  Serial.begin(9600); //makes no difference on 32u4
  gps_serial.begin(4800); // setups serial communications for the GPS

  say(DEBUG_CRITICAL,"Control system start up");

  // required for printf ignore these next 3 lines
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
  dprintf(DEBUG_IMPORTANT,"Printf configured\r\n");

  delay(5000); // makes the serial print out look professional, everyone loves a good delay :P

  // Here we begin to setup the servos, on Dermot we have only one servo and that controls
  // the rudder
  say(DEBUG_IMPORTANT,"Setting up servos...");
  //Use .attach for setting up connection to the servo
  rudderServo.attach(5, 1060, 1920); // Attach, with the output limited
  rudderServo.writeMicroseconds(1500); // Centre it roughly
  say(DEBUG_IMPORTANT,"Done");

  // The compass is connected using a protocol known as I2C which you will learn over the course of the year. The arduino wire library, 
  // handles most of that for us
  say(DEBUG_IMPORTANT,"Setting up I2C...");
  Wire.begin();
  say(DEBUG_IMPORTANT,"Done");

  // To enable the GPS we have to set the enable line to be high. This tells the GPS to start pumping information down the software 
  // serial gps_serial
  say(DEBUG_IMPORTANT,"Setting up GPS...");
  pinMode(GPS_ENABLE_PIN, OUTPUT); // This sets the pin that the enable line is connected to, to be a output pin
  digitalWrite(GPS_ENABLE_PIN,1); // This tells the arduino to set the enable line to be high (5v). So this wire is now live

  // Check to see if the GPS is pumping information out. We check the gps serial line for any data and store it in a buffer
  char buffer[15];
  for(int i = 0; i < 15; i++) {
  	if(gps_serial.available()) {
  		buffer[i] = gps_serial.read();
  	}
  }
  // Once we have enough data we print it out.
  for(int i = 0; i < 15; i++) {
  	Serial.print(buffer[i]);
  }
  Serial.println();
  delay(1000);

  say(DEBUG_IMPORTANT,"Done");
  say(DEBUG_IMPORTANT,"Setup Complete\n");
}

//////////////////////////////////////////////////////////////////////////////////////////
/// This function uses I2C to read the compass, this is currently not called as it hangs 
/// if the device isn't actually connected, which it isn't right now.
/// 
int readCompass() {
  byte buf[6];

  say(DEBUG_MINOR, "Set compass register");
  Wire.beginTransmission(HMC6343_ADDRESS); // Start communicating with the HMC6343 compasss
  Wire.write(HMC6343_HEADING_REG); // Send the address of the register that we want to read
  Wire.write(0x55); // Send the address of the register that we want to read
  Wire.endTransmission();

  say(DEBUG_MINOR, "Request 6 bytes");
  Wire.requestFrom(HMC6343_ADDRESS, 6); // Request six bytes of data from the HMC6343 compass, 
                                        // if you look at the data sheet you will see that the 
                                        // data we want is stored in a 6 byte register
  for(int i=0;i<6;i++)
  {
    while(Wire.available() < 1); // Busy wait while there is no byte to receive
    buf[i]=Wire.read();
    //printf("buf[%d]=%d\r\n",i,buf[i]);
  }

  // Now convert those siz bytes into a useful format using bit shifting. We have 3 values 
  // stored in 6 bytes, so 2 bytes per a value. If you don't understand whats quite going on
  // here thats perfectly fine. Hopefully you will by the end of the year :)
  int heading = ((buf[0] << 8) + buf[1]); // the heading in degrees
  int pitch =   ((buf[2] << 8) + buf[3]); // the pitch in degrees
  int roll = ((buf[4] << 8) + buf[5]); // the roll in degrees*/

  // Bring our values in the 0-359 range
  heading=heading/10;
  roll=roll/10;
  pitch=pitch/10;
  data.roll=(int8_t)roll;
  data.pitch=(int8_t)pitch;
  data.heading=(uint16_t)heading;

  delay(100);

  return (int)heading; // Print the sensor readings to the serial port.
}

//////////////////////////////////////////////////////////////////////////////////////////
/// This function reads the gps data as I can imagine you guessed. The GPS constantly 
/// streams nmea strings at us and they look something like "$PSRF103,04,01,00,01*21". 
/// Luckily we have a library called TinyGPS which will parse that for us
void readGPS() {
  unsigned long fix_age=9999,time,date;

  say(DEBUG_MINOR,"About to read GPS");

  //make sure the GPS has a fix, this might cause a wait the first time, but it should 
  // be quick any subsequent time
  while(fix_age == TinyGPS::GPS_INVALID_AGE||fix_age>3000)
  {
    dprintf(DEBUG_MINOR,"NMEA string: ");
    unsigned long start = millis(); // This times us out, so if the wire comes lose we won't 
                                    // get stuck here forever
    while(millis()<start+2000)
    {
      // Here we just pass the data over to TinyGPS if we have any
      if(gps_serial.available())
      {
        int c = gps_serial.read();
        gps.encode(c);
        // Prints out the characters coming in
        if(DEBUG_THRESHOLD>=DEBUG_MINOR)
        {
          Serial.write(c);
        }
        // Each NMEA string ends in a new line character
        if(c=='\n')
        {
          break;
        }
      }
    }

    // Now we ask TinyGPS for the data and store it outselves
    gps.get_datetime(&date,&time,&fix_age);

    // Here we just debug it out
    dprintf(DEBUG_MINOR,"fix age = %ld\r\n",fix_age);
    if(fix_age == TinyGPS::GPS_INVALID_AGE)
    {
      dprintf(DEBUG_IMPORTANT,"Invalid fix, fix_age=%ld\r\n",fix_age);
      say(DEBUG_IMPORTANT,"No GPS fix");
    }
  }   

  // This is here because at WRSC we didn't have a working compass so we used the
  // GPS heading as our forward instead of the compass. Not ideal but can be done
  gps.get_datetime(&date,&time,&fix_age);
  gps.f_get_position(&data.lat,&data.lon,&fix_age);
  data.heading = gps.course();
   

  if(fix_age == TinyGPS::GPS_INVALID_AGE)
  {
    say(DEBUG_IMPORTANT,"Invalid fix");
  }

  else
  {
    // Updating the time and data if we habe a fix
    say(DEBUG_IMPORTANT,"Fix Valid");
    dprintf(DEBUG_IMPORTANT,"lat=%ld lon=%ld\r\n",(long)(data.lat*1000),(long)(data.lon*1000));

    int year;
    byte month,day,hour,min,sec;
    unsigned long age;
      
    gps.crack_datetime(&year,&month,&day,&hour,&min,&sec,NULL,&age);  
    setTime(hour,min,sec,day,month,year);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////
// Utility function that keeps angles betweeen 0 and 360
int mod(int value){
  int newValue;
  if(value < 0){
    newValue = value + 360;
  }
  else if(value >= 360){
    newValue = value - 360;
  }
  else{
    newValue = value;
  }
  return newValue;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Utility function that calculates difference between two headings taking wrap around 
// into account
int get_hdg_diff(int heading1,int heading2)
{
  int result;

  result = heading1-heading2;

  if(result<-180)
  {
    result = 360 + result;
    return result;
  } 

  if(result>180)
  {
    result = 0 - (360-result);
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////
/// The main arduino loop, it does everything!!!!!!!
void loop()
{
  unsigned long last_gps_read=0;
  unsigned long last_time=0,time_now=0;
  int wp_hdg=0;
  float wp_dist=0.0;
  int wp_num=0;

  float igain=0.01;
  float pgain=0.1;
  float running_err=0.0;
  int hdg_err=0;
  int relwind;
  
  long last_telemetry=0;

  #define TELEMETRY_INTERVAL 10
  #define TARGET_LOOP_INTERVAL 100 //number of milliseconds between loop intervals

  ///////////////////////////////////////////////////////////////////////////////////
  /// Here we setup waypint stuff, these are actually the coordinates of the place
  /// Dermot was actually meant to race in at WRSC in Galway last summer
  #define NUM_OF_WAYPOINTS 8

  float wp_lats[NUM_OF_WAYPOINTS]; 
  float wp_lons[NUM_OF_WAYPOINTS];

  // start
  wp_lats[0] = 53.257804;
  wp_lons[0] = -9.117945;

  wp_lats[1] = 53.257795;
  wp_lons[1] = -9.117450;

  wp_lats[2] = 53.257805;
  wp_lons[2] = -9.117440;

  wp_lats[3] = 53.257918;
  wp_lons[3] = -9.117673;

  wp_lats[4] = 53.257928;
  wp_lons[4] = -9.117683;

  wp_lats[5] = 53.257814;
  wp_lons[5] = -9.117955;

  wp_lats[6] = 53.257805;
  wp_lons[6] = -9.117440;

  // End / Home
  wp_lats[7]= 53.25851;
  wp_lons[7]= -9.11918;
  ///////////////////////////////////////////////////////////////////////////////////

  // This is actually the main loop area, I don't know why Colin did this, I don't recommend 
  // it personally, bit weird...
  Serial.println("Entering loop");
  while(1)
  {    
    // Ensures the loop takes a constant time, not really needed though. 
    time_now=millis();
    if(time_now-last_time>0&&time_now-last_time<TARGET_LOOP_INTERVAL)
    {
      delay(TARGET_LOOP_INTERVAL-(time_now-last_time));
    }
    last_time=millis();

    // Here we read the GPS, generally you don't want to do this as often as we are doing here. But oh well...
    say(DEBUG_MINOR,"Reading GPS");
    readGPS();

    ///////////////////////////////////////////////////////////////////////////////////
    // So now we are onto the waypoint logic
    
    // TinyGPS has some lovely functions for working out the heading to a GPS coordinate
    // and the distance to work. Lon and Lat can be a pain to work with over
    wp_hdg = (int) gps.course_to(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);
    wp_dist = gps.distance_between(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);

    // Move onto next waypoint if we are inside the waypoint's radius
    if(wp_dist<WP_THRESHOLD)
    {       
      wp_num++;

      if(wp_num==NUM_OF_WAYPOINTS) //reached last waypoint already, keep us here
      {
        wp_num--;          
      }
      else //reached new waypoint
      {
        // Work out the new heading and distance
        wp_hdg = (int) gps.course_to(data.lat, data.lon,wp_lats[wp_num],wp_lons[wp_num]);
        wp_dist = gps.distance_between(data.lat, data.lon, wp_lats[wp_num],wp_lons[wp_num]);
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////
    //rudder logic
    //
    // So this actually does some slightly advance math stuff known as PID which right
    // now I can't explain. Ask me and Dave in person if you want to know more but
    // essentially it means we have a smooth turn towards where we want to go which 
    // kinda accounts for errors (Ughhh maths...)
    
    hdg_err = get_hdg_diff(wp_hdg,data.heading);

    running_err = running_err + (float)hdg_err;

    // clip the running error to -4000..4000
    if(running_err>4000)
    	running_err=4000;
    else if(running_err<-4000)
    	running_err=-4000;

    // apply a geometric decay to the running error
    running_err = running_err * 0.9;   

    // apply PI control
    data.rudder = (int) round((pgain * (float)hdg_err) + (igain * running_err));
    if(data.rudder<-5)
    {
      data.rudder=-5;
    }
    else if(data.rudder>5)
    {
      data.rudder=5;
    }

    // This command actually rotates the rudder
    rudderServo.writeMicroseconds(1500+(data.rudder*100));
    ///////////////////////////////////////////////////////////////////////////////////

    // Here we are just debug printing all the data we have. We don't do this every
    // loop through which is what the if statement is for
    if(last_telemetry + (TELEMETRY_INTERVAL * 1000) < millis())
    {
      dprintf(DEBUG_CRITICAL,"time=%ld hdg=%d hdg_err=%d roll=%d pitch=%d rudder=%d wp_num=%d wp_hdg=%d wp_dist=%ld ",now(),data.heading,hdg_err,data.roll,data.pitch,data.rudder,wp_num,wp_hdg,(long)wp_dist);
      
      if(DEBUG_THRESHOLD>=DEBUG_CRITICAL)
      {
        Serial.print("lat=");
        Serial.print(data.lat,5);
        Serial.print(" lon=");
        Serial.print(data.lon,5);
        Serial.print(" wplat=");
        Serial.print(wp_lats[wp_num],5);
        Serial.print(" wplon=");
        Serial.print(wp_lons[wp_num],5);
        Serial.print(" running_err=");
        Serial.println(running_err);
      }

      last_telemetry=millis();
    }
  }
}