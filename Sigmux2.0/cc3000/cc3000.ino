#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"


#include <avr/io.h>
#include <avr/interrupt.h>

#undef UNO

#define	LOG_DATA

// These are the interrupt and control pins for the UNO
#ifdef UNO
  #define ADAFRUIT_CC3000_IRQ   3      // MUST be an interrupt pin!
  // These can be any two pins
  #define ADAFRUIT_CC3000_VBAT  5
  #define ADAFRUIT_CC3000_CS    10     //  http://forum.arduino.cc/index.php?topic=241369.0
// These are the interrupt and control pins for the ATmega32u4 (same as Leonardo)
#else
  #define ADAFRUIT_CC3000_IRQ   7      // MUST be an interrupt pin!
  // These can be any two pins
  #define ADAFRUIT_CC3000_VBAT  6
  #define ADAFRUIT_CC3000_CS    17     //  http://forum.arduino.cc/index.php?topic=241369.0
#endif


// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
					 ADAFRUIT_CC3000_IRQ,
				         ADAFRUIT_CC3000_VBAT,
					 SPI_CLOCK_DIVIDER);

// cannot be longer than 32 characters!
#define WLAN_SSID       "Team_26"        
#define WLAN_PASS       "rmcpazzword"
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define EDT_UDP_SERVICE	5002

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2

Adafruit_CC3000_Client udpClient;

#define SERIAL_CONVERT
#define MUX_TELEOP         0
#define MUX_AUTONOMOUS     1
#define MUX_LED            11    // MUX select pin
#define AUTONOMOUS_LED     4     // Autonomous mode LED
#define MANUAL_LED         12    // Manual mode LED
#define SAFE_LED           PE2
#define WLAN_LED           5     // WLAN LED
#define DHCP_LED           13

#define SAFE_MODE       0
#define AUTONOMOUS_MODE       1
#define MANUAL_MODE     2

#define SAFE_DELAY 1000      // delay in ms every time we switch to/from SAFE_MODE

#ifdef SERIAL_CONVERT
  #define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = ((_Data[j] << 1) ^ 0xff);}}
#else
  #define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = _Data[j];}}
#endif

// if we're running the UNO, we can't use USART so don't do anything
#ifdef UNO
  #define USART_Transmit(_Data, _Index) {}
#endif

static volatile uint8_t currentMode;
static volatile uint8_t g_newMode;
bool dhcp = false;
uint8_t packets = 0;
unsigned long timeLastReceivedPacket = millis();
#define PACKET_TIME_LIMIT 500  // time limit since last packet for LED blinking in millis

int lastActuatorVal = 0;
// constants multiplied by value from panel udp socket
#define LEFT_WHEEL_CONSTANT 100
#define RIGHT_WHEEL_CONSTANT 100
#define ACTUATOR_CONSTANT 100
#define DIG_CONSTANT 500

    // Emergency stop command (!EX) to all motor controllers
    const unsigned char KILL_COMMAND[] = {'@', '0', '1', '!', 'E', 'X',
        '_', '@', '0', '2', '!', 'E', 'X',
        '_', '@', '0', '3', '!', 'E', 'X',
        '_', '@', '0', '4', '!', 'E', 'X',
        '_', '@', '0', '5', '!', 'E', 'X', '\r'};  // length 35 chars
    // Resume command to re-activate and come out of emergency stop
    const unsigned char RESUME_COMMAND[] = {'@', '0', '1', '!', 'M', 'G',
        '_', '@', '0', '2', '!', 'M', 'G',
        '_', '@', '0', '3', '!', 'M', 'G',
        '_', '@', '0', '4', '!', 'M', 'G',
        '_', '@', '0', '5', '!', 'M', 'G', '\r'};  // length 35 chars
    // current command being sent to roboteq motor controller
    // example "@01!G 1 1000_@01!G 2 -1000_@02!G 1 1000_@02!G 2 -1000_@03!G 1 1000_@03!G 2 -1000_@04!G 1 700_@04!G 2 700_@05!G 1 500\r"
    // max length: 14 bytes for each channel for each roboteq (4 of them have 2 channels, 1 has one channel, total 9 channels), so 14*9 = 126 bytes, add one for safety
//    unsigned char roboteqCommand = "@01!G 1 1000_@01!G 2 -1000_@02!G 1 1000_@02!G 2 -1000_@03!G 1 1000_@03!G 2 -1000_@04!G 1 700_@04!G 2 700_@05!G 1 500\r";
    
void setup()
{
        pinMode(SAFE_LED, OUTPUT);
        pinMode(DHCP_LED, OUTPUT);
        pinMode(MANUAL_LED, OUTPUT);
        pinMode(AUTONOMOUS_LED, OUTPUT);
        pinMode(WLAN_LED, OUTPUT);
//        digitalWrite(SAFE_LED, HIGH);
        DDRE |= (1 << DDE2);		// DDRE2 set outbound for Safe Mode LED
//        PORTE |= (1 << PORTE2);					// Safe Mode LED on
        
        
        // initialize timer for dhcp_led timer (simulates Rx)
        // initialize Timer1
        cli();          // disable global interrupts
        TCCR1A = 0;     // set entire TCCR1A register to 0
        TCCR1B = 0;     // same for TCCR1B
     
        // set compare match register to desired timer count:
        OCR1A = 15624 / 10;  // 100 ms
        // turn on CTC mode:
        TCCR1B |= (1 << WGM12);
        // Set CS10 and CS12 bits for 1024 prescaler:
        TCCR1B |= (1 << CS10);
        TCCR1B |= (1 << CS12);
        // enable timer compare interrupt:
        TIMSK1 |= (1 << OCIE1A);
        sei();          // enable global interrupts
        
        // first set it into safe mode
        currentMode = MANUAL_MODE;  // set this variable to not SAFE just until we call ModeSet()
        ModeSet(SAFE_MODE);
        currentMode = SAFE_MODE;
        
#ifdef UNO        
	Serial.begin(115200);
#endif
    // Check that cc3000.begin() returns true
	while (!cc3000.begin())
	{
#ifdef UNO
		Serial.println(F("Unable to initialize the CC3000! Check your wiring?"));
#endif
		delay(500);
	}
	
    // Disable TCP timeout
	disableIdleTimout();
	
    // Check Firmware version
	uint16_t firmware = checkFirmwareVersion();
	if (firmware < 0x113)
	{
#ifdef UNO
		Serial.println(F("Wrong firmware version!"));
#endif
		while(1);
	}
  
    // Print out some debugging info
    displayDriverMode();
	displayMACAddress();
  
    // Attempt connection to AP

	// NOTE: Secure connections are not available in 'Tiny' mode!
	// By default connectToAP will retry indefinitely, however you can pass an
	//  optional maximum number of retries (greater than zero) as the fourth parameter.
     
	// ALSO NOTE: By default connectToAP will retry forever until it can connect to
	// the access point.  This means if the access point doesn't exist the call
	// will _never_ return!  You can however put in an optional maximum retry count
	// by passing a 4th parameter to the connectToAP function below.  This should
	// be a number of retries to make before giving up, for example 5 would retry
	// 5 times and then fail if a connection couldn't be made.
#ifdef UNO
	Serial.print(F("Connecting to AP - "));
#endif
        //wlan_disconnect();
	
    if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY))
	{
		Reboot("Connecting to AP - Failed", 1);
		return;
	}
    digitalWrite(WLAN_LED, HIGH);

#ifdef UNO   
	Serial.println(F("OK"));
#endif  
    // Check for DHCP and timeout after 20 seconds
    unsigned long dhcpTimeout = 20000;	// Try for 20 sec
	unsigned long retry       = 0;
   
	for(unsigned long t = millis(); ((millis() - t) <= dhcpTimeout);)
	{
#ifdef UNO
		Serial.print(F("Querying DHCP - "));
#endif
		if(cc3000.checkDHCP())
		{
#ifdef UNO
			Serial.println(F("OK"));
#endif
                        digitalWrite(DHCP_LED, HIGH);
                        dhcp = true;
			break;
		}
		else
		{
			if(retry >= 10)
				Reboot("FAILED - Rebooting!!!", 2);
#ifdef UNO
			Serial.println(retry+1);
#endif
                        dhcp = false;
		}
		
		++retry;
		delay(1000);
	}
	
    // Attempt connection to UDP
	uint32_t bindIP = displayConnectionDetails();
	uint32_t ip = cc3000.IP2U32(192, 168, 1, 192);	// EDT Panel IP
	udpClient   = cc3000.connectUDP(ip, EDT_UDP_SERVICE, 0);
#ifdef UNO
	if( udpClient.connected() )
          Serial.println(F("UDP Kurwa Connected"));
    else
		Serial.println(F("UDP Kurwa is dead"));
#endif
}

void loop()
{
		
		if( udpClient.available() )
		{
  
            // parse command
#ifdef UNO
            Serial.println("attempting to read from udp socket");
#endif
            byte command[2];
            udpClient.read(&command, 2);

            uint8_t actuator    = 0;
            uint8_t dig         = 0;
            int8_t left        = 0;
            int8_t right       = 0;
            
            uint8_t mode;
            parseCommand(command, &actuator, &dig, &mode, &left, &right);
            g_newMode = mode;  // workaround because currentMode is volatile
            
            // for debugging
#ifdef UNO
            Serial.print("command bytes received: "); 
            cc3000.printHexChar(command, 2);
            Serial.print(command[0], BIN); Serial.print(command[1], BIN); Serial.println();
            Serial.print("actuator: "); Serial.println(actuator);
            Serial.print("dig: "); Serial.println(dig);
            Serial.print("mode: "); Serial.println(currentMode);
            Serial.print("left: "); Serial.println(left);
            Serial.print("right: "); Serial.println(right);
#endif
               String canCommand; // Start string off with "@0" 
               int leftMotorVal = left * LEFT_WHEEL_CONSTANT;
               int rightMotorVal = right * RIGHT_WHEEL_CONSTANT;
               int actuatorVal = actuator * ACTUATOR_CONSTANT;
               int digVal = dig * DIG_CONSTANT;
              
              
              // append motor value to canCommand if not zero
              if ( leftMotorVal != 0 || rightMotorVal != 0)
              { 
                for (int i = 1; i <= 3; i++)
                {
                    if ( i != 1)
                    {
                       canCommand += "_";
                    }
                    canCommand += "@0";
                    canCommand += i;
                    canCommand += "!G 1 ";
                    
                    canCommand += leftMotorVal;
                    canCommand += "_@0";
                    canCommand += i;
                    canCommand += "!G 2 ";
                    canCommand += rightMotorVal;
                   
                }
                  
              }
              
              // append actuatorVal to canCommand if last one has changed (no watchdog for actuators)
              if( actuatorVal != lastActuatorVal)
              {
                 canCommand += "@04!G 1 ";
                 canCommand += actuatorVal;
                 canCommand += "_@04!G 2 ";
                 canCommand += actuatorVal; 
              }
              lastActuatorVal = actuatorVal;  // update last actuator value
              
              // append digVal to canCommand
              canCommand += "@05!G 1 ";
              canCommand += digVal;
              canCommand += "\r";

            
            // switch modes, check if we fail
            if(!ModeSet(g_newMode))
            {
#ifdef UNO
                // TODO PRINT OUT ERROR
                Serial.print("Cannot change mode to "); Serial.println(currentMode);
#endif
            } else
            {
                currentMode = g_newMode;
            }

#ifdef UNO
            Serial.print("CAN command is: ");
            Serial.println(canCommand); //Send TX here 
           
            // TODO if manual mode,
            // construct RoboteQ commands and transmit over USART
            if(currentMode == MANUAL_MODE)
            {
                Serial.println("Transmitting CAN command...");
                USART_Transmit(canCommand, canCommand.length());
            } else {
                Serial.print("Not transmitting CAN command because we are in mode: ");
                Serial.println(currentMode);
            }
#endif            
            packets++;  // increment number of packets
            timeLastReceivedPacket = millis();
        }
}

// timer interrupt callback (toggle DHCP_LED if there's activity on UDP socket)
// otherwise, put DHCP_LED back the way it was
ISR(TIMER1_COMPA_vect)
{
    if(millis() - timeLastReceivedPacket < PACKET_TIME_LIMIT)
    {
        digitalWrite(DHCP_LED, !digitalRead(DHCP_LED));
    } else
    {
      digitalWrite(DHCP_LED, dhcp);
    }
}

uint16_t checkFirmwareVersion(void)
{
	uint8_t	 major, minor;
	uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
	if(!cc3000.getFirmwareVersion(&major, &minor))
		Reboot("Unable to retrieve the firmware version", 3);
	else
	{
		Serial.print(F("Firmware : "));
		Serial.print(major);
		Serial.print(F("."));
		Serial.println(minor);
		
		version = major; version <<= 8; version |= minor;
	}
#endif
	return version;
}

uint32_t displayConnectionDetails(void)
{
	uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
	if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
		Reboot("Failed to query IP", 4);
	else
	{
#ifdef UNO
		Serial.print(F(  " IP  : ")); cc3000.printIPdotsRev(ipAddress);
		Serial.print(F("\n Mask: ")); cc3000.printIPdotsRev(netmask);
		Serial.print(F("\n GW  : ")); cc3000.printIPdotsRev(gateway);
		Serial.print(F("\n DHCP: ")); cc3000.printIPdotsRev(dhcpserv);
		Serial.print(F("\n DNS : ")); cc3000.printIPdotsRev(dnsserv);
		Serial.println();
#endif
		return ipAddress;
	}
}

void Reboot(const char* errMsg, uint32_t errCode)
{
	Serial.println(errMsg);
	Serial.println(F("--REBOOTING--"));
	while(true);
	//cc3000.reboot();
        for(int x = 0; x < errCode; x++);
        {
          digitalWrite(WLAN_LED, HIGH);
          delay(300);
          digitalWrite(WLAN_LED, LOW);
          delay(300);
        }
}

void displayDriverMode(void)
{
#ifdef UNO
#ifdef CC3000_TINY_DRIVER
	Serial.println(F("CC3000 is configured in 'Tiny' mode"));
#else
	Serial.print(F("RX Buffer : "));
	Serial.print(CC3000_RX_BUFFER_SIZE);
	Serial.println(F(" bytes"));
	Serial.print(F("TX Buffer : "));
	Serial.print(CC3000_TX_BUFFER_SIZE);
	Serial.println(F(" bytes"));
#endif
#endif
}

void displayMACAddress(void)
{
	uint8_t macAddress[6];
#ifdef UNO
	if(!cc3000.getMacAddress(macAddress))
	{
		Serial.println(F("Unable to retrieve MAC Address!"));
	}
	else
	{
		Serial.print(F("MAC Address : "));
		cc3000.printHex((byte*)&macAddress, 6);
	}
#endif
}

void disableIdleTimout() {
    // Per http://e2e.ti.com/support/low_power_rf/f/851/t/292664.aspx
    // aucInactivity needs to be set to 0 (never timeout) or the socket will close after
    // 60 seconds of no activity
    unsigned long aucDHCP       = 14400;
    unsigned long aucARP        = 3600;
    unsigned long aucKeepalive  = 30;
    unsigned long aucInactivity = 0;
#ifdef UNO
    if(aucInactivity == 0)
        Serial.println(F("Setting netapp to not timeout"));
    else
    {
        Serial.print(F("Setting netapp to timeout in "));
        Serial.print(aucInactivity);
        Serial.println(F(" Seconds"));
    }
#endif
    long iRet = netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity);
    
    if (iRet != 0)
    {
#ifdef UNO
        Serial.print(F("Could not set netapp option, iRet = "));
        Serial.println(iRet);
        Serial.println(F(", aborting..."));
#endif
        while(1);
    }
}

/* parses commands from CC3000 to control motors
 *
 * Commands come as two bytes (big-endian):
 * _____________________________________       __________________________________________
 * | x | x | x | A1 | A2 | D | M1 | M2 |       | LW | LW | LW | LW | RW  | RW | RW | RW |
 * -------------------------------------       ------------------------------------------
 *                
 *  3 empty, 2 actuator, 1 dig, 2 mode bits... 1 left sign bit, 3 left bits, 1 right sign bit, 3 right bits
 *
 * We have to convert them to little-endian. Note that left and right values are signed (2's complement).
 *
 */
void parseCommand(byte* comm, uint8_t* actuator, uint8_t* dig, uint8_t* mode, int8_t* left, int8_t* right)
{
    // convert command bytes to little-endian
    uint16_t command = (uint16_t)( (comm[0] << 8) | (comm[1]) );
    
    uint16_t ACTUATOR_MASK = 0x1800 ; // 0b 00011000 00000000
    uint8_t ACTUATOR_OFFSET = 11;
    *actuator = (command & ACTUATOR_MASK) >> ACTUATOR_OFFSET;

    uint16_t DIG_MASK = 0x0400 ; // 0b 00000100 00000000
    uint8_t DIG_OFFSET = 10;
    *dig = (command & DIG_MASK) >> DIG_OFFSET;

    uint16_t MODE_MASK = 0x0300 ; // 0b 00000011 00000000
    uint8_t MODE_OFFSET = 8;
    uint8_t temp = (command & MODE_MASK) >> MODE_OFFSET;
    if(temp == SAFE_MODE)
        *mode = SAFE_MODE;
    else if(temp == AUTONOMOUS_MODE)
        *mode = AUTONOMOUS_MODE;
    else if(temp == MANUAL_MODE)
        *mode = MANUAL_MODE;
    // else
        // TODO ERROR

    // note: left and right values are signed (2's complement)
    uint16_t LEFT_MASK = 0x00F0 ; // 0b 00000000 11110000
    uint8_t LEFT_OFFSET = 4;
    *left = (int8_t)((command & LEFT_MASK)) / 16;


    uint16_t RIGHT_MASK = 0x000F ; // 0b 00000000 00001111
    *right = ((int8_t)((command & RIGHT_MASK) << 4) ) / 16;

}

void MuxSelect(uint8_t selection)
{
  switch (selection)
  {
    case MUX_TELEOP:
        digitalWrite(MUX_LED, LOW);     //Set MUX Select low to allow Atmega TX line to go through MUX
        break;
    case MUX_AUTONOMOUS:
        digitalWrite(MUX_LED, HIGH);    // Set MUX Select high to allow FTDI to go through MUX
        break;
  }
}

// TODO fix constant delays here
uint8_t ModeSet(uint8_t newMode)
{

    // don't do anything if we're in same mode
    if (newMode != currentMode)
    {
        switch (newMode)
        {
            case SAFE_MODE:
                USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);	
                currentMode = SAFE_MODE;
//                digitalWrite(SAFE_LED, HIGH);
		PORTE |= (1 << PORTE2);					// Safe Mode LED on
                digitalWrite(AUTONOMOUS_LED, LOW);
                digitalWrite(MANUAL_LED, LOW);
                delay(500);
                break;

            case AUTONOMOUS_MODE:            
                // Listen to all communications by the computer.
                // Do not forward any CC3000 motor controller commands.
                USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_AUTONOMOUS);
                USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(AUTONOMOUS_LED, HIGH);    	        // Autonomous Mode LED on
		PORTE &= ~(1 << PORTE2);					// Safe Mode LED off
                digitalWrite(MANUAL_LED, LOW);
                delay(500);  // TODO why do we need this delay? we don't want this to let the udp buffer fill up?
                currentMode = AUTONOMOUS_MODE;
                break;

            case MANUAL_MODE:
                USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);
                USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(MANUAL_LED, HIGH);		// Manual Mode LED on
                digitalWrite(AUTONOMOUS_LED, LOW);    	        // Autonomous Mode LED off
		PORTE &= ~(1 << PORTE2);					// Safe Mode LED off
                delay(500);
                currentMode = MANUAL_MODE;
                break;

            default:
                return -1;
                break;
        }

        return currentMode; // After setting the mode, this should now be the current mode we return
    }
    else
        return currentMode; // The current mode is not different from the new requested mode
}
