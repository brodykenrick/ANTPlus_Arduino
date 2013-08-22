//Copyright 2013 Brody Kenrick.

//Took code from:
//http://digitalhacksblog.blogspot.com.au/2012_10_01_archive.html
// as a starting point

#ifndef ANTPLus_h
#define ANTPLus_h

#include <Arduino.h>
#include <Stream.h>

//#define ANTPLUS_DEBUG //!< Prints various debug messages. Disable here or via using NDEBUG externally
//#define ANTPLUS_MSG_STR_DECODE //<! Stringiser for various codes for easier debugging

#if defined(NDEBUG)
#undef ANTPLUS_DEBUG
#endif

//These are from the ANT+ packages (under Apache license)
#include "antdefines.h"
#include "antmessage.h"



#define PACKETREADTIMEOUT  (100)
#define PACKETREADNEXTBYTETIMEOUT  (10) //<! If we get a byte in a read -- how long do we wait for the next byte before timing out...
#define MAXPACKETLEN        (80)

#define ANT_DEVICE_NUMBER_CHANNELS (8) //!< nRF24AP2 has an 8 channel version.



//TODO: Make this into a class
#define DATA_PAGE_HEART_RATE_0              (0x00)
#define DATA_PAGE_HEART_RATE_0ALT           (0x80)
#define DATA_PAGE_HEART_RATE_1              (0x01)
#define DATA_PAGE_HEART_RATE_1ALT           (0x81)
#define DATA_PAGE_HEART_RATE_2              (0x02)
#define DATA_PAGE_HEART_RATE_2ALT           (0x82)
#define DATA_PAGE_HEART_RATE_3              (0x03)
#define DATA_PAGE_HEART_RATE_3ALT           (0x83)
#define DATA_PAGE_HEART_RATE_4              (0x04)
#define DATA_PAGE_HEART_RATE_4ALT           (0x84)

#define DATA_PAGE_SPEED_DISTANCE_1              (0x01) 

#define DEVCE_TYPE_HRM     (120)
#define DEVCE_TYPE_CADENCE (121)
#define DEVCE_TYPE_SDM     (124)
#define DEVCE_TYPE_GPS     (  0)


//TODO: Tidy.... Perhaps add into the class as external functions (else just use normal print....)
extern void serial_print_byte_padded_hex(byte value);
extern void serial_print_int_padded_dec( int, byte, boolean final_carriage_return = false);

typedef struct ANT_Packet_struct
{
   byte sync;
   byte length;
   byte msg_id;
   byte data[];//Variable -- elements == length
   //byte checksum... This is data[length]. See ANT_PACKET_CHECKSUM
} ANT_Packet;

#define ANT_PACKET_CHECKSUM(/*Ant_Packet * */ packet) (packet->data[packet->length])

//TODO: Rename?
typedef struct ANT_Broadcast_struct
{
   byte channel_number;
   byte data[8];
} ANT_Broadcast;

typedef struct ANT_DataPage_struct
{
   byte data_page_number;
   byte sensor_specific_data[7];
} ANT_DataPage;


typedef struct ANT_HRMDataPage_struct
{
  byte data_page_number:7;
  byte page_change_toggle:1;
//TODO: Union below
   byte who_cares_1;
   byte who_cares_2;
   byte who_cares_3;
   byte who_cares_4;
   byte who_cares_5;
//TODO: Union above
   byte heart_beat_count;
   byte computed_heart_rate;

} ANT_HRMDataPage;

typedef struct ANT_SDMDataPage1_struct
{
  byte data_page_number;
  byte last_time_frac; //  1/200 of a second
  byte last_time_int;
  byte distance_int;
  byte distance_frac;  //  1/16 of metre
  byte inst_speed_int;
  byte inst_speed_frac;
  byte stride_count;
  byte update_latency;
} ANT_SDMDataPage1;




typedef struct ANT_Channel_struct
{
   int channel_number;
   int network_number;
   int timeout;
   int device_type;
   int freq;
   int period;
   unsigned char ant_net_key[8];
} ANT_Channel;
 



typedef enum
{
  MESSAGE_READ_NONE, //No message available (immediately or after timeout period)
  MESSAGE_READ_ERROR_BAD_CHECKSUM,
  MESSAGE_READ_ERROR_MISSING_SYNC,
  MESSAGE_READ_ERROR_PACKET_SIZE_EXCEEDED,
  MESSAGE_ERROR_TIMEOUT_MIDMESSAGE,
  MESSAGE_READ_INTERNAL, //This is remapped to one of the next two in the internal read function
  MESSAGE_READ_OTHER,
  MESSAGE_READ_EXPECTED

} MESSAGE_READ;


//TODO: tidy this up....
typedef enum
{
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  ANT_CHANNEL_ESTABLISH_COMPLETE,
  ANT_CHANNEL_ESTABLISH_ERROR,

}   ANT_CHANNEL_ESTABLISH;


//TODO: Look at ANT and ANT+ and work out the appropriate breakdown and have a subclass
class ANTPlus
{
  public:
    ANTPlus(
        int RTS_PIN,
        int SUSPEND_PIN,
        int SLEEP_PIN,
        int RESET_PIN
    );

    void     begin(Stream &serial);
    void     hardwareReset( );

    boolean send(unsigned msgId, unsigned msgId_ResponseExpected, unsigned char argCnt, ...);
    MESSAGE_READ readPacket( ANT_Packet * packet, int packetSize, int wait_timeout );
    
    void         printPacket(const ANT_Packet * packet, boolean final_carriage_return);

    void sleep( boolean activate_sleep=true );
    void suspend(boolean activate_suspend=true );
    
    //Callback from the main code
    void   rTSHighAssertion();

    boolean awaitingResponseLastSent() {return (msgResponseExpected != MESG_INVALID_ID);};

    //ANT+ to setup a channel
    ANT_CHANNEL_ESTABLISH progress_setup_channel( const ANT_Channel * const channel );

#if defined(ANTPLUS_MSG_STR_DECODE)
    static const char * get_msg_id_str(byte msg_id);
#endif /*defined(ANTPLUS_MSG_STR_DECODE)*/

  private:
    MESSAGE_READ      readPacketInternal( ANT_Packet * packet, int packetSize, unsigned int readTimeout);
    unsigned char     writeByte(unsigned char out, unsigned char chksum);

  private:
    Stream* mySerial; //!< Serial -- Software serial or Hardware serial

    long rx_packet_count;
    long tx_packet_count;
    
    unsigned msgResponseExpected; //TODO: This should be an enum.....
    
    volatile boolean clear_to_send;

    int RTS_PIN;
    int SUSPEND_PIN;
    int SLEEP_PIN;
    int RESET_PIN;

};

#endif //ANTPLus_h

