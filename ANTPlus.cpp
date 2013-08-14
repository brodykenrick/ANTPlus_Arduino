
#include "ANTPlus.h"

#define DEBUG 1

ANTPlus::ANTPlus(
        int RTS_PIN,
        int SUSPEND_PIN,
        int SLEEP_PIN,
        int RESET_PIN
)
{
this->RTS_PIN = RTS_PIN;
this->SUSPEND_PIN = SUSPEND_PIN;
this->SLEEP_PIN = SLEEP_PIN;
this->RESET_PIN = RESET_PIN;

}


void ANTPlus::begin(Stream &serial)
{
  mySerial = &serial;

  pinMode(SUSPEND_PIN, OUTPUT);
  pinMode(SLEEP_PIN,   OUTPUT);
  pinMode(RESET_PIN,   OUTPUT);
  pinMode(RTS_PIN,     INPUT);
  
  //Per datasheet
  digitalWrite(RESET_PIN,   HIGH);
  digitalWrite(SUSPEND_PIN, HIGH);
  digitalWrite(SLEEP_PIN,   LOW);
  
  //Interrupts?
  
  clear_to_send = false;
  msgResponseExpected = MESG_INVALID_ID;
}


void ANTPlus::hardwareReset()
{
  Serial.println("H/w Reset");  
  digitalWrite(RESET_PIN,   LOW);
  delay(10);
  digitalWrite(RESET_PIN,   HIGH);
  
  clear_to_send = false;
  msgResponseExpected = MESG_STARTUP_MESG_ID;
}

int rxBufCnt = 0;
unsigned char rxBuf[MAXPACKETLEN];


// Data <sync> <len> <msg id> <channel> <msg id being responded to> <msg code> <chksum>
// <sync> always 0xa4
// <msg id> always 0x40 denoting a channel response / event
// <msg code? success is 0.  See page 84 of ANT MPaU for other codes
//readTimeoutMs -- is amount of time to wait for first byte to appear
MESSAGE_READ ANTPlus::readPacketInternal( ANT_Packet * packet, int packetSize, unsigned int readTimeoutMs)
{
  unsigned char byteIn;
  unsigned char chksum = 0;
   
  unsigned long timeoutExit = millis() + readTimeoutMs;
  
//  Serial.println("readPacket");
 
  while (timeoutExit >= millis()) //First loop will go through (TODO: Swap to a do while for absolute efficiency)
  {
    //This is a busy read
    if (mySerial->available() > 0)
    {
      byteIn = mySerial->read();
      //serial_print_byte_padded_hex( chksum );
      //We have a byte -- so we want to finish off this message (increase timeout)
      timeoutExit += PACKETREADNEXTBYTETIMEOUT;
      if ((byteIn == MESG_TX_SYNC) && (rxBufCnt == 0))
      {
        rxBuf[rxBufCnt++] = byteIn;
        chksum = byteIn;
      }
      else if ((rxBufCnt == 0) && (byteIn != MESG_TX_SYNC))
      {
        return MESSAGE_READ_ERROR_MISSING_SYNC;
      }
      else if (rxBufCnt == 1)
      {
        rxBuf[rxBufCnt++] = byteIn;       // second byte will be size
        chksum ^= byteIn;
      }
      else if (rxBufCnt < rxBuf[1]+3)
      { // read rest of data taking into account sync, size, and checksum that are each 1 byte
        rxBuf[rxBufCnt++] = byteIn;
        chksum ^= byteIn;
      }
      else
      {
        rxBuf[rxBufCnt++] = byteIn;
        if (rxBufCnt > packetSize)
        {
          return MESSAGE_READ_ERROR_PACKET_SIZE_EXCEEDED;
        }
        else
        {
//          Serial.println("gotpacket");
          memcpy(packet, &rxBuf, rxBufCnt); // should be a complete packet. copy data to packet variable, check checksum and return
          rx_packet_count++;
          //serial_print_byte_padded_hex( chksum );
          //serial_print_byte_padded_hex( ANT_PACKET_CHECKSUM(packet) );
          
          if (chksum != ANT_PACKET_CHECKSUM(packet))
          {
            rxBufCnt = 0;
            return MESSAGE_READ_ERROR_BAD_CHECKSUM;
          }
          else
          {
            //Good packet
            rxBufCnt = 0;
            return MESSAGE_READ_INTERNAL;
          }
        }
      }
    }
  }
  
  if(rxBufCnt != 0)
  {
    return MESSAGE_ERROR_TIMEOUT_MIDMESSAGE;
  }
  return MESSAGE_READ_NONE;
}


unsigned char ANTPlus::writeByte(unsigned char out, unsigned char chksum) {
#ifdef DEBUG
  serial_print_byte_padded_hex(out);
  Serial.print(" ");
#endif
  mySerial->write(out);
  chksum ^= out;
  return chksum;
}

//TODO: DEBUG: Convert to a packet object for quicker/easier printing....
//TODO: Extend the return types
boolean ANTPlus::send(unsigned msgId, unsigned msgId_ResponseExpected, unsigned char argCnt, ...)
{
  va_list arg;
  va_start (arg, argCnt);
  unsigned char byteOut;
  unsigned char chksum = 0;
  int cnt = 0;
  
  boolean ret_val = false;

  if(clear_to_send && (msgResponseExpected == MESG_INVALID_ID))
  {
    #ifdef DEBUG
      Serial.print("TX[");
      serial_print_int_padded_dec( tx_packet_count, 6 );
      Serial.print("] @ ");
      serial_print_int_padded_dec( millis(), 8 );
      Serial.print(" ms > ");
      Serial.print( get_msg_id_str(msgId) );
      Serial.print("[0x");
      serial_print_byte_padded_hex(msgId);
      Serial.print("]");
      Serial.print(" - 0x");
    #endif
      tx_packet_count++;
     
      chksum = writeByte(MESG_TX_SYNC, chksum); // send sync
      chksum = writeByte(argCnt, chksum);       // send length
      chksum = writeByte(msgId, chksum);        // send message id
       
      // send data
      //TODO: Is this corect, or offset by 2 now?
      for (cnt=1; cnt <= argCnt; cnt++)
      {
        byteOut = va_arg(arg, unsigned int);
        chksum = writeByte(byteOut, chksum);
      }
      va_end(arg);
       
      writeByte(chksum,chksum);                 // send checksum 
      
      clear_to_send = false;
      ret_val = true;
      
      //We are now waiting for this message
      //There are other functions that take care of the checks
      //and eventually will have timeouts... and possibly callbacks...
      msgResponseExpected = msgId_ResponseExpected;
    }
    else
    {
      Serial.println("Can't send -- not clear to send or awaiting a response");
      ret_val = false;
    }
#ifdef DEBUG
  Serial.println();
#endif
    return ret_val;
}



MESSAGE_READ ANTPlus::readPacket( ANT_Packet * packet, int packetSize, int wait_timeout = 0 )
{
    MESSAGE_READ ret_val = MESSAGE_READ_NONE;
    {
        ret_val = readPacketInternal(packet, packetSize, wait_timeout);
        if (ret_val == MESSAGE_READ_INTERNAL)
        {
            //printPacket( packet, false );

            if( packet->msg_id == msgResponseExpected )
            {
                //Serial.println("Received expected message!");
                msgResponseExpected = MESG_INVALID_ID;
                ret_val = MESSAGE_READ_EXPECTED;
            }
            else
            {
                //Serial.println("Received unexpected message!");
                ret_val = MESSAGE_READ_OTHER;
            }
        }
    }
    return ret_val; 
}





//TODO: Move these to progmem
#ifdef DEBUG
//Static
const char * ANTPlus::get_msg_id_str(byte msg_id)
{
  switch (msg_id)
  {
      case MESG_RESPONSE_EVENT_ID:
        return "MESG_RESPONSE_EVENT_ID";
      case MESG_CAPABILITIES_ID:
        return "MESG_CAPABILITIES_ID";
      case MESG_BROADCAST_DATA_ID:
        return "MESG_BROADCAST_DATA_ID";
      case MESG_ASSIGN_CHANNEL_ID:
        return "MESG_ASSIGN_CHANNEL_ID";
      case MESG_CHANNEL_MESG_PERIOD_ID:
        return "MESG_CHANNEL_MESG_PERIOD_ID";
      case MESG_CHANNEL_SEARCH_TIMEOUT_ID:
        return "MESG_CHANNEL_SEARCH_TIMEOUT_ID";
      case MESG_CHANNEL_RADIO_FREQ_ID:
        return "MESG_CHANNEL_RADIO_FREQ_ID";


      case MESG_REQUEST_ID:
        return "MESG_REQUEST_ID";

      case MESG_STARTUP_MESG_ID:
        return "MESG_STARTUP_MESG_ID";



      case MESG_NETWORK_KEY_ID:
        return "MESG_NETWORK_KEY_ID";
      case MESG_SYSTEM_RESET_ID:
        return "MESG_SYSTEM_RESET_ID";
      case MESG_OPEN_CHANNEL_ID:
        return "MESG_OPEN_CHANNEL_ID";
      case MESG_CHANNEL_ID_ID:
        return "MESG_CHANNEL_ID_ID";

      default:
        return "...";

    } 
}
#endif



void ANTPlus::printPacket(const ANT_Packet * packet, boolean final_carriage_return = true)
{
  Serial.print("RX[");
  serial_print_int_padded_dec( rx_packet_count, 6 );
  Serial.print("] @ ");
  serial_print_int_padded_dec( millis(), 8 );
  Serial.print(" ms > ");
//  Serial.print("0x");
//  serial_print_byte_padded_hex(packet->sync);
//  Serial.print(" ");
  Serial.print( packet->length );
  Serial.print("B ");
  Serial.print( get_msg_id_str (packet->msg_id) );
  Serial.print("[0x");
  serial_print_byte_padded_hex(packet->msg_id);
  Serial.print("]");
  Serial.print(" : ");
  Serial.print("0x");
  int cnt = 0;
  while( cnt < ( packet->length ) )
  {
    serial_print_byte_padded_hex( packet->data[cnt] );
    Serial.print  (" ");
    cnt++;
  }
  if(final_carriage_return)
  {
    Serial.println("");
  }
  else
  {
     Serial.print(" ");
  }
}

