//Copyright 2013 Brody Kenrick.
//An Ant+ library over UART ('Serial' or SoftwareSerial)

#define __ASSERT_USE_STDERR
#include <assert.h>

#include "ANTPlus.h"


#if defined(ANTPLUS_DEBUG)
#define ANTPLUS_DEBUG_PRINT(x)  	        (Serial.print(x))
#define ANTPLUS_DEBUG_PRINTLN(x)	        (Serial.println(x))
#else
#define ANTPLUS_DEBUG_PRINT(x)  	        
#define ANTPLUS_DEBUG_PRINTLN(x)	        
//NOTE: The printPacket function still calls Serial directly. TODO: Adjust that.
#endif

ANTPlus::ANTPlus(
        byte RTS_PIN,
        byte SUSPEND_PIN,
        byte SLEEP_PIN,
        byte RESET_PIN
)
{
    this->RTS_PIN = RTS_PIN;
    this->SUSPEND_PIN = SUSPEND_PIN;
    this->SLEEP_PIN = SLEEP_PIN;
    this->RESET_PIN = RESET_PIN;
    
    hw_reset_count = 0;
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
  
  //Interrupts are set in the main program currently
  //TODO: Look to see if they should be brought 'in lib'
  
  
  //This should not be strictly necessary - the device should always come up by itself....
  //But let's make sure we didn't miss the first RTS in a power-up race
  hardwareReset();
}


void ANTPlus::hardwareReset()
{
  ANTPLUS_DEBUG_PRINTLN("H/w Reset");
  
  sleep(false);
  digitalWrite(RESET_PIN,   LOW);
  delay(5);
  //Reset all variables before we release the ANT
  clear_to_send = false;
  msgResponseExpected = MESG_START_UP;
  rxBufCnt = 0;
  rx_packet_count = 0;
  tx_packet_count = 0;
  hw_reset_count++;
  delay(5);
  digitalWrite(RESET_PIN,   HIGH);
}

// Data <sync> <len> <msg id> <channel> <msg id being responded to> <msg code> <chksum>
// <sync> always 0xa4
// <msg id> 0x40==MESG_RESPONSE_EVENT_ID denoting a channel response / event
// <msg id> 0x4E==MESG_BROADCAST_DATA_ID denoting a broadcast (e.g. HRM or SDM)
// <msg code> success is 0.  See page 84 of ANT MPaU for other codes
//readTimeoutMs -- is amount of time to wait for first byte to appeaer (can be 0)
MESSAGE_READ ANTPlus::readPacketInternal( ANT_Packet * packet, int packetSize, unsigned int readTimeoutMs)
{
  unsigned char byteIn;
  unsigned char chksum = 0;
  unsigned long timeoutExit = millis() + readTimeoutMs;
  
  while (timeoutExit >= millis()) //First loop will go through always
  {
    //This is a busy read
    if (mySerial->available() > 0)
    {
      byteIn = mySerial->read();
      //We have a byte -- so we want to finish off this message (increase timeout)
      timeoutExit += ANT_PACKET_READ_NEXT_BYTE_TIMEOUT_MS;
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
          //Likely we are missing something....
          //we reset our buffer count
          rxBufCnt = 0;
          return MESSAGE_READ_ERROR_PACKET_SIZE_EXCEEDED;
        }
        else
        {
          memcpy(packet, &rxBuf, rxBufCnt); // Should be a complete packet. copy data to packet variable, check checksum and return
          rx_packet_count++;
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
    //This may be recoverable but it is likely not worth the effort
    //we reset our buffer count
    rxBufCnt = 0;
  
    return MESSAGE_READ_INFO_TIMEOUT_MIDMESSAGE;
  }
  return MESSAGE_READ_NONE;
}

//! Write out a single byte and return the updated checksum
unsigned char ANTPlus::writeByte(unsigned char out, unsigned char chksum)
{
#ifdef ANTPLUS_DEBUG
  serial_print_byte_padded_hex(out);
  Serial.print(" ");
#endif
  mySerial->write(out);
  chksum ^= out;
  return chksum;
}

//TODO: DEBUG: Convert (or add function) for a packet struct for quicker/easier printing....
//TODO: Extend the return types
// msgId_ResponseExpected if set to another ID than MESG_INVALID_ID will not allow a subsequent send until that message is received.
// NOTE: This request/response check still has the potentioal for holes in it but it is sufficient for now
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
    #ifdef ANTPLUS_DEBUG
      Serial.print("TX[");
      serial_print_int_padded_dec( tx_packet_count, 6 );
      Serial.print("] @ ");
      serial_print_int_padded_dec( millis(), 8 );
      Serial.print(" ms > ");
    #if defined(ANTPLUS_MSG_STR_DECODE)
      Serial.print( get_msg_id_str(msgId) );
      Serial.print("[0x");
      serial_print_byte_padded_hex(msgId);
      Serial.print("]");
    #else
      Serial.print("0x");
      serial_print_byte_padded_hex(msgId);
    #endif //defined(ANTPLUS_MSG_STR_DECODE)
      Serial.print(" - 0x");
    #endif
      tx_packet_count++;
     
      chksum = writeByte(MESG_TX_SYNC, chksum); // send sync
      chksum = writeByte(argCnt, chksum);       // send length
      chksum = writeByte(msgId, chksum);        // send message id
       
      // Send data
      for (cnt=1; cnt <= argCnt; cnt++)
      {
        byteOut = va_arg(arg, unsigned int);
        chksum = writeByte(byteOut, chksum);
      }
      va_end(arg);
       
      writeByte(chksum,chksum);                 // send checksum 
      
      clear_to_send = false;
      ret_val = true;
      
      //We are now waiting for this message (if it was not set as INVALID)
      //There are other functions that take care of the checks
      //and eventually will have timeouts... and possibly callbacks...
      msgResponseExpected = msgId_ResponseExpected;
#ifdef ANTPLUS_DEBUG
      Serial.println();
#endif
    }
    else
    {
      //ANTPLUS_DEBUG_PRINTLN("Can't send -- not clear to send or awaiting a response");
      ret_val = false;
    }

    return ret_val;
}


//! Read a packet into ANT_Packet struct
//readTimeoutMs -- is amount of time to wait for first byte to appeaer (can be 0)
//Return an indication of error, no packet received, the expected packet was received or another packet was received.
MESSAGE_READ ANTPlus::readPacket( ANT_Packet * packet, int packetSize, int wait_timeout = 0 )
{
    MESSAGE_READ ret_val = MESSAGE_READ_NONE;
    {
        ret_val = readPacketInternal(packet, packetSize, wait_timeout);
        if (ret_val == MESSAGE_READ_INTERNAL)
        {
            if( packet->msg_id == msgResponseExpected )
            {
                //ANTPLUS_DEBUG_PRINTLN("Received expected message!");
                msgResponseExpected = MESG_INVALID_ID; //Not waiting on anything anymore
                ret_val = MESSAGE_READ_EXPECTED;
            }
            else
            {
                //ANTPLUS_DEBUG_PRINTLN("Received unexpected message!");
                ret_val = MESSAGE_READ_OTHER;
            }
        }
    }
    return ret_val; 
}





//TODO: Move these to progmem
#ifdef ANTPLUS_MSG_STR_DECODE
//! returns msg_id converted into a human readable string.
const char * ANTPlus::get_msg_id_str(byte msg_id)
{
  switch (msg_id)
  {
      case MESG_RESPONSE_EVENT_ID:
        return "RESPONSE_EVENT";
      case MESG_CAPABILITIES_ID:
        return "CAPABILITIES";
      case MESG_BROADCAST_DATA_ID:
        return "BROADCAST_DATA";
      case MESG_ASSIGN_CHANNEL_ID:
        return "ASSIGN_CHANNEL";
      case MESG_CHANNEL_MESG_PERIOD_ID:
        return "CHANNEL_MESG_PERIOD";
      case MESG_CHANNEL_SEARCH_TIMEOUT_ID:
        return "CHANNEL_SEARCH_TIMEOUT";
      case MESG_CHANNEL_RADIO_FREQ_ID:
        return "CHANNEL_RADIO_FREQ";

      case MESG_REQUEST_ID:
        return "REQUEST";

      case MESG_START_UP:
        return "START_UP";

      case MESG_NETWORK_KEY_ID:
        return "NETWORK_KEY";
      case MESG_SYSTEM_RESET_ID:
        return "SYSTEM_RESET";
      case MESG_OPEN_CHANNEL_ID:
        return "OPEN_CHANNEL";
      case MESG_CHANNEL_ID_ID:
        return "CHANNEL_ID";

      default:
        return "...";

    } 
}
#endif


//NOTE: This function calls Serial.println directly
void ANTPlus::serial_print_byte_padded_hex(byte value)
{
    if(value <= 0x0F)
  {
      Serial.print(0, HEX);
  }
  Serial.print(value, HEX);
}

//NOTE: This function calls Serial.println directly
void ANTPlus::serial_print_int_padded_dec(long int value, unsigned int width, boolean final_carriage_return)
{
  int div_num = value;
  int div_cnt = 0;
  while( div_num /= 10 )
  {
    div_cnt++;
  }
  if(div_cnt < width)
  {
    div_cnt = width - div_cnt - 1;
  }
  else
  {
    div_cnt = 0;
  }
  while( div_cnt-- )
  {
    Serial.print("0");
  }
  if(final_carriage_return)
  {
    Serial.println(value);
  }
  else
  {
    Serial.print(value);
  }
}


//! Print a packet for debugging. Does decoding of some ids/codes
//NOTE: This function calls Serial.println directly
void ANTPlus::printPacket(const ANT_Packet * packet, boolean final_carriage_return = true)
{
  Serial.print("RX[");
  serial_print_int_padded_dec( rx_packet_count, 6, false );
  Serial.print("] @ ");
  serial_print_int_padded_dec( millis(), 8, false );
  Serial.print(" ms > ");
//  Serial.print("0x");
//  serial_print_byte_padded_hex(packet->sync);
//  Serial.print(" ");
  Serial.print( packet->length );
  Serial.print("B ");

#if defined(ANTPLUS_MSG_STR_DECODE)
  Serial.print( get_msg_id_str (packet->msg_id) );
  Serial.print("[0x");
  serial_print_byte_padded_hex(packet->msg_id);
  Serial.print("]");
#else
  Serial.print("0x");
  serial_print_byte_padded_hex(packet->msg_id);
#endif //defined(ANTPLUS_MSG_STR_DECODE)
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

//Must be called with the same channel until an error or established (i.e. don't start with a different channel in the middle -- one channel at a time)
//TODO: Test that interleaved calls is relaxed (s.b. with moving of state_counter to struct)
//Must not be called with the same channel after it returns ESTABLISHED as that will attempt to reopen....
ANT_CHANNEL_ESTABLISH ANTPlus::progress_setup_channel( ANT_Channel * channel )
{
  boolean sent_ok = true; //Defaults as true as we want to progress the state counter
  
  ANT_CHANNEL_ESTABLISH ret_val = ANT_CHANNEL_ESTABLISH_PROGRESSING;

  if(channel->state_counter == 0)
  {
    //ANTPLUS_DEBUG_PRINTLN("progress_setup_channel() - Begin");  
  }
  else
  if(channel->state_counter == 1)
  {
    //Request CAPs
    sent_ok = send(MESG_REQUEST_ID, MESG_CAPABILITIES_ID/*Expected response*/, 2, 0/*Channel number always 0*/, MESG_CAPABILITIES_ID);
  }
  else
  if(channel->state_counter == 2)
  {
   // Assign Channel
    //   Channel: 0
    //   Channel Type: for Receive Channel
    //   Network Number: 0 for Public Network
    sent_ok = send(MESG_ASSIGN_CHANNEL_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 3, channel->channel_number, 0, channel->network_number); 
  }
  else
  if(channel->state_counter == 3)
  {
 
    // Set Channel ID
    //   Channel Number: 0
    //   Device Number LSB: 0 for a slave to match any device
    //   Device Number MSB: 0 for a slave to match any device
    //   Device Type: bit 7 0 for pairing request bit 6..0 for device type
    //   Transmission Type: 0 to match any transmission type
    sent_ok = send(MESG_CHANNEL_ID_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 5, channel->channel_number, 0, 0, channel->device_type, 0);
  }
  else
  if(channel->state_counter == 4)
  {
    // Set Network Key
    //   Network Number
    //   Key
    sent_ok = send(MESG_NETWORK_KEY_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 9, channel->network_number, channel->ant_net_key[0], channel->ant_net_key[1], channel->ant_net_key[2], channel->ant_net_key[3], channel->ant_net_key[4], channel->ant_net_key[5], channel->ant_net_key[6], channel->ant_net_key[7]);
  }
  else
  if(channel->state_counter == 5)
  {
    // Set Channel Search Timeout
    //   Channel
    //   Timeout: time for timeout in 2.5 sec increments
    sent_ok = send(MESG_CHANNEL_SEARCH_TIMEOUT_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 2, channel->channel_number, channel->timeout);
  }
  else
  if(channel->state_counter == 6)
  {
    //ANT_send(1+2, MESG_CHANNEL_RADIO_FREQ_ID, CHAN0, FREQ);
    // Set Channel RF Frequency
    //   Channel
    //   Frequency = 2400 MHz + (FREQ * 1 MHz) (See page 59 of ANT MPaU) 0x39 = 2457 MHz
    sent_ok = send(MESG_CHANNEL_RADIO_FREQ_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 2, channel->channel_number, channel->freq);
  }
  else
  if(channel->state_counter == 7)
  {
    // Set Channel Period
    sent_ok = send(MESG_CHANNEL_MESG_PERIOD_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 3, channel->channel_number, (channel->period & 0x00FF), ((channel->period & 0xFF00) >> 8));
  }
  else
  if(channel->state_counter == 8)
  {
    //Open Channel
    sent_ok = send(MESG_OPEN_CHANNEL_ID, MESG_RESPONSE_EVENT_ID/*Expected response*/, 1, channel->channel_number);
  }
  else
  if(channel->state_counter == 9)
  {
    //Check if the last message has been responded to
    if(!awaitingResponseLastSent())
    {
      ret_val = ANT_CHANNEL_ESTABLISH_COMPLETE;
      //ANTPLUS_DEBUG_PRINTLN("progress_setup_channel() - Complete");  
    }
    else
    {
      sent_ok = false; //Set this to false as it enables the error checking below.
    }
  }

  
  if(sent_ok)
  {
    channel->state_counter++;
  }
  else
  {
    //Not always an error - as sometimes there are messages in the queue that are awaiting a response
    //ANTPLUS_DEBUG_PRINTLN("Issue sending....");
    {
        if( digitalRead(RTS_PIN) == LOW)
        {
          static unsigned int sending_issue_counter = 0;
          sending_issue_counter++;
          
          //This should clear on the next loop ( this should be after ANT asserts -- but could conceivably be before it has even responded)
          // The ISR sets a loop flag and the ISR should be triggered within 50 usecs
          if(sending_issue_counter >= 50)
          {
            //Seems like we missed an RTS assertion.....
            ANTPLUS_DEBUG_PRINTLN( "Missed an RTS or none was executed by ANT. Restarting...." );
            sending_issue_counter = 0;
            hardwareReset();
            
            ret_val = ANT_CHANNEL_ESTABLISH_ERROR;
          }
        }
    }
  }
  
  channel->channel_establish = ret_val;
  
  return ret_val;
}

//! A function that is called when an RTS interrupt is received in the main program
void   ANTPlus::rTSHighAssertion()
{
      //"Waiting for ANT to RTS (let us send again)."
      //Need to make sure it is low again
      while( digitalRead(RTS_PIN) != LOW )
      {
        //TODO: Is this a bad idea in an ISR?
        delayMicroseconds(50);
      }
      clear_to_send = true;
}


//!Put ANT module into sleep mode. NOTE: This seems to have some issues.
void ANTPlus::sleep(boolean activate_sleep)
{
    int logic_level = HIGH; //Sleep
    if(!activate_sleep)
    {
        logic_level = LOW; //Wake
    }
    digitalWrite(SLEEP_PIN, logic_level);
}

//!Put ANT module into suspend mode. NOTE: Not implemented
void ANTPlus::suspend(boolean activate_suspend)
{
    //TODO:
    assert(false);
}

// SDM -- 6.2.2
//Distance, time and stride count
int ANTPlus::update_sdm_rollover( byte MessageValue, unsigned long int * Cumulative, byte * PreviousMessageValue )
{
  //Initialize CumulativeDistance to 0
  //Above is external to this function
  //PreviousMessageDistance is set to -1 to indicate no previous message -- external to this function
  //initialize PreviousMessageDistance to the distance in the first SDM data message.
  if((*PreviousMessageValue) == -1)
  {
    (*PreviousMessageValue) = MessageValue;
    //This assumes that the first measurement we get from device is at 'point 0' -- any first measurement is therefore ignored in the cumulative
  }
  else
  {
    //For each subsequent SDM sensor Data message
    //a. CumulativeDistance += MessageDistance – PreviousMessageDistance
    (*Cumulative) += (MessageValue - (*PreviousMessageValue));
  
    //b. If PreviousMessageDistance > MessageDistance, CumulativeDistance += 256m
    if ((*PreviousMessageValue) > MessageValue)
    {
      (*Cumulative) += 256; //All fields rollover on this amount
    }
    //c. PreviousMessageDistance = MessageDistance
    (*PreviousMessageValue) = MessageValue;
  }
  return (*Cumulative);
}

