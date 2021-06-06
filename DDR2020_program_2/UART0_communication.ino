void init_USART0(uint8_t baud_rate)
{
  //UCSR0B = (1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0);
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
    
  switch (baud_rate)
  {
    case 1:
    UBRR0L = 103; //9600
    break;
    case 2:
    UBRR0L = 51; //19200
    break;
    case 3:
    UBRR0L = 25; //38400
    break;
    case 4:
    UBRR0L = 16; //57600
    break;
    case 5:
    UBRR0L = 12; //76800
    break;
    case 6:
    UBRR0L = 8; //115200
    break;
    default:
    UBRR0L = 103; //9600
  }
}

void serialWrite()
{
  String c = transmit_packet;
  // put data into transmit buffer
  for(uint8_t i=0; i<c.length(); i++)
  {
    appendSerial(c[i]); // // write data into ring buffer at proper index
  }
  
  start_tx_interrupt();
}

void appendSerial(char c)
{
  tx_buffer[tx_buffer_write_pos] = c;
  tx_buffer_write_pos++;

  if(tx_buffer_write_pos >= TX_BUFFER_SIZE)
  {
    tx_buffer_write_pos = 0;
  }
}

void start_tx_interrupt()
{
  UCSR0B |= (1<<UDRIE0);
}

ISR(USART0_UDRE_vect)
{
  if(tx_buffer_write_pos != tx_buffer_read_pos)
  {
    UDR0 = tx_buffer[tx_buffer_read_pos];
    tx_buffer_read_pos++;

    if(tx_buffer_read_pos >= TX_BUFFER_SIZE)
    {
      tx_buffer_read_pos=0;
    }
  }
  else
  {
    UCSR0B &= ~(1<<UDRIE0);    
  }
}
