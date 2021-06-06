void communicate_data()
{
  send_data_custom();
  receive_data_custom();
}

void send_data_custom()
{
  transmit_packet = "a";
  transmit_packet += (long)(X*100);    // (cm)*100
  transmit_packet += "b";
  transmit_packet += (long)(Y*100);    // (cm)*100
  transmit_packet += "c";
  transmit_packet += (long)(PHI*100);  // (degree)*100
  transmit_packet += "d";
  transmit_packet += (long)(Vc*10000); // (m/sec)*10000
  transmit_packet += "e";
  transmit_packet += (long)(Wc*100);   // (degree/sec)*100
  transmit_packet += "f";
  transmit_packet += "\r\n";
  
  serialWrite();
}

void receive_data_custom()
{
  Vc0 = 0.5;
  Wc0 = rad2deg(Vc0/0.6);
}
