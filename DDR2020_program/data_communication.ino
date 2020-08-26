void communicate_data()
{
  send_data();
  receive_data();
}

void send_data()
{
  Serial.print(X); Serial.print('\t');
  Serial.print(Y); Serial.print('\t');
  Serial.print(PHI); Serial.print('\t');
  Serial.print(Vc); Serial.print('\t');
  Serial.println(Wc);
}

void receive_data()
{
  rx_str = "";  
  while(Serial.available()>0)
  {
    rx_char = Serial.read();
    rx_str += (char)rx_char;
  }

  if(rx_char=='c')
  {
    String Vc0_str = "";
    String Wc0_str = "";
      
    //unsigned int rx_srt_len = rx_str.length();
    unsigned int ind_a = rx_str.indexOf("a");
    unsigned int ind_b = rx_str.indexOf("b");
    unsigned int ind_c = rx_str.indexOf("c");
    
    for(int i=ind_a+1;i<ind_b;i++)
    {
      Vc0_str += rx_str[i];
    }
  
    for(int i=ind_b+1;i<ind_c;i++)
    {
      Wc0_str += rx_str[i];
    }
    
    //Serial.print("rx_srt: "); Serial.println(rx_str);
    //Serial.print("rx_srt_len: "); Serial.println(rx_srt_len);
    //Serial.print("ind_a: "); Serial.println(ind_a);
    //Serial.print("ind_b: "); Serial.println(ind_b);
    //Serial.print("ind_c: "); Serial.println(ind_c);
    //Serial.print("Vc0_str: "); Serial.println(Vc0_str);
    //Serial.print("Wc0_str: "); Serial.println(Wc0_str);
    rx_char = 'z';

    for(int i=0;i<ind_b-ind_a;i++)
    {
      Vc0_char[i] = Vc0_str[i];
    }
    for(int i=ind_b-ind_a;i<6;i++)
    {
      Vc0_char[i] = ' ';
    }

    for(int i=0;i<ind_c-ind_b;i++)
    {
      Wc0_char[i] = Wc0_str[i];
    }
    for(int i=ind_c-ind_b;i<6;i++)
    {
      Wc0_char[i] = ' ';
    }

    Vc0 = atof(Vc0_char);
    Wc0 = atof(Wc0_char);
    Serial.print("Vc0: "); Serial.println(Vc0);
    Serial.print("Wc0: "); Serial.println(Wc0);
  }
  
  Vc0 = 0.1;
  Wc0 = rad2deg((0.1/0.6));
}

void receive_data2()
{
  rx_str = "";
  String Vc0_str = "";
  String Wc0_str = "";
  
  while(Serial.available()>0)
  {
    rx_char = Serial.read();
    
    if(rx_char == 'a')
    {
      Vc0_comming = 1;
      Wc0_comming = 0;

      Vc0_str = "";
      Wc0_str = "";
    }
    if(rx_char == 'b')
    {
      Vc0_comming = 0;
      Wc0_comming = 1;

      //Vc0_str = "";
      Wc0_str = "";
    }
    if(rx_char == 'c')
    {
      Vc0_comming = 0;
      Wc0_comming = 0;
      
      Vc0 = Vc0_str.toInt();
      Vc0 = (float)Vc0*0.001;

      Wc0 = Wc0_str.toInt();
      Wc0 = (float)Wc0*0.001;
      
      Serial.print(Vc0,4); Serial.print('\t'); Serial.println(Wc0,4);
    }

    if(Vc0_comming && (char)rx_char != 'a' && (char)rx_char != 'b' && (char)rx_char != 'c')
    {
      Vc0_str += (char)rx_char;
    }

    if(Wc0_comming && (char)rx_char != 'a' && (char)rx_char != 'b' && (char)rx_char != 'c')
    {
      Wc0_str += (char)rx_char;
    }
  }
    
  //Vc0 = 0.1;
  //Wc0 = rad2deg((0.1/0.6));
}
