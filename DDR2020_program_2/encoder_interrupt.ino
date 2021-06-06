void initialize_interrupt()
{
  //interrupt [2,3,4,5]->[19,18,2,3]
  EICRA = (1 << ISC20) | (1 << ISC30);
  EICRB = (1 << ISC40) | (1 << ISC50);
  EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
  sei();
}

// Right wheel encoder
ISR(INT2_vect)
{
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(a1==1)
  {
    if(b1==1)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }
  else
  {
    if(b1==0)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }

  //phi += Dr/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dr*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

ISR(INT3_vect)
{
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(b1==1)
  {
    if(a1==0)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }
  else
  {
    if(a1==1)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }

  //phi += Dr/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dr*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

// Left wheel encoder
ISR(INT4_vect)
{
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(a2==1)
  {
    if(b2==1)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }
  else
  {
    if(b2==0)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }

  //phi -= Dl/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dl*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

ISR(INT5_vect)
{
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(b2==1)
  {
    if(a2==0)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }
  else
  {
    if(a2==1)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }

  //phi -= Dl/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dl*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}
