void run_estimators()
{
  estimate_motors_rpm();
  estimate_vehicle_position();
  estimate_vehicle_twist();
}

void estimate_motors_rpm()
{
  // angle of rotation of motor
  th1 = (360.0/1700.0)*(float)count1;
  th2 = (360.0/1700.0)*(float)count2;
  //Serial.print(th1); Serial.print('\t');
  //Serial.println(th2);

  // rpm of motor
  rpm1 = (th1 - th_pre1)/dt;
  th_pre1 = th1;
  rpm2 = (th2 - th_pre2)/dt;
  th_pre2 = th2;
  //Serial.print(rpm1); Serial.print('\t');
  //Serial.println(rpm2);
}

void estimate_vehicle_position()
{
  X = x*100;
  Y = y*100;
  PHI = rad2deg(phi);

  //Serial.print(X); Serial.print('\t');
  //Serial.print(Y); Serial.print('\t');
  //Serial.println(PHI);
}

void estimate_vehicle_twist()
{
  Vc = 0.000370882466048795*(rpm1+rpm2);
  Wc = 0.2125*(rpm1-rpm2);
  //Serial.print(Vc); Serial.print('\t');
  //Serial.println(Wc);
}
