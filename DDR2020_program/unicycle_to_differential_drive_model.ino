void vehicle_twist_to_motor_rpm(float Vc, float Wc)
{
  // degree/sec to rad/sec
  Wc = deg2rad(Wc);

  // unicycle to differential drive model
  rpm01 = (2*Vc+Wc*L)/(2*r);
  rpm02 = (2*Vc-Wc*L)/(2*r);

  // rad/sec to deg/sec
  rpm01 = rad2deg(rpm01);
  rpm02 = rad2deg(rpm02);

  //Serial.print(rpm01); Serial.print('\t');
  //Serial.print(rpm1); Serial.print('\t');
  //Serial.print(rpm02); Serial.print('\t');
  //Serial.println(rpm2);
}
