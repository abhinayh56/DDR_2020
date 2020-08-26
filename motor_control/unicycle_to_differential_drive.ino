void twist_to_rpm(float Vc, float Wc)
{
  // degree/sec to rad/sec
  Wc = Wc*0.01745329251994329576923690768489; //Wc*PI/180;

  // unicycle to differential drive model
  rpm01 = (2*Vc+Wc*L)/(2*r);
  rpm02 = (2*Vc-Wc*L)/(2*r);

  // rad/sec to rad/sec
  //rpm01 = rpm01*9.5492965855137201461330258023509; //rpm01*60/(2*PI);
  //rpm02 = rpm02*9.5492965855137201461330258023509;

  // rad/sec to deg/sec
  rpm01 = rpm01*57.295779513082320876798154814105; //rpm01*180/PI;
  rpm02 = rpm02*57.295779513082320876798154814105;

  //Serial.print(rpm01); Serial.print('\t');
  //Serial.print(rpm1); Serial.print('\t');
  //Serial.print(rpm02); Serial.print('\t');
  //Serial.println(rpm2);
}
