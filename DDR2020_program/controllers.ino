void run_controllers()
{
  //control_angle();
  vehicle_twist_to_motor_rpm(Vc0, Wc0); //m/sec, deg/sec
  control_rpm();
}

void control_angle()
{
  th01 = 360;
  error_th1 = th01 - th1;
  rpm01 = Kp_th1*error_th1;
  rpm01 = constrain(rpm01,-240,240);

  th02 = 360;
  error_th2 = th02 - th2;
  rpm02 = Kp_th2*error_th2;
  rpm02 = constrain(rpm02,-240,240);

  /*Serial.print(th01); Serial.print('\t');
  Serial.print(th1); Serial.print('\t');
  Serial.print(th02); Serial.print('\t');
  Serial.println(th2);*/
}

void control_rpm()
{
  //rpm01 = 120; //[-180,180]
  error_rpm1 = rpm01 - rpm1;
  pwm_I1 = pwm_I1 + Ki1*error_rpm1;
  pwm_I1 = constrain(pwm_I1,-240,240);
  pwm1 = Kp1*error_rpm1 + pwm_I1;
  pwm1 = constrain(pwm1,-240,240);
  command_motor1(pwm1);
  //Serial.print(rpm01); Serial.print('\t');
  //Serial.print(rpm1); Serial.print('\t');

  //rpm02 = 150; //[-180,180]
  error_rpm2 = rpm02 - rpm2;
  pwm_I2 = pwm_I2 + Ki2*error_rpm2;
  pwm_I2 = constrain(pwm_I2,-240,240);
  pwm2 = Kp2*error_rpm2 + pwm_I2;
  pwm2 = constrain(pwm2,-240,240);
  command_motor2(pwm2);
  //Serial.print(rpm02); Serial.print('\t');
  //Serial.println(rpm2);

  //Serial.print(pwm1); Serial.print('\t');
  //Serial.println(pwm2);
}

void command_motor1(float pwm1)
{
  if(pwm1<0)
  {
    digitalWrite(4,1);
    digitalWrite(5,0);
    analogWrite(6,(int)(-1*pwm1));
  }
  else
  {
    digitalWrite(4,0);
    digitalWrite(5,1);
    analogWrite(6,(int)pwm1);
  }
}

void command_motor2(float pwm2)
{
  if(pwm2<0)
  {
    digitalWrite(8,1);
    digitalWrite(9,0);
    analogWrite(7,(int)(-1*pwm2));
  }
  else
  {
    digitalWrite(8,0);
    digitalWrite(9,1);
    analogWrite(7,(int)pwm2);
  }
}
