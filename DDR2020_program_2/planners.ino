void run_planner()
{
  //calculate_trajectory();
}

void calculate_trajectory()
{
  float th0_max = 180;
  float w_th = 0.25*PI;
  //Serial.println(w_th);
  th01 = th0_max*sin(w_th*(float)timer);
  th02 = th0_max*sin(w_th*(float)timer+PI*0.85);
  timer = timer + dt;
}
