float deg2rad(float _deg)
{
  //return _deg*PI/180;
  return _deg*0.01745329251994329576923690768489;
}

float rad2deg(float _rad)
{
  //return _rad*180/PI;
  return _rad*57.295779513082320876798154814105;
}

float rad2rpm(float _rad)
{
  //return _rad*60/(2*PI)
  return _rad*9.5492965855137201461330258023509;
}

float rpm2rad(float _rad)
{
  //return _rev*2*PI/60;
  return _rad*0.10471975511965977461542144610932;
}
