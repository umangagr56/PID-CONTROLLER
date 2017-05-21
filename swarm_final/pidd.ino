const double kpA=1000;
const double kiA=22;
const double kdA=1.3;

const double kpB=1000;
const double kiB=22;
const double kdB=1.3;

void pwm_pid(){
 errorA = set_rpmA - curr_rpmA;
 IA+=errorA;
  double x = 0;
 if((kpA*errorA + kiA*IA + kdA*(errorA-preverrorA))<0)
  x = 0;
 else
  x = (kpA*errorA + kiA*IA + kdA*(errorA-preverrorA)); 
  //Serial.println(x);
 PWMA=map(x, 0, max_rpmA*kpA, 0, 255);
 preverrorA=errorA;

double y = 0;
 errorB = set_rpmB - curr_rpmB;
IB+=errorB;
if((kpB*errorB + kiB*IB + kdB*(errorB-preverrorB))<0)
  y = 0;
  else 
  y = (kpB*errorB + kiB*IB + kdB*(errorB-preverrorB));
 PWMB=map(y, 0, max_rpmB*kpB, 0, 255);
 preverrorB=errorB;

  }


