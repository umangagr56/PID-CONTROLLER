
const float kpOm=0.5;
const float kpV=0.5;

void interrupt_init(){
//   Initialise the Timer and attach Interrupt.
//   Timer1.initialize(50000);
//   Timer1.attachInterrupt(calculateRPM);
//   Attach Interrupts to the encoders input.
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinD),increase_CounterB, CHANGE);
}

void increase_CounterA() {
  ext_counterA++;
}

void increase_CounterB(){
  ext_counterB++;

}


void calculateRPM(){
  curr_distance = sqrt(sq(set_pos_x-curr_pos_x)+sq(set_pos_y-curr_pos_y));
  set_velocity = kpV*curr_distance;
  
  set_theta = atan2((set_pos_y-curr_pos_y),(set_pos_x-curr_pos_x));
  error_theta = (set_theta-theta);
  omega  = kpOm*error_theta;

  set_rpmA = (2*set_velocity+omega*w2w)/(2*wheel_radius);
  set_rpmB = (2*set_velocity-omega*w2w)/(2*wheel_radius);
  
  if(set_rpmA<0){
    set_rpmA*=-1;
    motorA_backward();
  }
  else
    motorA_forward();
    
  if(set_rpmB<0){
    set_rpmB*=-1;
    motorB_backward();
  }
  else
    motorB_forward();
    
  set_rpmA = map(set_rpmA, 0, kpOm*PI*w2w/wheel_radius, 0, max_rpmA);
  set_rpmB = map(set_rpmB, 0, kpOm*PI*w2w/wheel_radius, 0, max_rpmB);
  
  curr_rpmA=(ext_counterA*60)/(CPR*gear_ratio*del_t);
  curr_rpmB=(ext_counterB*60)/(CPR*gear_ratio*del_t);
  
  
  //Serial.print(curr_pos_x);
  //Serial.print("\t");
  //Serial.println(curr_pos_y);
  
  ext_counterA=0;
  ext_counterB=0;
    
}


