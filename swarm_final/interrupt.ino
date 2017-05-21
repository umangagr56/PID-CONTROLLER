
void interrupt_init(){
  //Initialise the Timer and attach Interrupt.
  Timer1.initialize(50000);
  Timer1.attachInterrupt(calculateRPM);
  //Attach Interrupts to the encoders input.
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinD),increase_CounterB, CHANGE);
}

void increase_CounterA() {
  ext_counterA++;
}

void increase_CounterB(){
  ext_counterB++;

}

void pid_dis();

void calculateRPM(){
  curr_rpmA=(ext_counterA*60)/(CPR*gear_ratio*del_t);
  curr_rpmB=(ext_counterB*60)/(CPR*gear_ratio*del_t);
  
  if(motion_direction==1){
    Dl=2*PI*wheel_radius*curr_rpmA*del_t/60;
    Dr=2*PI*wheel_radius*curr_rpmB*del_t/60;
  }
  else if(motion_direction==2){
    Dl=(-1)*2*PI*wheel_radius*curr_rpmA*del_t/60;
    Dr=2*PI*wheel_radius*curr_rpmB*del_t/60;
  }
  else if(motion_direction==3){
    Dl=2*PI*wheel_radius*curr_rpmA*del_t/60;
    Dr=(-1)*2*PI*wheel_radius*curr_rpmB*del_t/60;
  }
  else if(motion_direction==4){
    Dl=(-1)*2*PI*wheel_radius*curr_rpmA*del_t/60;
    Dr=(-1)*2*PI*wheel_radius*curr_rpmB*del_t/60;
  }  

  Dc+=(Dl+Dr)/2;
  current_theta+=(Dr-Dl)/w2w;
  if(current_theta>2*PI)
  {
    current_theta-=2*PI;
  }
  curr_pos_x+=(Dc*cos(current_theta));
  curr_pos_y+=(Dc*sin(current_theta));
  
  //Serial.print(curr_pos_x);
  //Serial.print("\t");
  //Serial.println(curr_pos_y);
  
  ext_counterA=0;
  ext_counterB=0;
  pid_dis();
  
}

/*
const double kpa=1;
const double kia=1;
const double kda=1;

const double kpb=1;
const double kib=1;
const double kdb=1;

double current_pos_x,current_pos_y,set_pos_x,set_pos_y;
double current_dis,set_dis=0;
double err_dis,prev_err_dis,sum_err;
double set_theta,theta;
double err_theta,prev_err_theta,sum_err_theta;
double velocity, omega;

void pid_dis(){ 
  current_dis = sqrt((sq(set_pos_x-current_pos_x)+(sq(set_pos_y-current_pos_y))));
  err_dis = current_dis - set_dis; 
  sum_err += err_dis;
  double p;
  if((kpa*err_dis + kia*sum_err + kda*(err_dis-prev_err_dis))<0)
  p = 0;
 else
  p = (kpa*err_dis + kia*sum_err + kda*(err_dis-prev_err_dis));
   theta=atan((set_pos_x-current_pos_x)/(set_pos_y-current_pos_y));
  if(set_pos_y>current_pos_y){
      if(set_pos_x>current_pos_x){
          set_theta = 2*PI - theta;
      }
      else{
          set_theta = (-1)*theta;
      }
  }
   else{
      set_theta = (-1)*theta + PI; 
       }
  err_theta= set_theta - current_theta;
  sum_err_theta += err_theta;
  double q;
  if((kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta))<0)
      q = 0;
  else
      q = kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta);
  
  velocity = map(p,0,max_distance*kpa,0,max_velocity);
  omega = map(q,0,max_angle*kpb,0,max_omega);
  set_rpmA = ((120*velocity/(2*PI*wheel_radius)) + (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  set_rpmB =  ((120*velocity/(2*PI*wheel_radius))- (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  prev_err_dis = err_dis;
  prev_err_theta = err_theta;
}     */  
const double kpa=1;
const double kia=1;
const double kda=1;

const double kpb=1;
const double kib=1;
const double kdb=1;



void pid_dis(){ 
  current_dis = sqrt((sq(set_pos_x-current_pos_x)+(sq(set_pos_y-current_pos_y))));
  err_dis = current_dis - set_dis; 
  sum_err += err_dis;
  double p;
  if((kpa*err_dis + kia*sum_err + kda*(err_dis-prev_err_dis))<0)
  p = 0;
 else
  p = (kpa*err_dis + kia*sum_err + kda*(err_dis-prev_err_dis));
   theta=atan((set_pos_y-current_pos_y)/(set_pos_x-current_pos_x));
  if(set_pos_y>current_pos_y){
      if(set_pos_x>current_pos_x){
          set_theta = theta;
      }
      else{
          set_theta = PI+theta;
      }
  }
   else{
       if(set_pos_x>current_pos_x){
           set_theta = theta + 2*PI;
       }
       else{
          set_theta=PI+theta;  
         }
       }
  e = set_theta - current_theta;
 err_theta = atan2(sin(e),cos(e));
 if(err_theta > 0){
  sum_err_theta += err_theta;
  double q;
     if((kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta))<0)
      q = 0;
     else
      q = kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta);
  
  velocity = map(p,0,max_distance*kpa,0,max_velocity);
  omega = map(q,0,max_angle*kpb,0,max_omega);
  set_rpmA = ((120*velocity/(2*PI*wheel_radius)) + (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  set_rpmB =  ((120*velocity/(2*PI*wheel_radius))- (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  prev_err_dis = err_dis;
  prev_err_theta = err_theta;
}      
 else{
   if((kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta))<0)
      q = 0;
     else
      q = kpb*err_theta + kib*sum_err_theta+kdb*(err_theta - prev_err_theta);
  
  velocity = map(p,0,max_distance*kpa,0,max_velocity);
  omega = map(q,0,max_angle*kpb,0,max_omega);
  set_rpmB = ((120*velocity/(2*PI*wheel_radius)) + (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  set_rpmA =  ((120*velocity/(2*PI*wheel_radius))- (60*omega/((2*PI*wheel_radius)*(w2w))))/2;
  prev_err_dis = err_dis;
  prev_err_theta = err_theta;
 }
}  

