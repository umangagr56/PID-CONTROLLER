void motorA_forward(){
  motion_directionA=1;
  digitalWrite(InputAA, HIGH);
  digitalWrite(InputBB, LOW);
  }

void motorB_forward(){
  motion_directionB=1;
  digitalWrite(InputAB, HIGH);
  digitalWrite(InputBA, LOW);
  }

void motorA_backward(){
  motion_directionA=0;
  digitalWrite(InputAA, LOW);
  digitalWrite(InputBB, HIGH);
  }

void motorB_backward(){
  motion_directionB=0;
  digitalWrite(InputAB, LOW);
  digitalWrite(InputBA, HIGH);
  }
