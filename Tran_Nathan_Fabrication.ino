int mr1=8; //right motor 1 to pin 8
int mr2=9; //right motor 2 to pin 9
int ml1=10; //left motor 1 to pin 10
int ml2=11; //left motor 2 to pin 11
int sr=6; //right sensor to pin 6
int sl=7; //left sensor to pin 7
int svr=0; //right sensor value
int svl=0; //left sensor value
int led=13; //enable led light
int enr=3; //enable right motor
int enl=5; //enable left motor
int speeds=100 //straight speed (0-255)
int speedt=255 //turn speed (0-255)
int delayt=20 //turn delay (20 ms)

void setup() //sets up outputs and inputs
{
  pinMode(mr1,OUTPUT);
  pinMode(mr2,OUTPUT);
  pinMode(ml1,OUTPUT);
  pinMode(ml2,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(sr,INPUT);
  pinMode(sl,INPUT);
  delay(5000);
}

void loop() //all sensor scenarios
{
  svr=digitalRead(sr);
  svl=digitalRead(sl);
  if (svl==LOW && svr==LOW)
  {
    forward();
  }
  if (svl==HIGH && svr==LOW)
  {
    left();
  }
  if (svl==LOW && svr==HIGH)
  {
    right();
  }
  if (svl==HIGH && svr==HIGH)
  {
    stop();
  }
}

void forward()
{
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  analogWrite(enr,speeds);
  analogWrite(enl,speeds);
}

void backward()
{
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  analogWrite(enr,speeds);
  analogWrite(enl,speeds);
}

void right()
{
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,HIGH);
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  analogWrite(enr,speedt);
  analogWrite(enl,speedt);
  delay(delayt);
}

void left()
{
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,HIGH);
  analogWrite(enr,speedt);
  analogWrite(enl,speedt);
  delay(delayt);
}

void stop()
{
  analogWrite(enr,0);
  analogWrite(enl,0);
}
