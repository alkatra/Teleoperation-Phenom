 int motorright = 9;                                                              
int motorrightdir  = 7;
int motorleft = 10;
int motorleftdir  = 8;

long duration;
int distance;

int speed = 120;

const int echoPinLeft = 13;
const int trigPinLeft = 11;

const int echoPinFront = 4;
const int trigPinFront = 5;

const int echoPinRight = 2;
const int trigPinRight = 3;

//For serial receive.
const byte numChars = 11;
char receivedChars[numChars]; // an array to store the received data
String received; //The data as a string
boolean newData = false;

void setup() {
  pinMode(motorright, OUTPUT);                                                      
  pinMode(motorleft, OUTPUT);     
  pinMode(motorrightdir, OUTPUT);  
  pinMode(motorleftdir, OUTPUT);  

  pinMode(trigPinLeft, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLeft, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinRight, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRight, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); //normal printing
}

void loop() 
{

  int left = ultrasonic(echoPinLeft,trigPinLeft);
  int front = ultrasonic(echoPinFront,trigPinFront);
  int right = ultrasonic(echoPinRight,trigPinRight);
  if(left > 99) {
    left = 99;
  }
  if(front > 99) {
    front = 99;
  }
  if(right > 99) {
    right = 99;
  }
  String a,b,c = "";
  if(left < 10) {
    a = "0";
  }
  if(front < 10) {
    b = "0";
  }
  if(right < 10) {
    c = "0";
  }
  String message = "";
  message = message + "[" + a + left + "cm," + b + front + "cm," + c + right + "cm]\n";
  Serial.print(message);
  delay(50);
  recvWithEndMarker();
  processCommand();              
}

int ultrasonic(int echoPin, int trigPin) {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);

// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
return distance;
}

void processCommand() {
 if (newData == true)
 {
  String instruction = received.substring(0,5);
//   String data = received.substring(6,10);
  if(instruction == "MOVEF") forward();
  if(instruction == "MOVEB") backward();
  if(instruction == "TURNR") rightward();
  if(instruction == "TURNL") leftward();
  if(instruction == "STOPN") stop();
  if(instruction == "PIVL") pivotLeft();
  if(instruction == "PIVR") pivotRight();
  if(instruction == "FASTR" && speed < 240) {
    speed += 40;
    forward();
  }
  if(instruction == "SLOWR" && speed > 40) {
    speed -= 40;
    forward();
  }
  newData = false;
 }
}

void recvWithEndMarker() 
{
 static byte ndx = 0;
 char endMarker = '\n';
 char rc;
 
 while (Serial.available() > 0 && newData == false) 
 {
  rc = Serial.read();

  if (rc != endMarker) 
  {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= numChars) 
    {
      ndx = numChars - 1;
    }
 }
 else 
  {
  receivedChars[ndx] = '\0'; // terminate the string
  received = String(receivedChars);
  ndx = 0;
  newData = true;
  }
 }
}


void forward()
{
digitalWrite(motorrightdir, LOW);
analogWrite(motorright,speed); 
digitalWrite(motorleftdir, LOW);
analogWrite(motorleft, speed); 

}

void backward()
{
digitalWrite(motorrightdir, HIGH);
analogWrite(motorright,speed); 
digitalWrite(motorleftdir, HIGH);
analogWrite(motorleft, speed);
}

void leftward()
{
  digitalWrite(motorleftdir, HIGH);
  digitalWrite(motorrightdir, LOW);
  analogWrite(motorleft,180);
  analogWrite(motorright,180);
}

void rightward()
{
  digitalWrite(motorleftdir, LOW);
  digitalWrite(motorrightdir, HIGH);
  analogWrite(motorleft,180);
  analogWrite(motorright,180);
}

void pivotLeft()
{
  digitalWrite(motorleftdir, HIGH);
  digitalWrite(motorrightdir, LOW);
  analogWrite(motorleft,180);
  analogWrite(motorright,180);
  delay(90);
}

void pivotRight()
{
  digitalWrite(motorleftdir, LOW);
  digitalWrite(motorrightdir, HIGH);
  analogWrite(motorleft,180);
  analogWrite(motorright,180);
  delay(90);
}

void stop()
{
analogWrite(motorright, 0); 
analogWrite(motorleft, 0);   
}