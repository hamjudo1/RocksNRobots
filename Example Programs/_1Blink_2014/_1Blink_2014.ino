//one color
int led = 3;     //Declares Var led to pin 3
int x=0;
void setup()
{
  pinMode(led, OUTPUT);    //Declares pin 3 to be an output   
}
void loop(){
 //for(x, x < 10, x++)   //repeats while x is < 10 x goes up by a constant 1
{
  digitalWrite(led, HIGH); //sets led on HIGH
  delay(100);    //waits 1 sec
  digitalWrite(led, LOW);    //sets led to LOW
  delay(100);   //waits 1 sec
}
}
