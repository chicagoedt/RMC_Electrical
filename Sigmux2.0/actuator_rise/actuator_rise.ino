#define ACTUATOR_UP 21
#define ACTUATOR_DOWN 23

int PRESS = 0;

void setup()
{
  Serial.begin(115200);
  // make ACTUATOR_UP as input
  pinMode(ACTUATOR_UP, INPUT);
}

void loop()
{
  int input;
  // Read input
  input = digitalRead(ACTUATOR_UP);
  if(input == 1)
  {
    PRESS += 10;
    Serial.println("Counter was incremented!");
    // Raise both Actuators
    sendCanCommand();
  }
}

void sendCanCommand()
{
  String canCommand;
  // Actuator 1
  canCommand = "@04!G 1 ";
  canCommand += PRESS;
  Serial.print("CAN:  ");
  Serial.println(canCommand);
  Serial1.write(canCommand.c_str());
  
  // Actuator 2
  canCommand = "@04!G 2 ";
  canCommand += PRESS;
  Serial.print("CAN:  ");
  Serial.println(canCommand);
  Serial1.write(canCommand.c_str());
  
}
