int rgb_cam = 13;
int thermal_cam = 7;

// ------ RGB camera FLIR BlackflyS USB3 GPIO cable settting ------
// brown: GND, camera power ground
// blue: Opto GND, Opto-isolated ground
// green: power input, auxiliary input voltage (DC)
// black: OPTOIN, Opto-isolated input
// !!!!!!!!!!! Rising Edge Trigger !!!!!!!!!!!!!!!!!!!!!!1

// BROWN, BLUE -> GND
// GREEN -> Power
// BLACK -> 13

// ------ Thermal camera FLIR A6701 GPIO cable settting ------
// Outer: GND
// Inner: Sync (trigger) input
// !!!!!!!!!!! Rising Edge Trigger !!!!!!!!!!!!!!!!!!!!!!1

// Outer -> GND
// Inner -> 7

int flag=0;

void setup() {
  Serial.begin(9600);
  pinMode(rgb_cam, OUTPUT);
  pinMode(thermal_cam, OUTPUT);

  // Pins Initialization
  digitalWrite(rgb_cam,LOW);
  digitalWrite(thermal_cam,LOW);
  delay(50);
}

void loop() {
  if(flag==1){
    digitalWrite(rgb_cam, LOW); // set camera signal to HIGH, wait for next trigger signal
    digitalWrite(thermal_cam, LOW); // set thermal camera signal to LOW, wait for next trigger signal
 
    delay(500); //frame rate

    digitalWrite(rgb_cam, HIGH); // trigger camera
    digitalWrite(thermal_cam, HIGH); // trigger camera

    delay(5);

  }

  char r = Serial.read();

  if(r=='s'){
    flag=1;
  }
  else if(r=='e'){
    flag=0;
  }
  else{
    flag=flag;
  }

  delay(5);
 
}
