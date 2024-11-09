// Harsh, 2103116
// Harsh Choudhary, 2103117
// Soham Jadhav, 2103136

#include "./qLearningEff.h"

// Ultrasonic Sensor Pins (1 for left, 2 for right)
#define trigPin1 2
#define echoPin1 5
#define trigPin2 12
#define echoPin2 13

// IR Sensor Pins (1 for left, 2 for right)
#define irSensor1 7
#define irSensor2 4

// Motor Pins
#define motorLeftIN1 10
#define motorLeftIN2 11
#define motorRightIN3 8
#define motorRightIN4 9


int enr = 3;
int enl = 6;

int vlspeed = 165;    
int vrspeed = 125;    
int tlspeed = 165-15;
int trspeed = 125-15;
int tdelay = 20;
int oneSec = 100;

char state[stateRepresentationSize];
char nextState[stateRepresentationSize];
float reward;

int flag = 0;

void printQTable(){
	Serial.print("The number of entries: ");
	Serial.println(qTableSize);
	for(int i=0;i<qTableSize;++i){
		Serial.print("State is:");
		printStr(qTable[i].state);
		Serial.print(" Action is:");
		Serial.print(qTable[i].action);
		Serial.print(" Value is:");
		Serial.println(qTable[i].value);		
	}
}

void saveQTable() {
    EEPROM.put(0, sentinelValueForEEPROM);
    int startAddr = sizeof(sentinelValueForEEPROM);
    EEPROM.put(startAddr, qTableSize);
    startAddr = startAddr + sizeof(int);
    for (int i = 0; i < qTableSize; ++i) {
    	// Calculate the address
        int addr = startAddr + (i) * sizeof(qTableEntry); 
        // Save each entry
        EEPROM.put(addr, qTable[i]); 
    }
}

void resetSentinelValue() {
    // Write the sentinel value to the EEPROM
    EEPROM.put(0, 0);
    Serial.println("Sentinel value has been reset.");
}

void loadQTable() {
    int present;
    EEPROM.get(0, present);

    // Check if the EEPROM has been initialized
    if (present != sentinelValueForEEPROM) {
        // delay(100);
        Serial.println("EEPROM not initialized. Writing sentinel value.");
        return;
    } else {
        Serial.println("EEPROM already initialized.");
        // Skip sentinel value
        int startAddr = sizeof(sentinelValueForEEPROM); 
        // Get the size of the Q-table
        EEPROM.get(startAddr, qTableSize); 
        // Skip size of table byte address value
        startAddr += sizeof(int); 
        for (int i = 0; i < qTableSize; ++i) {
        	// Calculate the address
            int addr = startAddr + (i) * sizeof(qTableEntry); 
            // Load each entry
            EEPROM.get(addr, qTable[i]); 
        }
    }
}


void backward(){
  digitalWrite(motorRightIN3,LOW);
  digitalWrite(motorRightIN4,HIGH);
  digitalWrite(motorLeftIN1,LOW);
  digitalWrite(motorLeftIN2,HIGH);
  analogWrite (enl,vlspeed);
  analogWrite (enr,vrspeed);
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(motorLeftIN1, HIGH);
  digitalWrite(motorLeftIN2, LOW);
  digitalWrite(motorRightIN3, HIGH);
  digitalWrite(motorRightIN4, LOW);
  analogWrite (enl,vlspeed);
  analogWrite (enr,vrspeed);
}

// slow down left motor or reverse right motor, check whether multiplier needed here too?
void turnRight() {
  Serial.println("Moving Right");
  digitalWrite(motorLeftIN1, HIGH);
  digitalWrite(motorLeftIN2, LOW);
  digitalWrite(motorRightIN3, LOW);
  digitalWrite(motorRightIN4, HIGH);
  analogWrite (enl,tlspeed);
  analogWrite (enr,trspeed);
}

// slow down right motor or reverse left motor, check whether multiplier needed here too?
void turnLeft() {
  Serial.println("Moving Left");
  digitalWrite(motorLeftIN1, LOW);
  digitalWrite(motorLeftIN2, HIGH);
  digitalWrite(motorRightIN3, HIGH);
  digitalWrite(motorRightIN4, LOW);
  analogWrite (enl,tlspeed);
  analogWrite (enr,trspeed);
}

void stopMoving() {
  Serial.println("Stop");
  analogWrite (enr,0);
  analogWrite (enl,0);
}

// cm = (microseconds)/(2*29.1)
float getDistance(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
  return distance;
}

void setup() {
  TCCR0B = TCCR0B & B11111000 | B00000010 ;
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);

  pinMode(motorLeftIN1, OUTPUT);
  pinMode(motorLeftIN2, OUTPUT);
  pinMode(motorRightIN3, OUTPUT);
  pinMode(motorRightIN4, OUTPUT);

  Serial.begin(9600);
  
  Serial.println("Setup called");
  // loadQTable();
  
  // Seed randomness using an unconnected analog pin
  randomSeed(analogRead(0));
}

void loop() {
  // Read IR sensor values
  int leftSensor = digitalRead(irSensor1);  
  int rightSensor = digitalRead(irSensor2); 

  // Fetch and print ultrasonic sensor readings
  long distanceLeft = getDistance(trigPin1, echoPin1);  
  long distanceRight = getDistance(trigPin2, echoPin2); 

  if(flag==0){
    getInitialState(distanceLeft, distanceRight, state);
    flag = 1;
  }else{    
    copyState(state, nextState);
  }  
  
  char action = act(state);
  getRewardNextState(distanceLeft, distanceRight, state, action, nextState, &reward);
  updateQTable(state, action, nextState, reward);
  // action = actionMove;
  if(action == actionStop){
    stopMoving();
  }
  // Logic for line following behavior
  // Case 1: Both sensors off the line (LOW), move forward
  else if (leftSensor == LOW && rightSensor == LOW) {
    moveForward();
  }
  // Case 2: Left sensor on the line, right sensor off the line, turn left
  else if (leftSensor == HIGH && rightSensor == LOW) {
    turnLeft();
  }
  // Case 3: Left sensor off the line, right sensor on the line, turn right
  else if (leftSensor == LOW && rightSensor == HIGH) {
    turnRight();
  }
  // Case 4: Both sensors on the line, stop
  else if (leftSensor == HIGH && rightSensor == HIGH) {
    stopMoving();
  }
  
  printStr(state);
  Serial.print(" --");
  Serial.print(action);
  Serial.print("-->");
  printStr(nextState);
  Serial.print(" : ");
  Serial.println(reward);

  // printQTable();

  Serial.println("\n-------------------------------------------------------------");
  // saveQTable();
  // resetSentinelValue();
  // return;
  
  // 1s delay
  // ***demerit -> the speed to motors will be constant for 1s as the signal value is already set/unset
  delay(oneSec); 
}

