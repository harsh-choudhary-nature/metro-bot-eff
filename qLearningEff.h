// This file is only for Q learning related functions
// Harsh Choudhary, 2103117
#include <EEPROM.h>
#define sentinelValueForEEPROM 0x01

#define stateRepresentationSize 3
#define maxEntries 24

// we are not using closeStationThresholdLowForEnvironment anywhere
#define closeStationThresholdLowForEnvironment 10
#define closeStationThresholdHighForEnvironment 80
#define closeSensorValue '0'
#define moderateSensorValue '1'
#define farSensorValue '2'
#define actionStop '0'
#define actionMove '1'
#define false '0'
#define true '1'
#define maxStopTime '5'

#define learningRate 0.2f
#define epsilon 0.0f
#define discount 0.95f

/*
stationArrived values [0,1] in char, indicates whether at the station or not
movedAfterStop values [0,1] in char, and value is true only at station, false otherwise
stoppedFor values [0,1,2,3,4,5] in char, and this shall take value only at station (that too remains fixed once movedAfterStop), 0 otherwise
*/

#define stationArrived 0
#define movedAfterStop 1
#define stoppedFor 2

// when a state action pair is inserted it will always be inserted as state, stop and state, move
// qTable structure -> state will be key, action and float value
struct __attribute__((packed)) qTableEntry{
    char state[stateRepresentationSize];
    char action;
    float value;
};
struct qTableEntry qTable[maxEntries];  
int qTableSize = 0;


void printStr(char state[stateRepresentationSize]){
  for(int j=0;j<stateRepresentationSize ;++j){
    Serial.print(state[j]);
  }
}

void copyState(char state[stateRepresentationSize], char nextState[stateRepresentationSize]){
  for(int j=0;j<stateRepresentationSize;++j){
    state[j] = nextState[j];
  }
}

void keepWaiting(){
  	while (true) {
  	}
}

int checkIfPresentInQTable(char state[stateRepresentationSize]){
    int matchFound;
    for(int i=0;i<qTableSize;i = i+2){
        matchFound = 1;
        for(int j=0;j<stateRepresentationSize;++j){
            if(state[j] != qTable[i].state[j]){
                matchFound = 0;
                break;
            }
        }
        if(matchFound == 1){
            return i;
        }
    }
    return -1;
}

int insertInQTable(char state[stateRepresentationSize]){
    for(int j=0;j<stateRepresentationSize;++j){
        qTable[qTableSize].state[j] = state[j];
    }
    qTable[qTableSize].action = actionStop;
    qTable[qTableSize].value = 0;
    qTableSize++;
    for(int j=0;j<stateRepresentationSize;++j){
        qTable[qTableSize].state[j] = state[j];
    }
    qTable[qTableSize].action = actionMove;
    qTable[qTableSize].value = 0;
    qTableSize++;
    return qTableSize-2;
}

void updateQTable(char state[stateRepresentationSize], char action, char nextState[stateRepresentationSize], float reward){
    int index = checkIfPresentInQTable(state);
    if(index == -1){
        // insert in QTable
        index = insertInQTable(state);
    }

    float maxQANextState = 0;
    int nextIndex = checkIfPresentInQTable(nextState);
    if(nextIndex == -1){
        // insert in QTable
        nextIndex = insertInQTable(nextState);
    }

    if(qTable[nextIndex].value>=qTable[nextIndex+1].value){
        maxQANextState = qTable[nextIndex].value;
    }else{
        maxQANextState = qTable[nextIndex+1].value;
    }

    if(action == actionMove){
        index++;
    }

    qTable[index].value = (1-learningRate)*qTable[index].value + learningRate*(reward + discount*maxQANextState);

}

char getBestAction(char state[stateRepresentationSize]){
    int index = checkIfPresentInQTable(state);
    if(index == -1){
        index = insertInQTable(state);
    }
    
    // tie breaker is actionStop
    if(qTable[index].value>=qTable[index+1].value){
        Serial.println("We got stop because of tie breaker");
        return actionStop;
    }
    return actionMove;
}

char ultrasonicSensorValueToIndex(long sensorValue){
    if(sensorValue < closeStationThresholdLowForEnvironment){
        return closeSensorValue;
    }
    if(sensorValue < closeStationThresholdHighForEnvironment){
        return moderateSensorValue;
    }
    return farSensorValue;
}

void getInitialState(long ultrasonicSensorLeftValue, long ultrasonicSensorRightValue, char state[stateRepresentationSize]){
    
    char leftSensorIndex = ultrasonicSensorValueToIndex(ultrasonicSensorLeftValue);
    char rightSensorIndex = ultrasonicSensorValueToIndex(ultrasonicSensorRightValue);
    char highThresholdIndex = ultrasonicSensorValueToIndex(closeStationThresholdHighForEnvironment);
    if(min(leftSensorIndex, rightSensorIndex) < highThresholdIndex){
    	// handle the case for initially at station	
		state[stationArrived] = true;    
    }else{
		state[stationArrived] = false;    
    }
    // assume the bot was moving previously, not stopped, so moveAfterStop is not false
    state[movedAfterStop] = false;		
    state[stoppedFor] = '0';     
}

void handleStationArrived(long ultrasonicSensorLeftValue, long ultrasonicSensorRightValue, char state[stateRepresentationSize], char action, char nextState[stateRepresentationSize]){

	// left and right sensor value only needed to get other useful information, and need not be stored in state
	char leftSensorIndex = ultrasonicSensorValueToIndex(ultrasonicSensorLeftValue);
    char rightSensorIndex = ultrasonicSensorValueToIndex(ultrasonicSensorRightValue);
    char highThresholdIndex = ultrasonicSensorValueToIndex(closeStationThresholdHighForEnvironment);
    if(min(leftSensorIndex, rightSensorIndex) < highThresholdIndex){
    	nextState[stationArrived] = true;
    }else{
    	nextState[stationArrived] = false;
    }
    
}

void handleMovedAfterStop(char state[stateRepresentationSize], char action, char nextState[stateRepresentationSize]){
	if(nextState[stationArrived] == false){
		// false if not at station
		nextState[movedAfterStop] = false;
	}else{
		// at station
		if(state[movedAfterStop] == true){
			// if once moved after stop, it will remain same throughout the station, and will change when leaving the station to false, which is handled in if case above nesting
			nextState[movedAfterStop] = true;	
		}else{
			// at station, and stopped continuously or never stopped
			if(action == actionMove){
				// at station, and moving
				if(state[stoppedFor]>'0'){
					// means stopped at station continuously before (that's why movedAfterStop is false but stoppedFor > '0')
					nextState[movedAfterStop] = true;
				}else{
					// means never stopped at station (that's why movedAfterStop is false but stoppedFor == '0')
					nextState[movedAfterStop] = false;
				}
			}else{
				// at station, took stop and did not moveAfterStop so either stopping for first time or stopping in first chunk
				nextState[movedAfterStop] = false;
			}
		}
	}
}

void handleStoppedFor(char state[stateRepresentationSize], char action, char nextState[stateRepresentationSize]){
	if(nextState[stationArrived] == false){
		nextState[stoppedFor] = '0';
	}else{
		// at station
		if(nextState[movedAfterStop] == true){
			// moved after stop, so stopped for remains same for entire station
			nextState[stoppedFor] = state[stoppedFor];
		}else{
			// not moved after stop, but at station, so either never stopped or continue stopped
			if(action == actionStop){
				// at station, not moved after stop, so either never stopped or stopped continuously
				nextState[stoppedFor] = min(1+state[stoppedFor],maxStopTime); 
			}else{
				// at station, not moved after stop, so never stopped only, can make directly '0'
				nextState[stoppedFor] = '0';
			}
		}
	}
}

void getRewardNextState(long ultrasonicSensorLeftValue, long ultrasonicSensorRightValue, char state[stateRepresentationSize], char action, char nextState[stateRepresentationSize], float* reward){

    handleStationArrived(ultrasonicSensorLeftValue, ultrasonicSensorRightValue, state, action, nextState);
    handleMovedAfterStop(state, action, nextState);
    handleStoppedFor(state, action, nextState);
    
    // give reward
    *reward = 0;
    if(state[stationArrived] == false){
    	// if not at station then must move and not stop
    	if(action == actionStop){
    		*reward = *reward - 1;
    	}else{
    		*reward = *reward + 1;
    	}
    }else{
    	if(state[movedAfterStop] == true){
    		// if at station but moved after stopping then must move and not stop
    		if(action == actionStop){
    			*reward = *reward - 1;
    		}else{
    			*reward = *reward + 1;
    		}
    	}else{
    		// if at station but not moved after stop, then either never stopped or stopped continously
    		if(state[stoppedFor] == maxStopTime){
    			// if at station but not moved after stop, and stopped continously, then must move and not stop
    			if(action == actionStop){
    				*reward = *reward - 1;
    			}else{
    				*reward = *reward + 1;
    			}
    		}else{
    			// if at station but not moved after stop, then either never stopped or stopped for short time, so must not move and stop
    			if(action == actionStop){
    				*reward = *reward + 1;
    			}else{
    				*reward = *reward -1;
    			}
    		}
    	}
    }
}

char act(char state[stateRepresentationSize]){
    // Generate a random action: 0 for stop, 1 for move
    // Generates a random integer in entire range
    int rnd = random(100);
    float r = (float)rnd/(100);
    if(r < 0){ 
      r = r + 1;
      Serial.println("error r is -ve overflow");
    }
    // code change begins
    if(r < epsilon){
        rnd = random(0, 2);
        if(rnd == 0){
          Serial.print("The value of r is:");
          Serial.println(r);
          Serial.println("Are we here by chance?");
            return actionStop;
        }
        return actionMove;
    }
    // code change ends
    char bestAction = getBestAction(state);
    return bestAction;
}

