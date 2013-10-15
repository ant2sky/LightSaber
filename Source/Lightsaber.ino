#include <Wire.h>

//===== System Mode =====
enum {
	SYSTEM_OFF		= 0,
	SYSTEM_ON,
	SYSTEM_MODE_1,
	SYSTEM_MODE_2,
	
	TOTAL_SYSTEM_MODE
} SystemMode_t;

int currentIndex = 0;
int currentTime = 0;
int changeMode = 0;

//===== Polling Time ====
#define SYSTEM_POLLING_TIME	1

//===== Array Led ====
#define LED_ON	HIGH
#define LED_OFF	LOW

enum {
	LED_MODE_BLINK	= 0,
	LED_MODE_GRADUAL,
	LED_MODE_MULTI_GRADUAL,
	
	TOTAL_LED_MODE
} LedMode_t;

#define ARRAY_LED_NUM 7
#define PWD_LED_PIN	4
int ArrayLed[ARRAY_LED_NUM] = {4, 3, 5, 6, 9, 10, 11};


//===== Audio Module =====
#define AUDIO_RESET_PIN	13
#define AUDIO_MODE_PIN_1	1
#define AUDIO_MODE_PIN_2	7
#define AUDIO_MODE_PIN_3	8
#define AUDIO_MODE_PIN_4	12
enum {
	AUDIO_MODE_OFF	= 0,
	AUDIO_MODE_1,
	AUDIO_MODE_2,
	AUDIO_MODE_3,
	AUDIO_MODE_4,
	
	TOTAL_AUDIO_MODE
} AudioMode_t;

//===== Button =====
#define BUTTON_PIN	2
int previousButtonState = HIGH;
int buttonCounter = 0;

//===== Accelerometer =====
#define Register_ID 0
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
#define ADXAddress  (0xA7 >> 1)
int reading = 0;
int val=0;
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
double Xg,Yg,Zg;
#define ACCELEROMETER_SENSOR_DELAY 50
int accelerometerCount = 0;


//===== Vibratormeter =====
#define VIBRATORMETER_PIN	0
#define VIBRATORMETER_SENSOR_DELAY 50
#define VIBRATORMETER_DEBOUNCING_DELAY 5
int vibratormeterDebouncing = 0;
int vibratormeterCount = 0;

#define LED_ON_VALUE 255
#define LED_OFF_VALUE 100
#define LED_SC3_INTER_VALUE 10
#define LED_SC3_CONTINUANCE_VALUE	10
void LED_Scenario_2(int * currentIndex, int * currentTime)
{
	int currentValue;
	int totalTime;
	
	totalTime = ((LED_ON_VALUE - LED_OFF_VALUE) / LED_SC3_INTER_VALUE) * LED_SC3_CONTINUANCE_VALUE;
	
	switch((*currentIndex))
	{
	case 0:
		// HIGH --> LOW
		currentValue = LED_ON_VALUE - (LED_SC3_INTER_VALUE * (*currentTime / LED_SC3_CONTINUANCE_VALUE));
		break;
	case 1:
		// LOW --> HIGH
		currentValue = LED_OFF_VALUE + (LED_SC3_INTER_VALUE * (*currentTime / LED_SC3_CONTINUANCE_VALUE));
		break;
	}
	
	for(int index = 0; index < ARRAY_LED_NUM; index++)
	{
		analogWrite(ArrayLed[index], currentValue);
	}
	
	(*currentTime) += SYSTEM_POLLING_TIME;

	if(*currentTime >= totalTime)
	{
		*currentTime = 0;
		(*currentIndex)++;
		if((*currentIndex) >= 2) (*currentIndex) = 0;
	}
}

#define LED_SC1_INTER_VALUE 5
#define LED_SC1_CONTINUANCE_VALUE	5
void LED_Scenario_1(int * currentIndex, int * currentTime)
{
	int currentValue;
	int totalTime;
	
	totalTime = ((LED_ON_VALUE - LED_OFF_VALUE) / LED_SC1_INTER_VALUE) * LED_SC1_CONTINUANCE_VALUE;
	
	switch((*currentIndex))
	{
	case 0: case 1:	case 2:	case 3:	case 4:	case 5:
		// HIGH --> LOW
		currentValue = LED_ON_VALUE - (LED_SC1_INTER_VALUE * (*currentTime / LED_SC1_CONTINUANCE_VALUE));
		analogWrite(ArrayLed[*currentIndex], currentValue);
		break;
	case 6: case 7: case 8: case 9: case 10: case 11:
		// LOW --> HIGH
		currentValue = LED_OFF_VALUE + (LED_SC1_INTER_VALUE * (*currentTime / LED_SC1_CONTINUANCE_VALUE));
		analogWrite(ArrayLed[((*currentIndex) - 6)], currentValue);
		break;
	}
	
	
	(*currentTime) += SYSTEM_POLLING_TIME;

	if(*currentTime >= totalTime)
	{
		*currentTime = 0;
		(*currentIndex)++;
		if((*currentIndex) >= 12) (*currentIndex) = 0;
	}
}

#define LED_SC_ON_CONTINUANCE_VALUE	100
void LED_Scenario_on(int * currentIndex, int * currentTime)
{
	if((*currentIndex) >= ARRAY_LED_NUM)
		return;

	if(changeMode == 1)
	{
		digitalWrite(PWD_LED_PIN, LED_ON);
		for(int index = 0; index < ARRAY_LED_NUM; index++)
			digitalWrite(ArrayLed[index], LED_OFF);
	}

	(*currentTime) += SYSTEM_POLLING_TIME;

	if(*currentTime >= LED_SC_ON_CONTINUANCE_VALUE)
	{
		digitalWrite(ArrayLed[*currentIndex], LED_ON);
		*currentTime = 0;
		(*currentIndex)++;
	}
}

#define LED_SC_OFF_CONTINUANCE_VALUE	100
void LED_Scenario_off(int * currentIndex, int * currentTime)
{
	if((*currentIndex) >= ARRAY_LED_NUM) 
	{
		digitalWrite(PWD_LED_PIN, LED_OFF);
		return;
	}

	if(changeMode == 1)
	{
		for(int index = 0; index < ARRAY_LED_NUM; index++)
			digitalWrite(ArrayLed[index], LED_ON);
	}
	
	(*currentTime) += SYSTEM_POLLING_TIME;

	if(*currentTime >= LED_SC_OFF_CONTINUANCE_VALUE)
	{
		digitalWrite(ArrayLed[ARRAY_LED_NUM - (*currentIndex) - 1], LED_OFF);
		*currentTime = 0;
		(*currentIndex)++;
	}
}

int readButtonState()
{
	int ret = 0;
	
	int buttonState = digitalRead(BUTTON_PIN);   // read the pushbutton:
	if ((buttonState != previousButtonState)    // if the button state has changed, 
		&& (buttonState == HIGH)) {            // and it's currently pressed:
		ret = 1;
	}
	// save the current button state for comparison next time:
	previousButtonState = buttonState; 
	
	return ret;
}

void playAudio(int mode)
{
	digitalWrite(AUDIO_RESET_PIN, LOW);
	digitalWrite(AUDIO_MODE_PIN_1, HIGH);
	digitalWrite(AUDIO_MODE_PIN_2, HIGH);
	digitalWrite(AUDIO_MODE_PIN_3, HIGH);
	digitalWrite(AUDIO_MODE_PIN_4, HIGH);
	
	switch(mode)
	{
	case AUDIO_MODE_1:
		digitalWrite(AUDIO_MODE_PIN_1, LOW);
		break;
	case AUDIO_MODE_2:
		digitalWrite(AUDIO_MODE_PIN_2, LOW);
		break;
	case AUDIO_MODE_3:
		digitalWrite(AUDIO_MODE_PIN_3, LOW);
		break;
	case AUDIO_MODE_4:
		digitalWrite(AUDIO_MODE_PIN_4, LOW);
		break;
	}
	digitalWrite(AUDIO_RESET_PIN, HIGH);
}

void initAccelerometerSensor()
{
	Wire.begin();
	delay(100);
	Wire.beginTransmission(ADXAddress);
	Wire.write(Register_2D);
	Wire.write(8);                //measuring enable
	Wire.endTransmission();     // stop transmitting
	
	//Serial.begin(115200); 
}

void getGData()
{
	//--------------X
	Wire.beginTransmission(ADXAddress); // transmit to device
	Wire.write(Register_X0);
	Wire.write(Register_X1);
	Wire.endTransmission();
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
	X0 = Wire.read();
	X1 = Wire.read(); 
	X1=X1<<8;
	X_out=X0+X1;   
	}

	//------------------Y
	Wire.beginTransmission(ADXAddress); // transmit to device
	Wire.write(Register_Y0);
	Wire.write(Register_Y1);
	Wire.endTransmission();
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
	Y0 = Wire.read();
	Y1 = Wire.read(); 
	Y1=Y1<<8;
	Y_out=Y0+Y1;
	}
	//------------------Z
	Wire.beginTransmission(ADXAddress); // transmit to device
	Wire.write(Register_Z0);
	Wire.write(Register_Z1);
	Wire.endTransmission();
	Wire.requestFrom(ADXAddress,2); 
	if(Wire.available()<=2)   
	{
	Z0 = Wire.read();
	Z1 = Wire.read(); 
	Z1=Z1<<8;
	Z_out=Z0+Z1;
	}

	//display the real value
	Xg=X_out/256.0;
	Yg=Y_out/256.0;
	Zg=Z_out/256.0;
	
	/*
	Serial.print("X= ");
	Serial.print(Xg);
	Serial.print("       ");
	Serial.print("Y= ");
	Serial.print(Yg);
	Serial.print("       ");
	Serial.print("Z= ");
	Serial.print(Zg);
	Serial.println("  ");
	*/	
}


void setup() 
{  
	pinMode(BUTTON_PIN, INPUT);
	pinMode(VIBRATORMETER_PIN, INPUT);
	
	pinMode(PWD_LED_PIN, OUTPUT);
	for(int index = 0; index < ARRAY_LED_NUM; index++)
		pinMode(ArrayLed[index], OUTPUT);
		
	pinMode(AUDIO_MODE_PIN_1, OUTPUT);
	pinMode(AUDIO_MODE_PIN_2, OUTPUT);
	pinMode(AUDIO_MODE_PIN_3, OUTPUT);
	pinMode(AUDIO_MODE_PIN_4, OUTPUT);
	pinMode(AUDIO_RESET_PIN, OUTPUT);

	// initial Led
	digitalWrite(PWD_LED_PIN, LED_OFF);
	for(int index = 0; index < ARRAY_LED_NUM; index++)
		digitalWrite(ArrayLed[index], LED_OFF);

	// initial audio pin
	digitalWrite(AUDIO_MODE_PIN_1, HIGH);
	digitalWrite(AUDIO_MODE_PIN_2, HIGH);
	digitalWrite(AUDIO_MODE_PIN_3, HIGH);
	digitalWrite(AUDIO_MODE_PIN_4, HIGH);
        digitalWrite(AUDIO_RESET_PIN, HIGH);

	// initial accelerometer sensor
	initAccelerometerSensor();
}

void loop()
{
	int scenarioIndex = 0;
	
	// Read button state
	if (readButtonState() == 1)
	{
		buttonCounter++;     
		// increment the button counter
		if(buttonCounter >= TOTAL_SYSTEM_MODE) buttonCounter = 0;
		
		currentIndex = 0;
		currentTime = 0;
		changeMode = 1;
		vibratormeterCount = 0;
		accelerometerCount = 0;
	}
	
	// Scenario Loop
	switch(buttonCounter)
	{
	case SYSTEM_OFF:
		LED_Scenario_off(&currentIndex, &currentTime);
		if (changeMode == 1) playAudio(AUDIO_MODE_1);
		break;
	case SYSTEM_ON:
		LED_Scenario_on(&currentIndex, &currentTime);
		if (changeMode == 1) playAudio(AUDIO_MODE_2);
		break;
	case SYSTEM_MODE_1:
		LED_Scenario_1(&currentIndex, &currentTime);
		break;
	case SYSTEM_MODE_2:
		LED_Scenario_2(&currentIndex, &currentTime);
		break;
	}
	
	// Accelerometer sensor
	if(buttonCounter != SYSTEM_OFF)
	{
		accelerometerCount++;
		if(accelerometerCount > ACCELEROMETER_SENSOR_DELAY)
		{
			getGData();
			if((Xg > 1) || (Yg > 1) || (Zg > 1))
			{
				playAudio(AUDIO_MODE_4);
				
				vibratormeterCount = 0;
				vibratormeterDebouncing = 0;
			}
			accelerometerCount = 0;
		}
	}

	// Vibratormeter sensor
	if(buttonCounter != SYSTEM_OFF)
	{
		vibratormeterCount++;
		if((vibratormeterCount > VIBRATORMETER_SENSOR_DELAY) || vibratormeterDebouncing != 0)
		{
			if(digitalRead(VIBRATORMETER_PIN) == 1)
			{
				if(vibratormeterDebouncing > VIBRATORMETER_DEBOUNCING_DELAY)
				{
					playAudio(AUDIO_MODE_3);
					vibratormeterDebouncing = 0;
				}
				
				vibratormeterDebouncing++;
			}
			else
			{
				vibratormeterCount = 0;
			}
		}
	}
		
	changeMode = 0;
	
	delay(SYSTEM_POLLING_TIME);
}

//=========================================================
