//Set this to 1 if you want to see debug
#define DEBUG 0

//Sensors
#define NUM_Sensors 3
const int trigPin[NUM_Sensors] = {5, 4, 2};
const int echoPin = 18;
const int push_button = 21;

//define sound speed in cm/uS
#define SOUND_SPEED 0.0344
#define CM_TO_INCH 0.393701

//For motor and PWM generation
#define NUM_Motors 3
const int motor_pin[NUM_Motors] = {33, 32, 19};
volatile int PWM_Ch[NUM_Motors];
//#define motor 19
//#define PWM1_Ch    0
#define PWM_Res   8
#define PWM_Freq  5000

//For Motor Vibration
#define detection_threshold 90.0
#define intensity 1.0
#define Duty_factor detection_threshold + 100.0

//Filter coefficients
#define refresh_freq 100.0
#define pi 3.14285714286 
#define fs 100.0
#define coeff 2/(1/fs)
#define frequency 20.0
#define cutoff  frequency*2*pi

float a1 =  (coeff - cutoff)/(coeff + cutoff);
float b0 =  cutoff/(coeff+cutoff);
float b1 = cutoff/(coeff+cutoff);
//y[n] = a1*y[n-1] + b0*x[n] + b1*x[n-1]  <-------------- Difference Equation 

void setup() {
  int i = 0;
  //initializing the PWM Channels for the motors. No. of channels is based on the the number of motors attached. MAX = 16
  for(i = 0; i < NUM_Motors; i++)
  {
    PWM_Ch[i] = i;
  }
  
  // Starts the serial communication
  Serial.begin(115200); 
  
  //Setting up all the triger Pins for the ultrasonic sensor
  for(i = 0; i < NUM_Sensors; i++)
  {
    pinMode(trigPin[i], OUTPUT); // Sets the corresponding trigPin for sensor as an Output
  }
  
  pinMode(echoPin, INPUT); // Sets the sensor echoPin as an Input, here all sensors are having a same echo pin. The cost is that we cannot take readings simultanoeusly.
  pinMode(push_button, INPUT); //Sets the push buton pin as Input
  
  //Set up PWM for Motor Driver
  //We use a for loop to set up the motor driving channels. The pins are prescpecified in the top of the file.
  for(i  = 0; i < NUM_Motors; i++)
  {
    ledcSetup(PWM_Ch[i], PWM_Freq, PWM_Res);   //First we make a channel for the motor.
    ledcAttachPin(motor_pin[i], PWM_Ch[i]);    //Then we add the pin that drives the motor controller. 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  long time_start, time_stop; 
  unsigned duration;
  float distanceCm[NUM_Sensors];
  float y[NUM_Sensors], y_prev[NUM_Sensors], x_prev[NUM_Sensors];
  float y2n = 0.0, y22 = 0.0, x22 = 0.0, yn = 0.0, y1 = 0.0, x1 = 0.0;
  int duty_cycle[NUM_Motors];
  int i = 0;

  char output[100];
  
  //Initialize the data structures
  Serial.print("\nInitalizing distance array\n");
  initialize(distanceCm, NUM_Sensors);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  //Move to next step

  Serial.print("\nInitalizing Y array\n");
  initialize(y, NUM_Sensors);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  //Move to next step  

  Serial.print("\nInitalizing Y_prev array\n");
  initialize(y_prev, NUM_Sensors);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  //Move to next step    

  Serial.print("\nInitalizing X_prev array\n");
  initialize(x_prev, NUM_Sensors);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  //Move to next step
  
  Serial.print("\nInitalizing Duty_Cycle array\n");
  initialize_int(duty_cycle, NUM_Motors);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  
  //Main looping loop
  while(1)
  {
    //Take sensor readings and compare if the we need to turn on the motor.
    for(i = 0; i < NUM_Sensors; i++)
    {
      // Clears the trigPin
      digitalWrite(trigPin[i], LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigPin[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin[i], LOW);
   
     /* time_start = micros();
      while(!digitalRead(echoPin))
      {
        if((micros() - time_start) > 15000)
        {
          break;
        }
      }
      time_stop = micros();
      duration = time_stop - time_start;
      */
      // Reads the echoPin, returns the sound wave travel time in microseconds. Not very effective :(
      duration = pulseIn(echoPin, HIGH);
      
      // Calculate the distance
      distanceCm[i] = duration * SOUND_SPEED/2;
      if(distanceCm[i] > 250.00)
      {
        distanceCm[i] = x_prev[i];
      }
      //Calculate Output usign Low Pass filter
      y[i] = a1*y_prev[i] + b0*distanceCm[i] + b1*x_prev[i];
      
      //Now update the previous values
      y_prev[i] = y[i];
      x_prev[i] = distanceCm[i];
      //Give output for checking. Disabled of DEBUG in the top of the file is not equal to 1.
      if(DEBUG == 1)
      {
        sprintf(output, "\ni = %d, duration = %d, xn = %.4f, x_prev = %.4f, yn = %.4f, y_prev = %.4f\n", i, duration, distanceCm[i], x_prev[i], y[i], y_prev[i]);
        Serial.print(output); 
      }
    }
     
    // now print the distance in the Serial Monitor
    Serial.print("\n");
    for(i = 0; i < NUM_Sensors; i++)
    {
      if(DEBUG == 1)
      {
        sprintf(output, "Sensor[%d]: %.4f\t", i, y[i]);
        Serial.print(output);
      }
      sprintf(output, "%.4f \t", y[i]);
      Serial.print(output);
    }
    Serial.print("\n");
   
    //Now if Distance is less than the minimum specified in the top, then we start the motor for that corresponding sensor. The intensity increases as
    //Distance becomes smaller and smaller.
    for(i = 0; i < NUM_Sensors; i++)
    {
      if(y[i] <= detection_threshold)
      {
        duty_cycle[i] = 100 - (int)(y[i] * intensity);
        ledcWrite(PWM_Ch[i], duty_cycle[i]);
        sprintf(output, "Duty_Cycle[%i]: %i\t", i, duty_cycle[i]);
        Serial.print(output);
      }
      else
      {
        duty_cycle[i] = 0;
        ledcWrite(PWM_Ch[i], duty_cycle[i]);
      }
    }
    Serial.print("\n"); 
    delay((1/refresh_freq)*1000);
  }
}

void initialize(float *input, int width)
{
  for(int i = 0; i < width; i++)
  {
    input[i] = 0.0;
    Serial.print(input[i],2);
    Serial.print("\t");
  }

  Serial.print("\n----Initialized-------\n");
  delay(200);
}

void initialize_int(int *input, int width)
{
  for(int i = 0; i < width; i++)
  {
    input[i] = 0.0;
    Serial.print(input[i],2);
    Serial.print("\t");
  }

  Serial.print("\n----Initialized-------\n");
  delay(200);
}
