//Set this to 1 if you want to see debug
#define DEBUG 0

//Wifi Settings
#include <WiFi.h>
const char* ssid = "Spidey_V1";
const char* password = "BeGreater";
// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;


//Sensors
#define NUM_Sensors 3
const uint8_t trigPin[NUM_Sensors] = {5, 4, 2};
const uint8_t echoPin = 18;
const uint8_t push_button = 21;

//define sound speed in cm/uS
#define SOUND_SPEED 0.0344

//For motor and PWM generation
#define NUM_Motors 3
const uint8_t motor_pin[NUM_Motors] = {33, 32, 19};
volatile uint8_t PWM_Ch[NUM_Motors];
#define PWM_Res   8
#define PWM_Freq  5000

//For Motor Vibration
#define detection_threshold 100.0
#define NUM_Region 4
#define intensity 1.0
#define motor_intensity 100
const uint8_t motor_rate[NUM_Region] = {50, 30, 20, 10};

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

//Timer configuration
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//MOTOR PULSE GENERATION 
#define prescalar 80
#define timer_rate_ms 20
#define timer_value 80000000/prescalar * timer_rate_ms/1000
#define initial_pulse_rate 80
volatile uint8_t master_motor_control[NUM_Motors];
volatile uint8_t motor_time_counter[NUM_Motors];
volatile uint8_t motor_time_threshold[NUM_Motors];
volatile uint8_t motor_running[NUM_Motors];
volatile uint8_t enable_motor[NUM_Motors];

//Timer Interrupt Routine.
void IRAM_ATTR onTimer()
{
  //increment the counter and set time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  
  //Increment the counter
  for(int i = 0; i < NUM_Motors; i++)
  {
    motor_time_counter[i]++;
    //Check if the main loop wants to enable the ith motor.
    if(enable_motor[i] == 1 && master_motor_control[i] == 1)
    {
      //If its is time to switch on/off the motor(s)
      if(motor_time_counter[i] > (motor_time_threshold[i] - 1))
      {
        //Reset the particular counter
        motor_time_counter[i] = 0;
      
        //if its running then turn it OFF.
        if(motor_running[i] == 1)
        {
          ledcWrite(PWM_Ch[i], 0);
          motor_running[i] = 0;
        }
        //otherwise turn it ON.
        else
        {
          ledcWrite(PWM_Ch[i], motor_intensity);
          motor_running[i] = 1;
        }    
      }
    }
    else
    {
      //Stop the motor immediately
      ledcWrite(PWM_Ch[i], 0);
      motor_running[i] = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  
  uint8_t i = 0;

  //--------------Starts the serial communication--------------------
  Serial.begin(115200); 

  //--------------Set Up Wifi----------------------------------------
  Serial.print("Setting AP (Access Point)...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\nAP IP address: ");
  Serial.println(IP);
  server.begin();
  
  //--------------Ultrasonic Sensor Initialization-------------------
  //Setting up all the triger Pins for the ultrasonic sensor
  for(i = 0; i < NUM_Sensors; i++)
  {
    pinMode(trigPin[i], OUTPUT); // Sets the corresponding trigPin for sensor as an Output
  }
  pinMode(echoPin, INPUT); // Sets the sensor echoPin as an Input, here all sensors are having a same echo pin. The cost is that we cannot take readings simultanoeusly.
  pinMode(push_button, INPUT); //Sets the push buton pin as Input
  
  //--------------Set up PWM for Motor Driver-----------------------
  //initializing the PWM Channels for the motors. No. of channels is based on the the number of motors attached. MAX = 16
  for(i = 0; i < NUM_Motors; i++)
  {
    PWM_Ch[i] = i;
  }
  //We use a for loop to set up the motor driving channels. The pins are prescpecified in the top of the file.
  for(i  = 0; i < NUM_Motors; i++)
  {
    ledcSetup(PWM_Ch[i], PWM_Freq, PWM_Res);   //First we make a channel for the motor.
    ledcAttachPin(motor_pin[i], PWM_Ch[i]);    //Then we add the pin that drives the motor controller. 
  }

  //---------------Initialize the timers-----------------------------
  //Using timer 0, prescalar is 80, so 1MHz frequency. We want interrupts at every Xms
  timer = timerBegin(0, prescalar, true);

  //attach the onTimer interrupt function to our timer
  timerAttachInterrupt(timer, &onTimer, true);

  //Now give the number st that the timer repeats after Xms, true means it auto reloads
  timerAlarmWrite(timer, timer_value, true);

  //Start the timer
  timerAlarmEnable(timer);
}

void loop() {
  //--------------------Variables-----------------------------
  long time_start, time_stop; 
  unsigned duration;
  float distanceCm[NUM_Sensors];
  float y[NUM_Sensors], y_prev[NUM_Sensors], x_prev[NUM_Sensors];
  float y2n = 0.0, y22 = 0.0, x22 = 0.0, yn = 0.0, y1 = 0.0, x1 = 0.0;
  float detection_region[NUM_Region];
  uint8_t i = 0;
  char output[100];   //Used to output data to serial monitor.

  //------------Initialize the data structures----------------
  Serial.print("\nInitalizing distance array\n");
  initialize(distanceCm, NUM_Sensors);
  //Wait for user to press button
  //Move to next step
  Serial.print("\nInitalizing Y array\n");
  initialize(y, NUM_Sensors);
  //Move to next step  
  Serial.print("\nInitalizing Y_prev array\n");
  initialize(y_prev, NUM_Sensors);
  //Move to next step    
  Serial.print("\nInitalizing X_prev array\n");
  initialize(x_prev, NUM_Sensors);
  //Move to next step
  Serial.print("\nInitalizing motor_time_counter array\n");
  initialize_int(motor_time_counter, NUM_Motors, 0);
  //Move to next step
  Serial.print("\nInitalizing motor_running array\n");
  initialize_int(motor_running, NUM_Motors, 0);  
  //Move to next step
  Serial.print("\nInitalizing enable_motor array\n");
  initialize_int(enable_motor, NUM_Motors, 0);
  Serial.print("\nInitalizing motor_time_threshold\n");
  initialize_int(motor_time_threshold, NUM_Motors, initial_pulse_rate);
  Serial.print("\nInitalizing motor_pulse_regions\n");
  initialize_regions(detection_region, NUM_Region);
  Serial.print("\nInitalizing master_motor_control\n");
  initialize_int(master_motor_control, NUM_Motors, 1);
  //Wait for user to press button
  Serial.print("\nPress Button to continue...\n");
  if(DEBUG == 1)
  {
    while(digitalRead(push_button) == LOW);
    while(digitalRead(push_button) == HIGH); 
  }
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\nAP IP address: ");
  Serial.println(IP);
  Serial.print("\nAP IP address: ");
  Serial.println(IP);
  while(digitalRead(push_button) == LOW);
  while(digitalRead(push_button) == HIGH);
  //-----------------Main looping loop----------------------------
  while(1)
  {
    //Call web server stuff
    WIFIstuff();
    //------------------READ SENSOR-------------------------------
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
      //sprintf(output, "%.4f \t", y[i]);
      //Serial.print(output);
    }
    Serial.print("\n");
   
    //Now if Distance is less than the minimum specified in the top, then we start the motor for that corresponding sensor. The intensity increases as
    //Distance becomes smaller and smaller.
    for(i = 0; i < NUM_Sensors; i++)
    {
      if(master_motor_control[i] == 1)    //Check if our motor is supposed to be running
      {
        if(y[i] <= detection_threshold)
        {
          //Now we set the pulsing rate based on the distance
          for(int j = 0; j < NUM_Region; j++)
          {
           if(y[i] >= detection_region[j])
            {
              motor_time_threshold[i] = motor_rate[j];
              //sprintf(output, "\n Motor[%i] Detction_Rate = %i, region = %f\n", i, motor_time_threshold[i], detection_region[j]);
              //Serial.print(output);
              break;
            }
          }
          //Start the motor
          enable_motor[i] = 1;
        }
        else
        {
          //Stop the motor
         enable_motor[i] = 0;
        }
      }
    }
    Serial.print("\n"); 
    //ONLY WHEN WE WANT TO DEBUG
    if(digitalRead(push_button) == HIGH && DEBUG == 1)
    {
      while(digitalRead(push_button) == HIGH);
      switch(motor_time_threshold[0])
      {
        case 2:
          motor_time_threshold[0] = 5;
          Serial.print("\nMotor now set to 5\n");
          break; 
        case 5:
          motor_time_threshold[0] = 8;
          Serial.print("\nMotor now set to 8\n");
          break; 
        case 8:
          motor_time_threshold[0] = 10;
          Serial.print("\nMotor now set to 10\n");
          break; 
        case 10:
          motor_time_threshold[0] = 2;
          Serial.print("\nMotor now set to 2\n");
          break;  
        default:
          motor_time_threshold[0] = 2;
          Serial.print("\nMotor default to 2\n");
          break;         
      }
    }
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

void initialize_int(volatile uint8_t *input, int width, int seed)
{
  for(int i = 0; i < width; i++)
  {
    input[i] = seed;
    Serial.print(input[i],2);
    Serial.print("\t");
  }

  Serial.print("\n----Initialized-------\n");
  delay(200);
}

void initialize_regions(float *region, int divisions)
{
  float region_size = detection_threshold/divisions;
  for(int i = 0; i < divisions; i++)
  {
    if(i == 0)
    {
      region[i] = detection_threshold - region_size;  
      Serial.print(region[i],2);
      Serial.print("\t");
    }
    else
    {
      region[i] = region[i-1] - region_size;
      Serial.print(region[i],2);
      Serial.print("\t");
    }
  }  
  Serial.print("\n-----Initialized------\n");
}

void WIFIstuff()
{
  WiFiClient client = server.available();   //Listen for incoming clients
  if(client)
  {
    Serial.print("\nNew Client.");
    String currentLine = "";
    while(client.available())
    {
    if(client.available())
    {
      char c = client.read();
      Serial.print(c);
      header += c;
      if(c == '\n')
      {
        if(currentLine.length() == 0)
        {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println("Connection: close");
          client.println();
          // turns the GPIOs on and off
          if (header.indexOf("GET /M1/on") >= 0) 
          {
            Serial.println("MOTOR 1 OFF");
            master_motor_control[0] = 0;
          } 
          else if (header.indexOf("GET /M1/off") >= 0) 
          {
            Serial.println("MOTOR 1 ON");
            master_motor_control[0] = 1;
          } 
          else if (header.indexOf("GET /M2/on") >= 0) 
          {
            Serial.println("MOTOR 2 OFF");
            master_motor_control[1] = 0;
          } 
          else if (header.indexOf("GET /M2/off") >= 0) 
          {
            Serial.println("MOTOR 2 ON");
            master_motor_control[1] = 1;
          }
          else if (header.indexOf("GET /M3/off") >= 0) 
          {
            Serial.println("MOTOR 3 ON");
            master_motor_control[2] = 1;
          }
          else if (header.indexOf("GET /M3/on") >= 0) 
          {
            Serial.println("MOTOR 3 OFF");
            master_motor_control[2] = 0;
          } 
          //Display webpage
          client.println("<!DOCTYPE html><html>");
          client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
          client.println("<link rel=\"icon\" href=\"data:,\">");
          // CSS to style the on/off buttons 
          // Feel free to change the background-color and font-size attributes to fit your preferences
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
          client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
          client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
          client.println(".button2 {background-color: #555555;}</style></head>");
          // Web Page Heading
          client.println("<body><h1>Spidey_Sense_v1</h1>");
          if(master_motor_control[0] == 1){client.println("<p>Motor1 - State ON</p>");}
          else{client.println("<p>Motor 2 - State OFF</p>");}
          // If the output26State is off, it displays the ON button       
          if (master_motor_control[0] == 1) 
          {
            client.println("<p><a href=\"/M1/on\"><button class=\"button\">ON</button></a></p>");
          } 
          else 
          {
            client.println("<p><a href=\"/M2/off\"><button class=\"button button2\">OFF</button></a></p>");
          }
          if(master_motor_control[1] == 1){client.println("<p>Motor 2 - State ON</p>");}
          else{client.println("<p>Motor 2 - State OFF</p>");}
          if (master_motor_control[1] == 1) 
          {
            client.println("<p><a href=\"/M2/on\"><button class=\"button\">ON</button></a></p>");
          } 
          else 
          {
            client.println("<p><a href=\"/M2/off\"><button class=\"button button2\">OFF</button></a></p>");
          } 
          if(master_motor_control[2] == 1){client.println("<p>Motor 3 - State ON</p>");}
          else{client.println("<p>Motor 3 - State OFF</p>");}
          if (master_motor_control[2] == 1) 
          {
            client.println("<p><a href=\"/M3/on\"><button class=\"button\">ON</button></a></p>");
          } 
          else 
          {
            client.println("<p><a href=\"/M3/off\"><button class=\"button button2\">OFF</button></a></p>");
          } 
          client.println("</body></html>");  
          // The HTTP response ends with another blank line
          client.println();
          break;
        }
        else
        {
          currentLine = "";
        }
      }
      else if (c != '\r')
      {
        currentLine += c;
      }
    }
   }
  }
  //clear header variable
  header = "";
  //close the connection
  client.stop();
  Serial.println("Client Disconnected");
  Serial.println("");
}
