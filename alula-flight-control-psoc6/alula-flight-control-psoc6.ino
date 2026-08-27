/*******************************************************************************
* File Name        : main.ino
*
* Description      : This source file contains the main routine for the PSoC 6
*                    SoftAP-based UDP glider controller. It receives steering
*                    packets over Wi-Fi and drives two elevon servos via
*                    hardware PWM (TCPWM), with elevon mixing, smoothing and
*                    a connection-loss failsafe.
*
* Related Document : See README.md
*
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

/*******************************************************************************
 * Macros / Constants
 ******************************************************************************/
// WLAN access
const char* ssid     = NET_SECRET_SSID;
const char* password = NET_SECRET_PASSWORD;

// UDP settings
WiFiUDP udp;
unsigned int localPort = 5005;

// Hardware PWM servo config
const int SERVO_LEFT_PIN = 0;
const int SERVO_RIGHT_PIN = 1;
const int PWM_RESOLUTION_BITS = 16;
const long PWM_MAX_VALUE = 65535;     // 2^16 - 1
const int SERVO_FREQ_HZ = 50;        // standard servo frequency
const int SERVO_PERIOD_US = 20000;    // 20ms at 50Hz
const int SERVO_MIN_PULSE_US = 544;   // 0 degrees
const int SERVO_MAX_PULSE_US = 2400;  // 180 degrees

unsigned long lastServoUpdate = 0;
const int SERVO_INTERVAL = 50;

// Neutral positions — reference point for all calculations
const int NEUTRAL_ROLL = 90;   // roll neutral position (degrees)
const int NEUTRAL_PITCH = 90;  // pitch neutral position (degrees, slightly up for glider trim)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
// Shared state (updated from UDP packets)
volatile uint8_t target_roll  = NEUTRAL_ROLL;
volatile uint8_t target_pitch = NEUTRAL_PITCH; 
volatile bool    steer_active = false;

// Smoothing state
int last_left = NEUTRAL_ROLL;
int last_right = NEUTRAL_ROLL;
float smooth_pitch = NEUTRAL_PITCH;
float smooth_roll = NEUTRAL_ROLL;
int SERVO_LEFT_NEUTRAL = NEUTRAL_ROLL;
int SERVO_RIGHT_NEUTRAL = NEUTRAL_ROLL;

// Trimming to neutral
const int ROLL_TRIM = 0;
const int PITCH_TRIM = 0;

//Sensibility
const float PITCH_RATE = 0.3;
const float ROLL_RATE = 0.5;

//Failsafe variables
unsigned long lastPackageTime = millis();
const unsigned long TIMEOUT_CONNECTION_LOST = 1500;
bool hasEverReceived = false;

// Track previous steer_active to detect transitions
bool prev_steer_active = false;


/*******************************************************************************
* Function Name: servo_attach
********************************************************************************
* Summary:
*   Configures the given pin for 50Hz hardware PWM so a standard servo can be
*   driven directly by the TCPWM block without any CPU involvement.
*
* Parameters:
*  pin  GPIO pin number to attach the servo to.
*
* Return:
*  void
*
*******************************************************************************/
void servo_attach(int pin) {
    analogWriteResolution(PWM_RESOLUTION_BITS);
    setAnalogWriteFrequency(pin, SERVO_FREQ_HZ);
}

/*******************************************************************************
* Function Name: servo_write
********************************************************************************
* Summary:
*   Converts a servo angle in degrees into the matching PWM duty cycle and
*   writes it to the pin. Runs on the TCPWM hardware and is therefore immune
*   to Wi-Fi interrupt jitter.
*
* Parameters:
*  pin    GPIO pin the servo is attached to.
*  angle  Target angle in degrees (clamped to 0…180).
*
* Return:
*  void
*
*******************************************************************************/
void servo_write(int pin, int angle) {
    angle = constrain(angle, 0, 180);
    long pulse_us = SERVO_MIN_PULSE_US + (long)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180;
    long duty = pulse_us * PWM_MAX_VALUE / SERVO_PERIOD_US;
    analogWrite(pin, (int)duty);
}


/*******************************************************************************
* Function Name: elevon_mixing
********************************************************************************
* Summary:
*   Performs elevon mixing of the target pitch and roll commands and applies
*   an exponential interpolation for smooth servo motion. The mixed and
*   constrained left/right servo angles are returned via output pointers.
*
* Parameters:
*  target_pitch  Requested pitch angle in degrees (0…180, 90 = neutral).
*  target_roll   Requested roll angle in degrees (0…180, 90 = neutral).
*  out_left      Pointer that receives the resulting left servo angle.
*  out_right     Pointer that receives the resulting right servo angle.
*
* Return:
*  void
*
*******************************************************************************/
void elevon_mixing(int target_pitch, int target_roll, int* out_left, int* out_right){
    float interpolation = 0.80;

    //Absolute change from neutral
      float delta_pitch = (int)target_pitch - NEUTRAL_PITCH;
      float delta_roll = (int)target_roll - NEUTRAL_ROLL;
    //Scaling down movement
      float target_smooth_pitch = delta_pitch * ROLL_RATE;
      float target_smooth_roll  = delta_roll * PITCH_RATE;
    //Interpolation
      smooth_pitch += (target_smooth_pitch - smooth_pitch) * interpolation;
      smooth_roll += (target_smooth_roll - smooth_roll) * interpolation;
    //Mixing
      *out_left  = constrain((int)(SERVO_LEFT_NEUTRAL + smooth_pitch - smooth_roll), 60, 120);
      *out_right = constrain((int)(SERVO_RIGHT_NEUTRAL + smooth_pitch + smooth_roll), 60, 120);
}


/*******************************************************************************
* Function Name: Failsafe
********************************************************************************
* Summary:
*   Forces the glider into a safe recovery attitude by commanding full
*   up-elevator (both elevons up) with a slight roll. Called when the UDP
*   command link has been lost for longer than TIMEOUT_CONNECTION_LOST.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Failsafe(){
    target_roll  = 80;
    target_pitch = 110;
}


/*******************************************************************************
* Function Name: setup
********************************************************************************
* Summary:
*   Arduino one-shot init routine. Brings up the serial console, starts the
*   Wi-Fi SoftAP, attaches both servo outputs to hardware PWM and opens the
*   UDP listening socket used to receive steering commands.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("--- PSoC 6 SoftAP & UDP Server ---");

  // WiFi first
  WiFi.beginAP(ssid, password);
  delay(500);

  // Attach servos after WiFi — uses hardware TCPWM, immune to WiFi interrupts
  servo_attach(SERVO_LEFT_PIN);
  servo_attach(SERVO_RIGHT_PIN);

  Serial.print("AP active! SSID: ");
  Serial.println(ssid);
  Serial.print("PSoC IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.print("UDP server ready on port: ");
  Serial.println(localPort);
  Serial.println("Packet format: [0xAA, armed, roll, pitch]");
  Serial.println("----------------------------------");
}


/*******************************************************************************
* Function Name: loop
********************************************************************************
* Summary:
*   Arduino main loop. Drains the UDP RX buffer keeping only the latest valid
*   packet to avoid command lag, updates the shared steering state, runs the
*   connection-loss failsafe check and periodically updates the elevon
*   servos via elevon_mixing / servo_write at SERVO_INTERVAL cadence.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void loop() {
  // Drain UDP buffer - keep only the latest packet to avoid command lag
  uint8_t last_steer = 0, last_roll = NEUTRAL_ROLL, last_pitch = NEUTRAL_PITCH;
  bool got_packet = false;
  unsigned long currentMillis = millis();

  while (true) {
          int n = udp.parsePacket();
          if (n < 4) break; 

          uint8_t header = udp.read();
          if (header != 0xAA) continue;  // misaligned - discard

          last_steer = udp.read();   // byte[1]: steering active flag
          last_roll  = udp.read();   // byte[2]: roll left  servo angle (0–180)
          last_pitch = udp.read();   // byte[3]: pitch angle (0–180, 90 = neutral)
          got_packet = true;
  }

  if (got_packet) 
      {
          steer_active  = (last_steer != 0);
          target_roll   = last_roll;
          target_pitch  = last_pitch;
          lastPackageTime = currentMillis;
          hasEverReceived = true;

        Serial.print("RX: armed=");
        Serial.print(steer_active ? 1 : 0);
        Serial.print(" R="); Serial.print(target_roll);
        Serial.print(" P="); Serial.println(target_pitch);
      }
    
    if (hasEverReceived && (currentMillis - lastPackageTime > TIMEOUT_CONNECTION_LOST)) 
      {
        Failsafe();
      }

  if(currentMillis - lastServoUpdate >= SERVO_INTERVAL)
      {
        lastServoUpdate = currentMillis;
        
        int goal_left, goal_right;
          if (steer_active) 
              {
                elevon_mixing(target_pitch, target_roll, &goal_left, &goal_right);
                prev_steer_active = true;
              } 
          else 
              {
                // snap smooth state to neutral on disarm transition
                if (prev_steer_active) {
                    smooth_pitch = 0;
                    smooth_roll  = 0;
                    prev_steer_active = false;
                }
                elevon_mixing(NEUTRAL_PITCH, NEUTRAL_ROLL, &goal_left, &goal_right);
              }

          // write every cycle
              servo_write(SERVO_LEFT_PIN, goal_left);
              last_left = goal_left;
            
              servo_write(SERVO_RIGHT_PIN, goal_right);
              last_right = goal_right;
            
      }
}