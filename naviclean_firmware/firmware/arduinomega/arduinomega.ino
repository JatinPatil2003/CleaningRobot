#include <PID_v1.h>

#define Encoder_output_A_left 3  // pin2 of the Arduino
#define Encoder_output_B_left 5  // pin 3 of the Arduino
#define Encoder_output_A_right 2 // pin2 of the Arduino
#define Encoder_output_B_right 4 // pin 3 of the Arduino

#define Motor_R_Dir 6
#define Motor_R_PWM 7
#define Motor_L_Dir 8
#define Motor_L_PWM 9


#define Motor_ticks 3133 // Number of encoder ticks per revolution

int Count_pulses_left = 0;
int Count_pulses_right = 0;
int previous_pulses_left = 0;
int previous_pulses_right = 0;

double right_setpoint = 0.0;
double right_input = 0.0;
double right_output = 0.0;

double left_setpoint = 0.0;
double left_input = 0.0;
double left_output = 0.0;

double Kp_r = 15.0;
double Ki_r = 100.0;
double Kd_r = 0.01;
double Kp_l = 15.0;
double Ki_l = 100.0;
double Kd_l = 0.02;

PID rightMotor(&right_input, &right_output, &right_setpoint, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_input, &left_output, &left_setpoint, Kp_l, Ki_l, Kd_l, DIRECT);

unsigned long last_millis = 0;
const unsigned long interval = 50; // Control loop interval in milliseconds
const double time_interval = interval / 1000.0; // Time interval in seconds

void setup()
{
    pinMode(Encoder_output_A_left, INPUT);
    pinMode(Encoder_output_B_left, INPUT);
    pinMode(Encoder_output_A_right, INPUT);
    pinMode(Encoder_output_B_right, INPUT);

    pinMode(Motor_L_Dir, OUTPUT);
    pinMode(Motor_L_PWM, OUTPUT);
    pinMode(Motor_R_Dir, OUTPUT);
    pinMode(Motor_R_PWM, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(Encoder_output_A_left), DC_Motor_Encoder_left, RISING);
    attachInterrupt(digitalPinToInterrupt(Encoder_output_A_right), DC_Motor_Encoder_right, RISING);

    rightMotor.SetMode(AUTOMATIC);
    leftMotor.SetMode(AUTOMATIC);

    leftMotor.SetOutputLimits(-255, 255);
    rightMotor.SetOutputLimits(-255, 255);
    Serial.begin(115200);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String receivedData = Serial.readStringUntil('\n');

        int commaIndex = receivedData.indexOf(',');

        if (commaIndex != -1)
        {
            String left = receivedData.substring(0, commaIndex);
            String right = receivedData.substring(commaIndex + 1);

            left_setpoint = left.toFloat();  // Setpoint in rad/sec for left motor
            right_setpoint = right.toFloat(); // Setpoint in rad/sec for right motor
        }
    }

    unsigned long current_millis = millis();
    if (current_millis - last_millis >= interval)
    {
        // Calculate angular velocities from the difference in encoder pulses
        int pulse_difference_left = Count_pulses_left - previous_pulses_left;
        int pulse_difference_right = Count_pulses_right - previous_pulses_right;

        left_input = (pulse_difference_left * 2 * PI) / (Motor_ticks * time_interval);  // rad/sec
        right_input = (pulse_difference_right * 2 * PI) / (Motor_ticks * time_interval);  // rad/sec

        // Store the current pulse counts for the next iteration
        previous_pulses_left = Count_pulses_left;
        previous_pulses_right = Count_pulses_right;

        // Compute the PID outputs
        rightMotor.Compute();
        leftMotor.Compute();

        if(left_setpoint == 0.0)
            left_output = 0.0;
        if(right_setpoint == 0.0)
            right_output = 0.0;

        // Update motor control based on PID output
        controlMotor(Motor_L_Dir, Motor_L_PWM, left_output);
        controlMotor(Motor_R_Dir, Motor_R_PWM, right_output);

        double leftangle = Count_pulses_left * 2 * PI / Motor_ticks;
        double rightangle = Count_pulses_right * 2 * PI / Motor_ticks;

        String dataToSend = String(leftangle) + "," + String(rightangle);
        Serial.println(dataToSend);

//         String dataToSend = String(left_input) + "," + String(right_input) + "|" + String(left_output) + "," + String(right_output);
//         Serial.println(dataToSend);
        
        // Serial.print(Count_pulses_left);
        // Serial.print(",");
        // Serial.println(Count_pulses_right);

        last_millis = current_millis;
    }
}

void controlMotor(int dirPin, int pwmPin, double output)
{
    if (output > 0)
    {
        digitalWrite(dirPin, HIGH); // Set motor direction forward
        analogWrite(pwmPin, int(constrain(output, 0, 255))); // Generate PWM proportional to output (0 to 255)
    }
    else
    {
        digitalWrite(dirPin, LOW); // Set motor direction backward
        analogWrite(pwmPin, int(constrain(-output, 0, 255))); // Generate PWM proportional to the magnitude of the output
    }
}

void DC_Motor_Encoder_left()
{
    int b = digitalRead(Encoder_output_B_left);
    if (b > 0)
    {
        Count_pulses_left--;
    }
    else
    {
        Count_pulses_left++;
    }
}

void DC_Motor_Encoder_right()
{
    int b = digitalRead(Encoder_output_B_right);
    if (b > 0)
    {
        Count_pulses_right--;
    }
    else
    {
        Count_pulses_right++;
    }
}
