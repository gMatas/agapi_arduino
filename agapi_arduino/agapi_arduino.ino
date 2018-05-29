#include <Wire.h>
#include <LSM303.h>
#include <SoftwareSerial.h>


// Fall detection handling
// Set receiver pin (8) and transmiter pin (9)
SoftwareSerial bluetooth_serial(8, 9);

uint8_t fall_detection_pin = 2;
uint8_t user_input_pin_1 = 3;
uint8_t user_input_pin_2 = 4;

uint8_t fall_warning_led_pin = 11;
uint8_t confirmation_led_pin = 12;

unsigned long fall_warning_start_timestamp = 0;
unsigned long fall_warning_led_blink_start_timestamp = 0;
unsigned long confirmation_led_blink_start_timestamp = 0;

bool is_fall_detection_on = true;
bool is_fall_detected = false;

bool waiting_response = false;

bool is_help_abort_message_sent = false;
bool is_sending_help_abort_message = false;

bool is_help_message_sent = false;
bool is_sending_help_message = false;

// Fall detection sensor
LSM303 sensor;
const unsigned int position_data_length = 80;
float position_data[position_data_length];
unsigned int position_data_index;
unsigned int position_data_is_ready;
const float POSITION_HORIZONTAL_TILT_THRESHOLD_CONST = 4.0;
const float POSITION_VERTICAL_TILT_THRESHOLD_CONST = 8.0;
bool is_horizontal_position = false;
unsigned long fall_detect_occurance_timestamp = 0;

float global_accel_vector_modulus_low_pass = 0.0;
bool is_impact_detection_active = true;
const float IMPACT_UPPER_LIMIT_THRESHOLD_CONST = 16.0;
const float IMPACT_LOWER_LIMIT_THRESHOLD_CONST = -8.0;
bool pre_impact_detected = false;
unsigned long pre_impact_detection_timestamp = 0;
bool impact_detected = false;
unsigned long impact_detection_timestamp = 0;


void setup() 
{
    Serial.begin(9600);
    bluetooth_serial.begin(9600);

    bluetooth_serial.print("AT+NAMEAGAPI");
    bluetooth_serial.print("AT+ROLE1");

    pinMode(fall_detection_pin, INPUT);
    pinMode(user_input_pin_1, INPUT);
    pinMode(user_input_pin_2, INPUT);

    pinMode(fall_warning_led_pin, OUTPUT);
    pinMode(confirmation_led_pin, OUTPUT);

    Wire.begin();
    sensor.init();
    sensor.enableDefault();
 
    reset_position_data();
}

void loop()
{   
    // Handle fall detection
    if (is_fall_detection_on)
    {
        bool is_auto_fall = detect_fall();
        bool simulate_fall = (uint8_t) digitalRead(fall_detection_pin) == HIGH;
        bool is_manual_fall = (uint8_t) digitalRead(user_input_pin_1) == HIGH || simulate_fall;
        
        // is_auto_fall = false;

        handle_fall_detection(is_manual_fall || is_auto_fall);
        return;
    }
    if (is_help_message_sent)
    {
        if (digitalRead(user_input_pin_2) == HIGH)
        {       
            is_help_message_sent = false;
            waiting_response = true;
            is_sending_help_abort_message = true;
            bluetooth_serial.print("<AGAPI=HELP_ABORT>\n");
            return;
        }
    }
    if (is_help_abort_message_sent)
    {
        is_help_abort_message_sent = false;
        waiting_response = false;
        is_fall_detection_on = true;
        return;
    }

    // Master response handler
    if (waiting_response)
    {
        if (bluetooth_serial.available() > 0)
        {
            String response_message = "";
            char c;
            while ((c = (char) bluetooth_serial.read()) != -1)
            {
                response_message += c;
                delay(5);
                
            }
            response_message.trim();

            // Handle response from the master
            Serial.print("The Response: " + response_message + "\n");

            if (is_sending_help_message && response_message == "<AGAPI=HELP_OK>")
            {
                Serial.print("help is on the way: " + response_message + "\n");

                digitalWrite(fall_warning_led_pin, LOW);
                digitalWrite(confirmation_led_pin, HIGH);

                waiting_response = false;
                is_sending_help_message = false;
                is_help_message_sent = true;
                return;
            }
            if (is_sending_help_abort_message && response_message == "<AGAPI=HELP_ABORT_OK>")
            {
                Serial.print("help was stoped: " + response_message + "\n");

                digitalWrite(fall_warning_led_pin, LOW);
                digitalWrite(confirmation_led_pin, LOW);

                waiting_response = false;
                is_sending_help_abort_message = false;
                is_help_abort_message_sent = true;
                return;
            }
        }
    } 
    else
    {
        // Flush buffer
        while (bluetooth_serial.available() > 0)
        {
            bluetooth_serial.read();
            delay(2);
        }
    }

    if (is_sending_help_message && !is_help_message_sent)
    {
        // bluetooth_serial.print("<AGAPI=HELP+TEMPERATURE=69+ELEVATION=420>\n");  
        confirmation_led_blink_start_timestamp = blink(
            confirmation_led_pin, confirmation_led_blink_start_timestamp, 200);
    }
    else if (is_sending_help_abort_message && !is_help_abort_message_sent)
    {
        // bluetooth_serial.print("<AGAPI=HELP_ABORT>\n");
        confirmation_led_blink_start_timestamp = blink(
            confirmation_led_pin, confirmation_led_blink_start_timestamp, 200);
    }
}

void handle_fall_detection(bool is_fall)
{
    // Fall detection handler
    if (!is_fall_detected && is_fall)
    {
        is_fall_detected = true;
        fall_warning_start_timestamp = millis();
        fall_warning_led_blink_start_timestamp = millis();
        Serial.print("fall detected!\n");
        Serial.print("waiting for user response...\n");
    } 

    // Inform user about fall detection
    if (is_fall_detected)
    {
        // Wait for warning timeout
        if ((unsigned long) (millis() - fall_warning_start_timestamp) >= 10000)
        {            
            Serial.print("no user response. Fall might have occured.\n");
            digitalWrite(fall_warning_led_pin, HIGH);
            is_fall_detection_on = false;  // suspend fall detection
            is_fall_detected = false;
            is_sending_help_message = true;

            bluetooth_serial.print("<AGAPI=HELP+TEMPERATURE=69>\n");

            waiting_response = true;
            Serial.print("need to send help msg.\n");
        }
        else
        {
            // Wait for user response
            if (digitalRead(user_input_pin_1) == HIGH)
            {
                Serial.print("user response received! Fall occured.\n");
                digitalWrite(fall_warning_led_pin, HIGH);
                is_fall_detection_on = false;  // suspend fall detection
                is_fall_detected = false;
                is_sending_help_message = true;

                bluetooth_serial.print("<AGAPI=HELP+TEMPERATURE=69>\n");

                waiting_response = true;
                Serial.print("need to send help msg.\n");
            }
            if (digitalRead(user_input_pin_2) == HIGH)
            {
                Serial.print("user response received! Fall did not occured.\n");
                digitalWrite(fall_warning_led_pin, LOW);
                is_fall_detected = false;
            }
            // Blink warning led 
            else
            {
                fall_warning_led_blink_start_timestamp = blink(
                    fall_warning_led_pin, fall_warning_led_blink_start_timestamp, 300);
            }
        }
    }
}

unsigned long blink(unsigned int led_pin, unsigned long start_timestamp, unsigned int interval)
{
    if ((unsigned long) (millis() - start_timestamp) >= interval) 
    {
        digitalWrite(led_pin, digitalRead(led_pin) == HIGH ? LOW : HIGH);
        return millis();
    }
}

void reset_position_data()
{
    position_data_index = 0;
    for (int i = 0; i < position_data_length; i++) 
        position_data[i] = 0.0;
    position_data_is_ready = false;
}

float get_position_data_average(float current_position)
{
    float position_data_average = -1.0;
    position_data[position_data_index++] = current_position;
    if (position_data_index >= position_data_length)
    {
        position_data_index = 0;
        position_data_is_ready = true;
    }
    if (position_data_is_ready)
    {
        position_data_average = 0.0;
        for (int i = 0; i < position_data_length; i++)
            position_data_average += position_data[i];
        position_data_average /= position_data_length;
    }
    return position_data_average;
}

bool detect_fall()
{
    sensor.read();
    // Converting raw accel readings to G's:
    // * constants for the maximum range of +/-16G = 0.732mG/LSB (got it from the LSM303 datasheet);
    // * G's are calculated by using this formula: G = raw*const, const = 0.000732G/LSB;
    const float LSM303_const_val = (float)(0.000732);   // a value used for converting raw accel data to G's
    float x_G = sensor.a.x * LSM303_const_val;
    float y_G = sensor.a.y * LSM303_const_val;
    float z_G = sensor.a.z * LSM303_const_val;

    // Get the average value of the position
    float position_data_average = get_position_data_average(abs(y_G));

    // Get acceleration vector modulus
    float accel_vector_modulus = sqrt(x_G * x_G + y_G * y_G + z_G * z_G);  // |a| = sqrt(x_G^2 + y_G^2 + z_G^2)

    // Apply low-pass filter to acceleration value
    global_accel_vector_modulus_low_pass = 0.9 * global_accel_vector_modulus_low_pass + 0.1 * accel_vector_modulus;
    accel_vector_modulus -= global_accel_vector_modulus_low_pass;
    // Serial.print("A=" + String(accel_vector_modulus) + "\n");

    if (is_impact_detection_active)
    {
        if (accel_vector_modulus >= IMPACT_UPPER_LIMIT_THRESHOLD_CONST && !pre_impact_detected && !impact_detected)
        {
            // Detect fall pre-impact
            pre_impact_detection_timestamp = millis();
            pre_impact_detected = true;
            Serial.print("pre impact detected\n");
        } 
        else if (accel_vector_modulus <= IMPACT_LOWER_LIMIT_THRESHOLD_CONST && pre_impact_detected && !impact_detected)
        {
            // Detect fall impact
            impact_detection_timestamp = millis();
            pre_impact_detected = false;
            impact_detected = true;
            Serial.print("impact detected\n");
        }

        if (impact_detected && (unsigned long) (millis() - impact_detection_timestamp) >= 5000)
        {
            Serial.print("activating another\n");
            impact_detected = false;
            is_impact_detection_active = false;
        }
        else if (pre_impact_detected && (unsigned long) (millis() - pre_impact_detection_timestamp) >= 1000)
        {
            pre_impact_detected = false;
        }       
        return false;
    }

    // Fall position detection
    if (!is_impact_detection_active && position_data_is_ready)
    {    
        if (!is_horizontal_position && position_data_average <= POSITION_HORIZONTAL_TILT_THRESHOLD_CONST)
        {
            is_horizontal_position = true;
            fall_detect_occurance_timestamp = millis();
            Serial.println("user is now horizontal");
        } 
        else if (position_data_average >= POSITION_VERTICAL_TILT_THRESHOLD_CONST)
        {
            is_impact_detection_active = true;
            is_horizontal_position = false;
            Serial.println("user is now vertical");
            return false;
        } 
        if (is_horizontal_position && (unsigned long) (millis() - fall_detect_occurance_timestamp) >= 10000) 
        {
            is_impact_detection_active = true;
            reset_position_data();
            is_horizontal_position = false;
            return true;
        }
    }
    return false;    
}
