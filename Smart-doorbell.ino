#include <momin-project-1_inferencing.h>
#include "Seeed_Arduino_GroveAI.h"
#include "disk91_LoRaE5.h"
#include <Wire.h>

GroveAI ai(Wire);
Disk91_LoRaE5 lorae5(&Serial);

#define Frequency DSKLORAE5_ZONE_EU868
#define BUTTON_PIN WIO_5S_PRESS

char deveui[] = "";
char appeui[] = "";
char appkey[] = "";

uint8_t state = 0;
uint8_t data[3]; // Declare the data array at a higher scope
volatile bool buttonPressed = false; // Flag for button press

static const float features[] = {
    // copy raw features here (for example from the 'Live classification' page)
    // see https://docs.edgeimpulse.com/docs/running-your-impulse-arduino
};

void data_encode(int count, int light, uint8_t* data) {
    data[0] = (count >> 8) & 0xFF;
    data[1] = count & 0xFF;
    data[2] = light & 0xFF;

    // Debugging print
    Serial.print("Encoded data: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}


int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void print_inference_result(ei_impulse_result_t result);
void wakeUp();

void setup() {
    Serial.begin(115200);
    while (!Serial);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(WIO_BUZZER, OUTPUT);

    // Attach interrupt to the button pin
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeUp, FALLING);

    state = 1;
}

void loop() {
    switch(state) {
        case 1: {
            // Initialization state
            Serial.println("Initializing LoRa E5 module...");
            if (!lorae5.begin(DSKLORAE5_SEARCH_WIO)) {
                Serial.println("LoRa E5 Init Failed");
                state = 5; // Move to error state
            } else {
                Serial.println("LoRa E5 Initialized successfully");
                if (!lorae5.setup(Frequency, deveui, appeui, appkey)) {
                    Serial.println("LoRa E5 Setup Failed");
                    state = 5; // Move to error state
                } else {
                    Serial.println("LoRa E5 Setup successfully");
                    state = 2; // Move to waiting for button press and enter simulated sleep
                }
            }
            break;
        }
        case 2: {
            // Simulated sleep and wait for button press
            Serial.println("Entering simulated sleep");
            delay(100); // Allow time for the print statement to be sent
            // Simulated sleep by doing nothing until button is pressed
            while (!buttonPressed) {
                // Sleep in low power mode
                __WFI(); // Wait For Interrupt - low power sleep
            }
            buttonPressed = false; // Reset button press flag
            state = 3; // Move to running inference
            break;
        }
        case 3: {
            // Run inference
            analogWrite(WIO_BUZZER, 128);

            if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
                    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
                delay(1000);
                state = 5; // Move to error state
                break;
            }

            ei_impulse_result_t result = { 0 };
            signal_t features_signal;
            features_signal.total_length = sizeof(features) / sizeof(features[0]);
            features_signal.get_data = &raw_feature_get_data;

            EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
            if (res != EI_IMPULSE_OK) {
                ei_printf("ERR: Failed to run classifier (%d)\n", res);
                state = 5; // Move to error state
                break;
            }

            ei_printf("run_classifier returned: %d\r\n", res);
            print_inference_result(result);

            // Count the number of detected objects
            int detected_objects = 0;
            for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                if (result.bounding_boxes[i].value > 0) {
                    detected_objects++;
                }
            }

            // Read light sensor value
            int light = analogRead(WIO_LIGHT);

            // Debugging print for detected objects count and light value
            Serial.print("Detected objects: ");
            Serial.println(detected_objects);
            Serial.print("Light value: ");
            Serial.println(light);

            data_encode(detected_objects, light, data);

            state = 4; // Move to sending data
            break;
        }

        case 4: {
            // Sending data
            if (lorae5.send_sync(8, data, sizeof(data), false, 7, 14)) {
                Serial.println("Uplink done");
                if (lorae5.isDownlinkReceived()) {
                    Serial.println("A downlink has been received");
                    if (lorae5.isDownlinkPending()) {
                        Serial.println("More downlink are pending");
                    }
                }
            } else {
                Serial.println("Failed to send uplink");
            }
            analogWrite(WIO_BUZZER, 0);
            state = 2; // Move back to simulated sleep and wait for button press
            break;
        }
        case 5: {
            // Error state
            Serial.println("An error occurred");
            while (1); // Halt execution
            break;
        }
        default: {
            // Default case should not be reached
            Serial.println("Invalid state");
            state = 5; // Move to error state
            break;
        }
    }
}

void wakeUp() {
    // Wake up handler
    Serial.println("Waking up from simulated sleep");
    buttonPressed = true; // Set button press flag
}

void print_inference_result(ei_impulse_result_t result) {
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}
