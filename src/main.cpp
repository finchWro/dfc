#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static int64_t sampling_freq = EI_CLASSIFIER_FREQUENCY; // in Hz.
static int64_t time_between_samples_us = (1000000 / (sampling_freq - 1));

// to classify 1 frame of data you need EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE values
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

int main() {
    // output immediately without buffering
    setvbuf(stdout, NULL, _IONBF, 0);

    // get driver for the accelerometer
    const struct device *adi = DEVICE_DT_GET(DT_NODELABEL(accel));
    if (adi == NULL) {
        printf("Could not get adi device\n");
        return 1;
    }

    struct sensor_value accel[3];

    while (1) {
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
            // start a timer that expires when we need to grab the next value
            struct k_timer next_val_timer;
            k_timer_init(&next_val_timer, NULL, NULL);
            k_timer_start(&next_val_timer, K_USEC(time_between_samples_us), K_NO_WAIT);

            // read data from the sensor
            if (sensor_sample_fetch(adi) < 0) {
                printf("IIS2DLPC Sensor sample update error\n");
                return 1;
            }

            sensor_channel_get(adi, SENSOR_CHAN_ACCEL_XYZ, accel);

            // fill the features array
            features[ix + 0] = sensor_value_to_double(&accel[0]);
            features[ix + 1] = sensor_value_to_double(&accel[1]);
            features[ix + 2] = sensor_value_to_double(&accel[2]);

            // busy loop until next value should be grabbed
            while (k_timer_status_get(&next_val_timer) <= 0);
        }

        // frame full? then classify
        ei_impulse_result_t result = { 0 };

        // create signal from features frame
        signal_t signal;
        numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

        // run classifier
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        printf("run_classifier returned: %d\n", res);
        if (res != 0) return 1;

        // print predictions
        printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

        // print the predictions
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
        printf("anomaly:\t%.3f\n", result.anomaly);
    #endif
    }
}


