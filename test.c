/**
 * @file test_dtc_parser.c
 * @brief Test application for the DTC parser library
 *
 * This application simulates the reading of CAN frames and uses the DTC parser library to process them.
 * 
 * @authored by Roger da Silva Moschiel
 * @date 2024
 */

#include "dtc_parser.h"
#include <stdio.h>
#include <stdint.h>

#define MAX_TEST_FRAMES 10

void active_faults_callback(Fault* active_faults, size_t active_faults_count) {
    printf("Active Faults Updated:\n");
    print_faults(active_faults, active_faults_count);
}

int main() {
    // Register callback
    register_active_faults_callback(active_faults_callback);

    // Set debounce times
    set_debounce_times(1, 1, 20); // Times in seconds

    // Simulate CAN frames
    uint32_t can_ids[MAX_TEST_FRAMES] = {0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00, 0x18FECA00};
    uint8_t data[MAX_TEST_FRAMES][8] = {
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00},
        {0xFF, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00}
    };
    uint32_t timestamps[MAX_TEST_FRAMES] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000}; // Times in milliseconds

    for (int i = 0; i < MAX_TEST_FRAMES; i++) {
        process_can_frame(can_ids[i], data[i], timestamps[i]);
        check_faults(timestamps[i]); // Constantly remove inactive faults
    }

    return 0;
}
