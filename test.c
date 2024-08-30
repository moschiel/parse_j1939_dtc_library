/**
 * @file test_dtc_parser.c
 * @brief Test application for the DTC parser library
 *
 * This application simulates the reading of CAN frames and uses the DTC parser library to process them.
 * 
 * @authored by Roger da Silva Moschiel
 * @date 2024
 */

#include "dtc_parser/dtc_parser.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define MAX_LINE_LENGTH 256

void active_faults_callback(Fault* active_faults, size_t active_faults_count) {
    // printf("Active Faults Updated:\n");
    // print_faults(active_faults, active_faults_count);
}

void process_asc_file(const char* file_path) {
    FILE* file = fopen(file_path, "r");
    if (!file) {
        perror("Failed to open file");
        return;
    }
    uint32_t last_timestamp = 0;

    char line[MAX_LINE_LENGTH];
    while (fgets(line, sizeof(line), file)) {
        if (strstr(line, "Rx") && !strstr(line, "J1939TP")) {
            uint32_t can_id;
            double double_timestamp;
            int timestamp;
            char direction[3];
            char can_id_str[10];
            uint8_t data_bytes[8];

            // Parse the line to extract CAN frame information
            // Example line format: "   6.474846 1  18FECA03x       Rx   d 8 04 FF 22 EE E3 81 FF FF  Length = 559804 BitCount = 144 ID = 419351043x"
            if (sscanf(line, "%lf %s %s %*s %*s %*s %x %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                    &double_timestamp, direction, can_id_str,
                    &data_bytes[0], &data_bytes[1], &data_bytes[2], &data_bytes[3],
                    &data_bytes[4], &data_bytes[5], &data_bytes[6], &data_bytes[7]) == 11) {
                // timestamp = (int)(float_timestamp * 1000); // Convert to milliseconds
                timestamp = (unsigned int)double_timestamp;

                can_id = strtoul(can_id_str, NULL, 16);

                // Process the CAN frame
                process_j1939_dtc_frame(can_id, data_bytes, timestamp);
            }
            else if(sscanf(line, "%lf ", &double_timestamp) == 1) {
                timestamp = (unsigned int)double_timestamp;
            }

            //Check if it has passed 1 second
            if(timestamp - last_timestamp >= 1) {
                check_j1939_faults(timestamp); // 'check_faults' must be called periodically by the user's application
                last_timestamp = timestamp;
            }
        }
    }

    fclose(file);
}

int main() {
    // Register callback
    register_j1939_faults_callback(active_faults_callback);

    // Set debounce times
    set_j1939_fault_debounce(10, 10, 10);

    // Process the .asc file
    // const char* file_path = "canalyzer_logs/test.asc";
    const char* file_path = "canalyzer_logs/VWConstel2024_1.asc";
    // const char* file_path = "canalyzer_logs/VWConstel2024_2.asc";
    // const char* file_path = "canalyzer_logs/Accelo2023-2024_817.asc";
    // const char* file_path = "canalyzer_logs/Atego_2024_666kbs.asc";
    // const char* file_path = "canalyzer_logs/Atego03-07-24SaoGabriel.asc";
    // const char* file_path = "canalyzer_logs/Daf_BoaViagem_BDE8B87.asc";
    // const char* file_path = "canalyzer_logs/DAF_E6.asc";
    // const char* file_path = "canalyzer_logs/DAFBoa viagem.asc";
    
    process_asc_file(file_path);

    return 0;
}
