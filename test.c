/**
 * @file test.c
 * @brief Test application for the DTC parser library
 *
 * This application simulates the reading of CAN frames and uses the DTC parser library to process them.
 * 
 * @authored by Roger da Silva Moschiel
 * @date 1 August 2024
 */

#include "dtc_parser/dtc_parser.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define MAX_LINE_LENGTH 256
//There are 3 different methods for reading the current faults list:
#define TEST_FAULTS_CALLBACK 1      // Test fault callback notification to get the faults list whenever it has changed
#define TEST_FAULTS_COPY 1          // Test fault copy that is triggered when 'check_j1939_faults' returns 'true'
#define TEST_FAULTS_DYNAMIC_COPY 1  // Test fault dynamic copy that is triggered when 'check_j1939_faults' returns 'true'
#define TEST_FAULTS_REFERENCE 1     // Test fault direct access that is triggered when 'check_j1939_faults' returns 'true'

void active_faults_callback(const Fault* active_faults, const size_t active_faults_count) {
    printf("TEST Active Faults Callback: %i\n", active_faults_count);
    print_j1939_faults(active_faults, active_faults_count);
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
                last_timestamp = timestamp;
                
                // 'check_faults' MUST be called once per second by the user's application 
                // in order to remove inactive faults and verify if faults list was changed
                bool faults_changed = check_j1939_faults(timestamp); 
                
                if(faults_changed) {
                    #if TEST_FAULTS_COPY
                    Fault faults_copy[MAX_ACTIVE_FAULTS];
                    uint8_t faults_copy_count = 0;
                    if(copy_j1939_faults(faults_copy, sizeof(faults_copy), &faults_copy_count)) {
                        printf("TEST Active Faults Copy: %i\n", faults_copy_count);
                        print_j1939_faults(faults_copy, faults_copy_count);
                    }
                    #endif

                    #if TEST_FAULTS_DYNAMIC_COPY
                    Fault *faults_dynamic = NULL;
                    uint8_t faults_dynamic_count = 0;
                    if(dynamic_copy_j1939_faults(&faults_dynamic, &faults_dynamic_count)) {
                        printf("TEST Active Faults Dynamic Copy: %i\n", faults_dynamic_count);
                        print_j1939_faults(faults_dynamic, faults_dynamic_count);
                        free(faults_dynamic);
                    }
                    #endif

                    #if TEST_FAULTS_REFERENCE
                    if (take_j1939_faults_mutex()) {
                        uint8_t faults_reference_count = 0;
                        const Fault* faults_reference = get_reference_to_j1939_faults(&faults_reference_count);
                        printf("TEST Active Faults Reference: %i\n", faults_reference_count);
                        print_j1939_faults(faults_reference, faults_reference_count);
                        give_j1939_faults_mutex();
                    } 
                    #endif
                }
                
            }
        }
    }

    fclose(file);
}

int main() {
    #if TEST_FAULTS_CALLBACK
    // Register callback
    register_j1939_updated_faults_callback(active_faults_callback);
    #endif

    // Set debounce times
    set_j1939_fault_debounce(10, 10, 10, 5);

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
