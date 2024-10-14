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
#include <time.h>

#define MAX_LINE_LENGTH 256
//There are 3 different methods for reading the current DTCs list:
#define TEST_DTCS_CALLBACK 1      // Test DTC callback notification to get the DTCs list whenever it has changed
#define TEST_DTCS_COPY 1          // Test DTC copy that is triggered when 'check_dtcs' returns 'true'
#define TEST_DTCS_DYNAMIC_COPY 1  // Test DTC dynamic copy that is triggered when 'check_dtcs' returns 'true'
#define TEST_DTCS_REFERENCE 1     // Test DTC direct access that is triggered when 'check_dtcs' returns 'true'
#define WRITE_JSON_LOG_FILE 1

// Flag global para controlar se é a primeira execução
bool is_first_execution = true;


void write_dtcs_to_json_file(const DTC_Info_t* active_dtcs, const size_t active_dtc_count, int timestamp) {
    // Buffer para armazenar a string da data/hora
    char time_buffer[20];

    // Obter o timestamp atual
    time_t current_time = time(NULL);

    // Somar o timestamp atual com o timestamp fornecido
    time_t final_time = current_time + timestamp;

    // Converter o timestamp final para o formato de data/hora legível
    struct tm *time_info = localtime(&final_time);
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", time_info);

        // Se for a primeira execução, deletar o arquivo, se ele existir
    if (is_first_execution) {
        // Deletar o arquivo
        remove("dtcs_log.txt");
        // Marcar que a primeira execução já ocorreu
        is_first_execution = false;
    }
    
    // Abrir o arquivo no modo de anexo
    FILE* file = fopen("dtcs_log.txt", "a");
    if (file == NULL) {
        printf("Erro ao abrir o arquivo.\n");
        return;
    }

    // Escrever os dados no formato JSON
    fprintf(file, "{ data_pacote: '%s', dtcs: [ ", time_buffer);

    for (size_t i = 0; i < active_dtc_count; i++) {
        fprintf(file, "{ src: %u, spn: %u, fmi: %u }",
                active_dtcs[i].dtc.src,
                active_dtcs[i].dtc.spn,
                active_dtcs[i].dtc.fmi);

        if (i < active_dtc_count - 1) {
            fprintf(file, ", "); // Adicionar vírgula entre os DTCs, exceto o último
        }
    }

    fprintf(file, " ] },\n");

    // Fechar o arquivo
    fclose(file);
}

void active_dtcs_callback(const DTC_Info_t* active_dtcs, const size_t active_dtc_count) {
    printf("TEST Active DTCs Callback: %i\n", active_dtc_count);
    print_dtcs(active_dtcs, active_dtc_count);
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
                process_dtc_frame(can_id, data_bytes, timestamp);
            }
            else if(sscanf(line, "%lf ", &double_timestamp) == 1) {
                timestamp = (unsigned int)double_timestamp;
            }

            //Check if it has passed 1 second
            if(timestamp - last_timestamp >= 1) {
                last_timestamp = timestamp;
                
                // 'check_dtcs' MUST be called once per second by the user's application 
                // in order to remove inactive DTCs and verify if DTCs list was changed
                bool dtcs_changed = check_dtcs(timestamp); 
                
                if(dtcs_changed) {
                    #if TEST_DTCS_COPY
                    DTC_Info_t dtcs_copy[MAX_ACTIVE_DTCS];
                    uint8_t dtc_copy_count = 0;
                    if(copy_dtcs(dtcs_copy, sizeof(dtcs_copy), &dtc_copy_count)) {
                        printf("TEST Active DTCs Copy: %i\n", dtc_copy_count);
                        print_dtcs(dtcs_copy, dtc_copy_count);
                        #if WRITE_JSON_LOG_FILE
                        write_dtcs_to_json_file(dtcs_copy, dtc_copy_count, timestamp);
                        #endif
                    }
                    #endif

                    #if TEST_DTCS_DYNAMIC_COPY
                    DTC_Info_t *dtcs_dynamic = NULL;
                    uint8_t dtcs_dynamic_count = 0;
                    if(dynamic_copy_dtcs(&dtcs_dynamic, &dtcs_dynamic_count)) {
                        printf("TEST Active DTCs Dynamic Copy: %i\n", dtcs_dynamic_count);
                        print_dtcs(dtcs_dynamic, dtcs_dynamic_count);
                        free(dtcs_dynamic);
                    }
                    #endif

                    #if TEST_DTCS_REFERENCE
                    if (take_dtc_mutex()) {
                        uint8_t dtcs_reference_count = 0;
                        const DTC_Info_t* dtcs_reference = get_reference_to_dtcs(&dtcs_reference_count);
                        printf("TEST Active DTCs Reference: %i\n", dtcs_reference_count);
                        print_dtcs(dtcs_reference, dtcs_reference_count);
                        give_dtc_mutex();
                    } 
                    #endif
                }
                
            }
        }
    }

    fclose(file);
}

int main() {
    #if TEST_DTCS_CALLBACK
    // Register callback
    register_dtc_updated_callback(active_dtcs_callback);
    #endif

    // Set debounce times
    set_dtc_filtering(5, 5, 5, 5);

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
