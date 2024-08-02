/**
 * @file dtc_parser.h
 * @brief Header file for the DTC parser library
 *
 * This library parses J1939 Diagnostic Trouble Code (DTC) messages from CAN frames.
 * It manages active and inactive faults and supports multi-frame messages.
 * 
 * @authored by Roger da Silva Moschiel
 * @date 2024
 */

#ifndef DTC_PARSER_H
#define DTC_PARSER_H

#include <stdint.h>

#define MAX_CONCURRENT_MULTIFRAME 4    // Maximum concurrent multi-frame messages
#define MAX_MULTIFRAME_DATA_SIZE 512   // Maximum data size for multi-frame messages
#define MAX_CANDIDATE_FAULTS 50        // Maximum number of candidate faults
#define MAX_ACTIVE_FAULTS 20           // Maximum number of active faults

/**
 * @brief Struct for a fault
 */
typedef struct {
    uint8_t src;
    struct {
        uint8_t mil:2;      // Malfunction Indicator Lamp
        uint8_t rsl:2;      // Red Stop Lamp
        uint8_t awl:2;      // Amber Warning Lamp
        uint8_t pl:2;       // Protect Lamp status
        uint32_t spn:19;    // Suspect Number Parameter
        uint8_t fmi:5;      // Failure Module Indicator
        uint8_t cm:1;       // Conversion Method
        uint8_t oc:7;       // Occurrence Counter 
    } __attribute__((packed)); //DM1 (4 bytes)
    uint32_t first_seen;
    uint32_t last_seen;
    uint16_t occurrences;
} __attribute__((packed)) Fault;

/**
 * @brief Struct for a multi-frame message
 */
typedef struct {
    uint32_t message_id;
    uint32_t message_id_tp_ct;
    uint32_t total_size;
    uint32_t num_packets;
    uint32_t received_packets;
    uint8_t data[MAX_MULTIFRAME_DATA_SIZE];
} MultiFrameMessage;

/**
 * @brief Callback type for active faults
 */
typedef void (*ActiveFaultsCallback)(Fault* active_faults, size_t active_faults_count);

/**
 * @brief Sets the debounce times for faults
 *
 * @param active_count Number of occurrences for a fault to become active
 * @param active_time Debounce time for a fault to become active (in seconds)
 * @param inactive_time Debounce time for a fault to become inactive (in seconds)
 */
void set_debounce_times(uint32_t active_count, uint32_t active_time, uint32_t inactive_time);

/**
 * @brief Registers a callback function for active fault notifications
 *
 * @param callback Callback function to be called when active faults are updated
 */
void register_active_faults_callback(ActiveFaultsCallback callback);

/**
 * @brief Processes a CAN message and updates faults
 *
 * @param can_id CAN message ID
 * @param data CAN message data
 * @param timestamp Timestamp of the message in seconds
 */
void process_can_frame(uint32_t can_id, uint8_t data[8], uint32_t timestamp);

/**
 * @brief Check faults, must be called periodically by the user's application
 *
 * @param timestamp Current timestamp in seconds
 */
void check_faults(uint32_t timestamp);

/**
 * @brief Prints the fault list
 *
 * @param list Fault list to be printed
 * @param count Number of faults in the list
 */
void print_faults(Fault* list, size_t count);

#endif // DTC_PARSER_H
