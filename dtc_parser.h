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
#define MAX_MULTIFRAME_DATA_SIZE 512  // Maximum data size for multi-frame messages

/**
 * @brief Enum for fault status
 */
typedef enum {
    FAULT_CANDIDATE,
    FAULT_ACTIVE,
    FAULT_INACTIVE
} FaultStatus;

/**
 * @brief Struct for a fault
 */
typedef struct {
    uint32_t src;
    uint32_t spn;
    uint32_t fmi;
    uint8_t cm;
    uint8_t oc;
    uint8_t mil;
    uint8_t rsl;
    uint8_t awl;
    uint8_t pl;
    uint32_t first_seen;
    uint32_t last_seen;
    uint32_t occurrences;
    FaultStatus status;
} Fault;

/**
 * @brief Node in the linked list of faults
 */
typedef struct FaultNode {
    Fault fault;
    struct FaultNode* next;
} FaultNode;

/**
 * @brief Linked list of faults
 */
typedef struct {
    FaultNode* head;
} FaultList;

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
typedef void (*ActiveFaultsCallback)(FaultList* active_faults);

/**
 * @brief Sets the debounce times for faults
 *
 * @param active_time Debounce time for a fault to become active (in seconds)
 * @param active_count Number of occurrences for a fault to become active
 * @param inactive_time Debounce time for a fault to become inactive (in seconds)
 */
void set_debounce_times(uint32_t active_time, uint32_t active_count, uint32_t inactive_time);

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
 * @brief Prints the fault list
 *
 * @param list Fault list to be printed
 */
void print_faults(FaultList* list);

#endif // DTC_PARSER_H
