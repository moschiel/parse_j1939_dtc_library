/**
 * @file dtc_parser.h
 * @brief Header file for the DTC parser library
 *
 * This library is designed to parse J1939 Diagnostic Trouble Code (DTC) messages from CAN frames.
 * It manages the detection and tracking of faults by maintaining lists of candidate 
 * and active faults. The library can handle both single-frame and 
 * multi-frame DTC messages (using the BAM transport protocol). It is optimized for use within 
 * a CAN interrupt handler, with built-in mutex protection to ensure safe concurrent access 
 * to the fault list. If the fault list is being accessed when a new DTC frame arrives, the 
 * library will skip processing that frame to avoid concurrency issues, accepting the possibility 
 * of missing some frames for the sake of system stability.
 * 
 * Key features include:
 * - Parse J1939 DTC messages (single and multi-frame).
 * - Maintain active and candidate fault lists.
 * - Mutex-protected access to fault lists, suitable for use in interrupts.
 * - Debouncing mechanism to handle fault transition from 'candidate' to 'active'.
 * - Debouncing mechanism to handle fault removal once it is considered 'inactive'.
 * - Customizable fault handling with user-defined callback functions.
 *
 * @authored by Roger da Silva Moschiel
 * @date 1 August 2024
 */


#ifndef DTC_PARSER_H
#define DTC_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_CONCURRENT_MULTIFRAME 4    // Maximum concurrent multi-frame messages
#define MAX_MULTIFRAME_DATA_SIZE 256   // Maximum data size for multi-frame messages
#define MAX_CANDIDATE_FAULTS 40        // Maximum number of candidate faults
#define MAX_ACTIVE_FAULTS 20           // Maximum number of active faults

/**
 * @brief Struct with DTC parameters (j1939 DM1 parameters)
 */
typedef struct {
    uint8_t src;        // Source
    uint8_t mil:2;      // Malfunction Indicator Lamp
    uint8_t rsl:2;      // Red Stop Lamp
    uint8_t awl:2;      // Amber Warning Lamp
    uint8_t pl:2;       // Protect Lamp status
    uint32_t spn:19;    // Suspect Number Parameter
    uint8_t fmi:5;      // Failure Module Indicator
    uint8_t cm:1;       // Conversion Method
    uint8_t oc:7;       // Occurrence Counter 
} __attribute__((packed)) DTC_t; // (6 bytes)

/**
 * @brief Struct for a fault
 */
typedef struct {
    DTC_t dtc;
    uint32_t first_seen;
    uint32_t last_seen;
    uint16_t read_count;
} __attribute__((packed)) Fault; // (6 + 4 + 4 + 2 = 16 bytes)

/**
 * @brief Struct for a multi-frame message
 */
typedef struct {
    uint32_t message_id;
    uint32_t message_id_tp_dt;
    uint32_t total_size;
    uint32_t num_packets;
    uint32_t received_packets;
    uint32_t first_seen;
    uint32_t last_seen;
    uint8_t data[MAX_MULTIFRAME_DATA_SIZE];
} MultiFrameMessage;

/**
 * @brief Struct for debounces logic
 */
typedef struct  {
    uint32_t fault_active_read_count;       // Number of read_count that must occur within a time window for a fault to become active
    uint32_t fault_active_time_window;      // Time window for a fault to become active (in seconds)
    uint32_t debounce_fault_inactive_time;  // Remove faults that have not been updated by this amount of time (seconds)
    uint32_t timeout_multi_frame;           // Maximum time to receive a complete multiframe message, otherwise discards the message
} DtcParseConfig_t;

/**
 * @brief Callback type for active faults updated
 */
typedef void (*UpdatedActiveFaultsCallback)(const Fault* active_faults, const size_t active_faults_count);

/**
 * @brief Sets the debounce times for faults
 *
 * @param _fault_active_read_count_ Number of read_count that must occur within a time window for a fault to become active
 * @param _fault_active_time_window_ Time window for a fault to become active (in seconds)
 * @param _debounce_fault_inactive_time_ Remove faults that have not been updated by this amount of time (seconds)
 * @param _timeout_multi_frame_ Maximum time to receive a complete multiframe message, otherwise discards the message
 */
void set_j1939_fault_debounce(uint32_t _fault_active_read_count_, uint32_t _fault_active_time_window_, uint32_t _debounce_fault_inactive_time_, uint32_t _timeout_multi_frame_);

/**
 * @brief Registers a callback function for active fault updated notifications
 *
 * @param callback Callback function to be called when active faults are updated
 */
void register_j1939_updated_faults_callback(UpdatedActiveFaultsCallback callback);

/**
 * @brief Processes a CAN message and updates faults, it has a built-in mutex protection
 *
 * This function is designed to be called inside a CAN interrupt handler on a bare-metal 
 * microcontroller. It uses a mutex to prevent concurrent access to the fault list. If 
 * the fault list is currently being accessed elsewhere and the mutex is occupied, this 
 * function will skip processing the frame to avoid concurrency issues. The user must 
 * be aware that skipping a frame could result in missing a fault frame, but it is 
 * necessary to maintain concurrence safety in a bare-metal environment.
 *
 * @param can_id CAN message ID
 * @param data CAN message data
 * @param timestamp Timestamp of the message in seconds
 */

void process_j1939_dtc_frame(uint32_t can_id, uint8_t data[8], uint32_t timestamp);

/**
 * @brief Check faults, *MUST* be called once per second by the user's application
 *
 * This function removes any inactive fault and checks for any changes in the fault list. 
 * It returns `true` if the fault list was updated, and `false` if there were no changes.
 * The function uses a mutex to ensure thread-safe access to the fault list.
 *
 * @param timestamp Current timestamp in seconds
 * @return bool True if the fault list was updated, false if there were no changes
 */
bool check_j1939_faults(uint32_t timestamp);

/**
 * @brief Prints the fault list
 *
 * @param list Fault list to be printed
 * @param count Number of faults in the list
 */
void print_j1939_faults(const Fault* list, const size_t count);

/**
 * @brief Copies the current active faults into a user-provided buffer.
 *
 * This function copies the list of active faults into a user-provided buffer. The buffer size 
 * is checked to ensure it is large enough to hold the active fault data. The function uses a 
 * mutex to ensure thread-safe access to the fault list.
 *
 * @param buf_faults_list Pointer to the buffer where the fault list will be copied
 * @param buf_size Size of the buffer provided by the user
 * @param faults_count Pointer to a variable where the number of copied faults will be stored
 * @return bool True if the faults were successfully copied, false if the buffer was too small or the mutex was not available
 */
bool copy_j1939_faults(Fault* buf_faults_list, uint16_t buf_size, uint8_t* faults_count);

/**
 * @brief Dynamically allocates and copies the current active faults.
 *
 * This function dynamically allocates memory for the active fault list and copies the 
 * current active faults into it. The function uses a mutex to ensure thread-safe access 
 * to the fault list. The user is responsible for freeing the allocated memory.
 *
 * Note: The function requires a pointer to a pointer (`Fault**`) as the first argument
 * to allow the allocated memory to be returned to the caller.
 *
 * @param buf_faults_list Pointer to a pointer that will point to the dynamically allocated buffer where the fault list will be copied
 * @param faults_count Pointer to a variable where the number of copied faults will be stored
 * @return bool True if the faults were successfully copied and memory was allocated, false if allocation failed or the mutex was not available
 */
bool dynamic_copy_j1939_faults(Fault **buf_faults_list, uint8_t* faults_count);

#endif // DTC_PARSER_H
