/**
 * @file dtc_parser.c
 * @brief Source file for the DTC parser library
 *
 * This library parses J1939 Diagnostic Trouble Code (DTC) messages from CAN frames.
 * It manages active and inactive faults and supports multi-frame messages.
 * 
 * @authored by Roger da Silva Moschiel
 * @date 2024
 */

#include "dtc_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Private variables
static Fault candidate_faults[MAX_CANDIDATE_FAULTS];
static size_t candidate_faults_count = 0;
static Fault active_faults[MAX_ACTIVE_FAULTS];
static size_t active_faults_count = 0;
static MultiFrameMessage multi_frame_messages[MAX_CONCURRENT_MULTIFRAME];
static uint32_t debounce_fault_active_time = 10;
static uint32_t debounce_fault_active_count = 3;
static uint32_t debounce_fault_inactive = 20;
static ActiveFaultsCallback active_faults_callback = NULL;
static uint8_t changedFaultList = 0;

// Private function prototypes
static void remove_inactive_faults(uint32_t timestamp);
static void add_candidate_fault(Fault fault);
static void add_active_fault(Fault fault);
static Fault* find_fault(Fault* list, size_t count, uint32_t src, uint32_t spn, uint32_t fmi);
static void update_fault_status(uint32_t timestamp);
static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp);
static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static MultiFrameMessage* find_multi_frame_message(uint32_t message_id);
static void remove_multi_frame_message(uint32_t message_id);

void set_debounce_times(uint32_t active_time, uint32_t active_count, uint32_t inactive_time) {
    debounce_fault_active_time = active_time;
    debounce_fault_active_count = active_count;
    debounce_fault_inactive = inactive_time;
}

void register_active_faults_callback(ActiveFaultsCallback callback) {
    active_faults_callback = callback;
}

void process_can_frame(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    if ((can_id & 0x00FFFF00) == 0x00FECA00) { // single frame DM1 message
        process_dm1_message(can_id, data, 8, timestamp);
    } 
    else if ((can_id & 0x00FF0000) == 0x00EC0000) { // multi frame message
        handle_tp_cm_message(can_id, data, timestamp);
    } 
    else if ((can_id & 0x00FF0000) == 0x00EB0000) { // multi frame data
        handle_tp_dt_message(can_id, data, timestamp);
    }
}

void print_faults(Fault* list, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        Fault* fault = &list[i];
        printf("SRC: 0x%X, SPN: %u, FMI: %u, Status: %d\n", fault->src, fault->spn, fault->fmi, fault->status);
    }
}

void check_faults(uint32_t timestamp) {
    remove_inactive_faults(timestamp);
    // Notify via callback
    if (changedFaultList && active_faults_callback != NULL) {
        changedFaultList = 0;
        active_faults_callback(active_faults, active_faults_count);
    }
}

// Private functions
static void remove_inactive_faults(uint32_t timestamp) {
    // Check candidate faults to be removed
    for (size_t i = 0; i < candidate_faults_count; ++i) {
        if (timestamp - candidate_faults[i].first_seen > debounce_fault_active_time) {
            // Shift remaining faults
            for (size_t j = i; j < candidate_faults_count - 1; ++j) {
                candidate_faults[j] = candidate_faults[j + 1];
            }
            --candidate_faults_count;
            --i; // recheck the current index
        }
    }

    // Check active faults to be removed
    for (size_t i = 0; i < active_faults_count; ++i) {
        if (timestamp - active_faults[i].last_seen > debounce_fault_inactive) {
            // Shift remaining faults
            for (size_t j = i; j < active_faults_count - 1; ++j) {
                active_faults[j] = active_faults[j + 1];
            }
            --active_faults_count;
            --i; // recheck the current index
            changedFaultList = 1;
        }
    }
}


static void add_candidate_fault(Fault fault) {
    if (candidate_faults_count < MAX_CANDIDATE_FAULTS) {
        candidate_faults[candidate_faults_count++] = fault;
    }
}

static void add_active_fault(Fault fault) {
    if (active_faults_count < MAX_ACTIVE_FAULTS) {
        active_faults[active_faults_count++] = fault;
    }
}

static Fault* find_fault(Fault* list, size_t count, uint32_t src, uint32_t spn, uint32_t fmi) {
    for (size_t i = 0; i < count; ++i) {
        if (list[i].src == src && list[i].spn == spn && list[i].fmi == fmi) {
            return &list[i];
        }
    }
    return NULL;
}

static void update_fault_status(uint32_t timestamp) {
    // Promote candidate faults to active if they meet the criteria
    for (size_t i = 0; i < candidate_faults_count; ++i) {
        if ((timestamp - candidate_faults[i].first_seen <= debounce_fault_active_time) && 
            (candidate_faults[i].occurrences >= debounce_fault_active_count)) {
            candidate_faults[i].status = FAULT_ACTIVE;
            add_active_fault(candidate_faults[i]);

            // Remove from candidates
            for (size_t j = i; j < candidate_faults_count - 1; ++j) {
                candidate_faults[j] = candidate_faults[j + 1];
            }
            --candidate_faults_count;
            --i; // recheck the current index
            changedFaultList = 1;
        }
    }
}

static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp) {
    uint32_t src = can_id & 0xFF;
    uint32_t spn;
    uint32_t fmi;
    uint8_t cm;
    uint8_t oc;
    uint8_t mil = (data[0] >> 6) & 0x03;
    uint8_t rsl = (data[0] >> 4) & 0x03;
    uint8_t awl = (data[0] >> 2) & 0x03;
    uint8_t pl = data[0] & 0x03;

    for (uint32_t i = 2; i < (length-2); i += 4) {
        spn = (((data[i + 2] >> 5) & 0x7) << 16) | ((data[i + 1] << 8) & 0xFF00) | data[i];
        fmi = data[i + 2] & 0x1F;
        cm = (data[i + 3] >> 7) & 0x01;
        oc = data[i + 3] & 0x7F;

        Fault* existing_fault = find_fault(active_faults, active_faults_count, src, spn, fmi);
        if (existing_fault) {
            existing_fault->oc = oc;
            existing_fault->mil = mil;
            existing_fault->rsl = rsl;
            existing_fault->awl = awl;
            existing_fault->pl = pl;
            existing_fault->last_seen = timestamp;
        } else {
            Fault new_fault = {src, spn, fmi, cm, oc, mil, rsl, awl, pl, timestamp, timestamp, 1, FAULT_CANDIDATE};
            add_candidate_fault(new_fault);
        }
    }

    update_fault_status(timestamp); // Update fault status
}

static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    uint32_t message_id = can_id & 0x1FFFFFFF;
    uint32_t pgn = (data[7] << 16) | (data[6] << 8) | data[5];

    if (pgn == 0xFECA) { // Multiframe DTC
        uint8_t control_byte = data[0];

        if (control_byte == 0x20) { // BAM message
            uint32_t total_size = (data[2] << 8) | data[1];
            uint32_t num_packets = data[3];

            for (int i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
                if (multi_frame_messages[i].message_id == 0) {
                    multi_frame_messages[i].message_id = message_id;
                    multi_frame_messages[i].message_id_tp_ct = (message_id & 0xFFFF00FF) | 0xEB00; // TP.CT version of the TP.CM message
                    multi_frame_messages[i].total_size = total_size;
                    multi_frame_messages[i].num_packets = num_packets;
                    multi_frame_messages[i].received_packets = 0;
                    memset(multi_frame_messages[i].data, 0, MAX_MULTIFRAME_DATA_SIZE);
                    break;
                }
            }
        }
    }
}

static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    uint32_t message_id = can_id & 0x1FFFFFFF;
    MultiFrameMessage* message = find_multi_frame_message(message_id);

    if (message) {
        uint8_t packet_number = data[0];
        uint32_t offset = (packet_number - 1) * 7;
        memcpy(&message->data[offset], &data[1], 7);
        message->received_packets++;

        if (message->received_packets == message->num_packets) {
            process_dm1_message(message_id, message->data, message->total_size, timestamp);
            remove_multi_frame_message(message_id);
        }
    }
}

static MultiFrameMessage* find_multi_frame_message(uint32_t message_id) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (multi_frame_messages[i].message_id == message_id) {
            return &multi_frame_messages[i];
        }
    }
    return NULL;
}

static void remove_multi_frame_message(uint32_t message_id) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (multi_frame_messages[i].message_id == message_id) {
            multi_frame_messages[i].message_id = 0;
            memset(multi_frame_messages[i].data, 0, MAX_MULTIFRAME_DATA_SIZE);
            return;
        }
    }
}

