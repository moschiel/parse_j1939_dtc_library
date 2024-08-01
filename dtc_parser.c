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
static FaultList candidate_faults_list = { NULL };
static FaultList active_faults_list = { NULL };
static MultiFrameMessage* multi_frame_messages[MAX_CONCURRENT_MULTIFRAME] = { NULL };
static uint32_t debounce_fault_active_time = 10000;
static uint32_t debounce_fault_active_count = 3;
static uint32_t debounce_fault_inactive = 20000;
static ActiveFaultsCallback active_faults_callback = NULL;

// Private function prototypes
static void add_fault(FaultList* list, Fault fault);
static void remove_fault(FaultList* list, FaultNode* prev_node, FaultNode* node);
static FaultNode* find_fault(FaultList* list, uint32_t src, uint32_t spn, uint32_t fmi);
static void update_fault_status(FaultList* active_faults, FaultList* candidate_faults, uint32_t timestamp);
static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp);
static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static MultiFrameMessage* find_multi_frame_message(uint32_t message_id);
static void add_multi_frame_message(MultiFrameMessage* message);
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
    if((can_id & 0x00FFFF00) == 0x00FECA00) // single frame DM1 message
    {
        process_dm1_message(can_id, data, 8, timestamp);
    } 
    else if (((can_id & 0x00FF0000) == 0x00EC0000)) //multi frame message
    {
        handle_tp_cm_message(can_id, data, timestamp);
    } 
    else if ((can_id & 0x00FF0000) == 0x00EB0000)  //multi frame data
    {
        handle_tp_dt_message(can_id, data, timestamp);
    }
}

void print_faults(FaultList* list) {
    FaultNode* current = list->head;
    while (current != NULL) {
        Fault* fault = &current->fault;
        printf("SRC: 0x%X, SPN: %u, FMI: %u, Status: %d\n", fault->src, fault->spn, fault->fmi, fault->status);
        current = current->next;
    }
}

// Private functions

static void add_fault(FaultList* list, Fault fault) {
    FaultNode* new_node = (FaultNode*)malloc(sizeof(FaultNode));
    new_node->fault = fault;
    new_node->next = list->head;
    list->head = new_node;
}

static void remove_fault(FaultList* list, FaultNode* prev_node, FaultNode* node) {
    if (prev_node == NULL) {
        list->head = node->next;
    } else {
        prev_node->next = node->next;
    }
    free(node);
}

static FaultNode* find_fault(FaultList* list, uint32_t src, uint32_t spn, uint32_t fmi) {
    FaultNode* current = list->head;
    while (current != NULL) {
        if (current->fault.src == src && current->fault.spn == spn && current->fault.fmi == fmi) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

static void update_fault_status(FaultList* active_faults, FaultList* candidate_faults, uint32_t timestamp) {
    FaultNode* current = candidate_faults->head;
    FaultNode* prev_node = NULL;

    while (current != NULL) {
        FaultNode* next = current->next;

        if (current->fault.status == FAULT_CANDIDATE) {
            if ((timestamp - current->fault.first_seen) <= debounce_fault_active_time && current->fault.occurrences >= debounce_fault_active_count) {
                current->fault.status = FAULT_ACTIVE;
                add_fault(active_faults, current->fault);
                remove_fault(candidate_faults, prev_node, current);
            } else {
                prev_node = current;
            }
        } else if (current->fault.status == FAULT_ACTIVE) {
            if (timestamp - current->fault.last_seen >= debounce_fault_inactive) {
                current->fault.status = FAULT_INACTIVE;
                remove_fault(active_faults, prev_node, current);
            } else {
                prev_node = current;
            }
        }
        current = next;
    }

    if (active_faults_callback != NULL) {
        active_faults_callback(active_faults);
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

    for (uint32_t i = 2; i < length; i += 4) {
        spn = (((data[i + 2] >> 5) & 0x7) << 16) | ((data[i + 1] << 8) & 0xFF00) | data[i];
        fmi = data[i + 2] & 0x1F;
        cm = (data[i + 3] >> 7) & 0x01;
        oc = data[i + 3] & 0x7F;

        FaultNode* existing_fault = find_fault(&active_faults_list, src, spn, fmi);
        if (existing_fault) {
            existing_fault->fault.oc = oc;
            existing_fault->fault.mil = mil;
            existing_fault->fault.rsl = rsl;
            existing_fault->fault.awl = awl;
            existing_fault->fault.pl = pl;
            existing_fault->fault.last_seen = timestamp;
        } else {
            Fault new_fault = {src, spn, fmi, cm, oc, mil, rsl, awl, pl, timestamp, timestamp, 1, FAULT_CANDIDATE};
            add_fault(&candidate_faults_list, new_fault);
        }
    }

    update_fault_status(&active_faults_list, &candidate_faults_list, timestamp);
}

static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    uint32_t message_id = can_id & 0x1FFFFFFF;
    uint32_t pgn = (data[7] << 16) | (data[6] << 8) | data[5];

    if(pgn == 0xFECA) { // Multiframe DTC
        uint8_t control_byte = data[0];

        if (control_byte == 0x20) { //BAM message
            uint32_t total_size = (data[2] << 8) | data[1];
            uint32_t num_packets = data[3];
            MultiFrameMessage* message = (MultiFrameMessage*)malloc(sizeof(MultiFrameMessage));
            message->message_id = message_id;
            message->message_id_tp_ct = (message_id & 0xFFFF00FF) | 0xEB00; // TP.CT version of the TP.CM message
            message->total_size = total_size;
            message->num_packets = num_packets;
            message->received_packets = 0;
            memset(message->data, 0, MAX_MULTIFRAME_DATA_SIZE);
            add_multi_frame_message(message);
        } 
        //else -> TODO: RTS / CTS message
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
    for (uint32_t i = 0; i < MAX_MULTIFRAME_DATA_SIZE; i++) {
        if (multi_frame_messages[i] && multi_frame_messages[i]->message_id == message_id) {
            return multi_frame_messages[i];
        }
    }
    return NULL;
}

static void add_multi_frame_message(MultiFrameMessage* message) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (!multi_frame_messages[i]) {
            multi_frame_messages[i] = message;
            return;
        }
    }
}

static void remove_multi_frame_message(uint32_t message_id) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (multi_frame_messages[i] && multi_frame_messages[i]->message_id == message_id) {
            free(multi_frame_messages[i]);
            multi_frame_messages[i] = NULL;
            return;
        }
    }
}
