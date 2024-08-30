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

// Print controls
#define PRINT_DM1_FRAME 0
#define PRINT_DM1_PARSED 0
#define PRINT_TP_CM_FRAME 0
#define PRINT_TP_CM_PARSED 0
#define PRINT_TP_DT_FRAME 0
#define PRINT_TP_DT_PARSED 0
#define PRINT_TP_DT_INCORRECT_ORDER 0
#define PRINT_TP_CONCAT_MULTI_FRAME 0
#define PRINT_NEW_AND_REMOVED_DTC 1
#define PRINT_WARNINGS 1


// Private variables
static Fault candidate_faults[MAX_CANDIDATE_FAULTS];
static size_t candidate_faults_count = 0;
static Fault active_faults[MAX_ACTIVE_FAULTS];
static size_t active_faults_count = 0;
static MultiFrameMessage multi_frame_messages[MAX_CONCURRENT_MULTIFRAME];
static uint32_t fault_active_count = 3;
static uint32_t fault_active_time_window = 10;
static uint32_t debounce_fault_inactive_time = 20;
static uint32_t timeout_multi_frame = 5;
static ActiveFaultsCallback active_faults_callback = NULL;
static uint8_t changedFaultList = 0;

// Private function prototypes
static void remove_inactive_faults(uint32_t timestamp);
static void add_candidate_fault(Fault fault);
static void add_active_fault(Fault fault);
static Fault* find_fault(Fault* list, size_t count, uint32_t src, uint32_t spn, uint32_t fmi);
static void update_fault_status(uint32_t timestamp, uint8_t src, uint32_t spn, uint8_t fmi, uint8_t cm, uint8_t oc, uint8_t mil, uint8_t rsl, uint8_t awl, uint8_t pl);
static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp);
static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static MultiFrameMessage* find_multi_frame_message(uint32_t message_id_tp_dt);
static void remove_multi_frame_message(uint32_t message_id_tp_dt);
static void remove_incomplete_multi_frame_message(uint32_t timestamp);

void set_j1939_fault_debounce(uint32_t _fault_active_count_, uint32_t _fault_active_time_window_, uint32_t _debounce_fault_inactive_time_, uint32_t _timeout_multi_frame_) {
    fault_active_count = _fault_active_count_;
    fault_active_time_window = _fault_active_time_window_;
    debounce_fault_inactive_time = _debounce_fault_inactive_time_;
    timeout_multi_frame = _timeout_multi_frame_;
}

void register_j1939_faults_callback(ActiveFaultsCallback callback) {
    active_faults_callback = callback;
}

void process_j1939_dtc_frame(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    if ((can_id & 0x00FFFF00) == 0x00FECA00) { // single frame DM1 message
        #if PRINT_DM1_FRAME
        printf("[%u] DM1_FRAME -> ID: %08X, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            timestamp,
            can_id, 
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        #endif
        process_dm1_message(can_id, data, 8, timestamp);
    } 
    else if ((can_id & 0x00FF0000) == 0x00EC0000) { // multi frame message
        handle_tp_cm_message(can_id, data, timestamp);
    } 
    else if ((can_id & 0x00FF0000) == 0x00EB0000) { // multi frame data
        handle_tp_dt_message(can_id, data, timestamp);
    }
}

void print_j1939_faults(const Fault* list, const size_t count) {
    for (size_t i = 0; i < count; ++i) {
        Fault* f = &list[i];
        printf("LastSeen: %u, SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u, CM: %u, OC: %u, MIL: %u, RSL: %u, AWL: %u, PL: %u\n", 
            f->last_seen, f->src, f->src, f->spn, f->spn, f->fmi, f->cm, f->oc, f->mil, f->rsl, f->awl, f->pl);
    }
}

void check_j1939_faults(uint32_t timestamp) {
    remove_inactive_faults(timestamp);
    remove_incomplete_multi_frame_message(timestamp);
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
        if ((timestamp - candidate_faults[i].first_seen) > fault_active_time_window) {
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
        if ((timestamp - active_faults[i].last_seen) > debounce_fault_inactive_time) {
            #if PRINT_NEW_AND_REMOVED_DTC
            Fault f = active_faults[i];
            printf("[%u] Removed fault -> SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u, LastSeen: %u\n",
                timestamp, f.src, f.src, f.spn, f.spn, f.fmi, f.last_seen);
            #endif

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
    } else {
        #if PRINT_WARNINGS
        printf("WARNING: Cannot exceed MAX_CANDIDATE_FAULTS: %u\n", MAX_CANDIDATE_FAULTS);
        #endif
    }
}

static void add_active_fault(Fault fault) {
    if (active_faults_count < MAX_ACTIVE_FAULTS) {
        active_faults[active_faults_count++] = fault;
        changedFaultList = 1;

        #if PRINT_NEW_AND_REMOVED_DTC
        printf("[%u] New fault -> SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u\n",
            fault.last_seen, fault.src, fault.src, fault.spn, fault.spn, fault.fmi);
        #endif
    } else {
        #if PRINT_WARNINGS
        printf("WARNING: Cannot exceed MAX_ACTIVE_FAULTS: %u", MAX_ACTIVE_FAULTS);
        #endif
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

static void update_fault_status(uint32_t timestamp, uint8_t src, uint32_t spn, uint8_t fmi, uint8_t cm, uint8_t oc, uint8_t mil, uint8_t rsl, uint8_t awl, uint8_t pl) {
    Fault* existing_fault = find_fault(active_faults, active_faults_count, src, spn, fmi);
    if (existing_fault) {
        // Update if exist on Active list already
        existing_fault->oc = oc;
        existing_fault->mil = mil;
        existing_fault->rsl = rsl;
        existing_fault->awl = awl;
        existing_fault->pl = pl;
        existing_fault->last_seen = timestamp;
    } else {
        existing_fault = find_fault(candidate_faults, candidate_faults_count, src, spn, fmi);
        if (existing_fault) {
            // Update if exist on Candidate list already
            existing_fault->oc = oc;
            existing_fault->mil = mil;
            existing_fault->rsl = rsl;
            existing_fault->awl = awl;
            existing_fault->pl = pl;
            existing_fault->occurrences += 1;
            existing_fault->last_seen = timestamp;
        } else {
            // Add new candidate fault to the fault list
            Fault new_fault = {
                .src = src, 
                .spn = spn, 
                .fmi = fmi, 
                .cm = cm, 
                .oc = oc, 
                .mil = mil, 
                .rsl = rsl, 
                .awl = awl, 
                .pl = pl, 
                .first_seen = timestamp, 
                .last_seen = timestamp, 
                .occurrences = 1
            };    
            add_candidate_fault(new_fault);
        }
    }
    
    // Promote candidate faults to active if they meet the criteria
    for (size_t i = 0; i < candidate_faults_count; ++i) {
        if ((timestamp - candidate_faults[i].first_seen <= fault_active_time_window) && //Check if is within the window time to become active
            (candidate_faults[i].occurrences >= fault_active_count)) { // Check if has the minimum amount of occurrences
            add_active_fault(candidate_faults[i]);
            // Remove from candidates
            for (size_t j = i; j < candidate_faults_count - 1; ++j) {
                candidate_faults[j] = candidate_faults[j + 1];
            }
            --candidate_faults_count;
            --i; // recheck the current index
        }
    }
}

static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp) {
    if(length < 6) return;

    uint32_t spn = (((data[4] >> 5) & 0x7) << 16) | ((data[3] << 8) & 0xFF00) | data[2];
    if(spn == 0) return;

    uint8_t src = can_id & 0xFF;
    uint8_t fmi;
    uint8_t cm;
    uint8_t oc;
    uint8_t mil = (data[0] >> 6) & 0x03;
    uint8_t rsl = (data[0] >> 4) & 0x03;
    uint8_t awl = (data[0] >> 2) & 0x03;
    uint8_t pl = data[0] & 0x03;

    #if PRINT_DM1_PARSED
    printf("[%u] DM1_PARSED -> SRC: 0x%02X (%u), MIL: %u, RSL: %u, AWL: %u, PL: %u\n",
        timestamp, src, src, mil, rsl, awl, pl);
    int f = 1;
    #endif

    for (uint32_t i = 2; i < (length-2); i += 4) {
        spn = (((data[i + 2] >> 5) & 0x7) << 16) | ((data[i + 1] << 8) & 0xFF00) | data[i];
        fmi = data[i + 2] & 0x1F;
        cm = (data[i + 3] >> 7) & 0x01;
        oc = data[i + 3] & 0x7F;

        #if PRINT_DM1_PARSED
        printf("        DTC[%u] -> SPN: 0x%X (%u), FMI: %u, CM: %u, OC: %u\n",
            f, spn, spn, fmi, cm, oc);
        f++;
        #endif

        update_fault_status(timestamp, src, spn, fmi, cm, oc, mil, rsl, awl, pl);
    }
}

static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    uint32_t pgn = (data[7] << 16) | (data[6] << 8) | data[5];
    if (pgn != 0xFECA) return; //If it is not a DTC multiframe, return

    uint32_t message_id = can_id & 0x1FFFFFFF;
    uint8_t control_byte = data[0];

    if (control_byte == 0x20) { // BAM message
        uint32_t total_size = (data[2] << 8) | data[1];
        uint32_t num_packets = data[3];

        #if PRINT_TP_CM_FRAME
        printf("[%u] TP_CM_FRAME -> ID: %08X, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            timestamp,
            can_id, 
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        #endif
        #if PRINT_TP_CM_PARSED
        printf("[%u] TP_CM_PARSED -> ID: %08X, Total Size: %u bytes, Number of Packets: %u\n",
            timestamp, can_id, total_size, num_packets);
        #endif

        if(total_size > MAX_MULTIFRAME_DATA_SIZE) {
            #if PRINT_WARNINGS
            printf("[%u] WARNING: Cannot exceed MAX_MULTIFRAME_DATA_SIZE: %u\n", timestamp, MAX_MULTIFRAME_DATA_SIZE);
            #endif
            return;
        }

        int8_t k = -1; // Point to available slot

        // "TP.CM" is the anouncement of a new multiframe BAM message, 
        // therefore it should not exist on memory, but if it exists, we will overwrite the slot
        for (int i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
            if (multi_frame_messages[i].message_id == message_id) {
                k = i;
                break;
            }
        }
        // If it is not on memory, we search for one available slot
        if(k == -1) {
            for (int i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
                if (multi_frame_messages[i].message_id == 0) {
                    k = i;
                    break;
                }
            }
        }

        // Store on memory if slot available
        if (k >= 0) {
            multi_frame_messages[k].message_id = message_id;
            multi_frame_messages[k].message_id_tp_dt = (message_id & 0xFF00FFFF) | 0xEB0000; // TP.DT version of the TP.CM message
            multi_frame_messages[k].total_size = total_size;
            multi_frame_messages[k].num_packets = num_packets;
            multi_frame_messages[k].received_packets = 0;
            multi_frame_messages[k].first_seen = timestamp;
            multi_frame_messages[k].last_seen = timestamp;
            memset(multi_frame_messages[k].data, 0, MAX_MULTIFRAME_DATA_SIZE);
        } else {
            #if PRINT_WARNINGS
            printf("[%u] WARNING: Cannot exceed MAX_CONCURRENT_MULTIFRAME: %u\n", timestamp, MAX_CONCURRENT_MULTIFRAME);
            #endif
        }
    } else {
        #if PRINT_WARNINGS
        printf("[%u] WARNING: NOT BAM MESSAGE\n", timestamp);
        #endif
    }
}

static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    uint32_t message_id = can_id & 0x1FFFFFFF;
    MultiFrameMessage* message = find_multi_frame_message(message_id);

    if (message) {
        uint8_t packet_number = data[0];
        
        #if PRINT_TP_DT_FRAME
        printf("[%u] TP_DT_FRAME -> ID: %08X, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            timestamp,
            can_id, 
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        #endif
        #if PRINT_TP_DT_PARSED
        printf("[%u] TP_DT_PARSED -> ID: %08X, Packet Number: %u of %u, Data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            timestamp, can_id, packet_number, message->num_packets, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        #endif

        if(packet_number != (message->received_packets + 1)) {
            #if PRINT_TP_DT_INCORRECT_ORDER
            printf("[%u] Packet Order is Incorrect, ID: %08X, Received: %u, Expected: %u\n", 
                timestamp, message_id, packet_number, message->received_packets + 1);
            #endif
            remove_multi_frame_message(message_id);
            return;
        }

        uint32_t offset = (packet_number - 1) * 7;
        memcpy(&message->data[offset], &data[1], 7);
        message->received_packets++;
        message->last_seen = timestamp;

        if (message->received_packets == message->num_packets) {
            #if PRINT_TP_CONCAT_MULTI_FRAME
                printf("[%u] TP_CONCAT -> ID: %08X, Size: %u, Data:", timestamp, message->message_id, message->total_size);
                for(uint8_t i=0; (i < message->total_size) && (i < sizeof(message->data)); i++) {
                    printf(" %02X", message->data[i]);
                }
                printf("\n");
            #endif
            process_dm1_message(message_id, message->data, message->total_size, timestamp);
            remove_multi_frame_message(message_id);
        }
    }
}

static MultiFrameMessage* find_multi_frame_message(uint32_t message_id_tp_dt) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (multi_frame_messages[i].message_id_tp_dt == message_id_tp_dt) {
            return &multi_frame_messages[i];
        }
    }
    return NULL;
}

static void remove_multi_frame_message(uint32_t message_id_tp_dt) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if (multi_frame_messages[i].message_id_tp_dt == message_id_tp_dt) {
            memset(&multi_frame_messages[i], 0, sizeof(MultiFrameMessage));
            return;
        }
    }
}

static void remove_incomplete_multi_frame_message(uint32_t timestamp) {
    for (uint32_t i = 0; i < MAX_CONCURRENT_MULTIFRAME; i++) {
        if(multi_frame_messages[i].message_id) {
            if ((timestamp - multi_frame_messages[i].last_seen) > timeout_multi_frame) {
                #if PRINT_WARNINGS
                printf("[%u] WARNING: discard incomplete multiframe, CM: 0x%X, DT: 0x%X, FirstSeen: %u, LastSeen: %u\n", 
                    timestamp, multi_frame_messages[i].message_id, multi_frame_messages[i].message_id_tp_dt, multi_frame_messages[i].first_seen, multi_frame_messages[i].last_seen);
                #endif
                memset(&multi_frame_messages[i], 0, sizeof(MultiFrameMessage));
            }
        }
    }
}