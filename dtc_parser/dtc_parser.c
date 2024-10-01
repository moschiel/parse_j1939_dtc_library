/**
 * @file dtc_parser.c
 * @brief Source file for the DTC parser library
 *
 * This library is designed to parse J1939 Diagnostic Trouble Code (DTC) messages from CAN frames.
 * It manages the detection and tracking of DTCs by maintaining lists of candidate 
 * and active DTCs. The library can handle both single-frame and 
 * multi-frame DTC messages (using the BAM transport protocol). It is optimized for use within 
 * a CAN interrupt handler, with built-in mutex protection to ensure safe concurrent access 
 * to the DTC list. If the DTC list is being accessed when a new DTC frame arrives, the 
 * library will skip processing that frame to avoid concurrency issues, accepting the possibility 
 * of missing some frames for the sake of system stability.
 * 
 * Key features include:
 * - Parse J1939 DTC messages (single and multi-frame).
 * - Maintain active and candidate DTC lists.
 * - Mutex-protected access to DTC lists, suitable for use in interrupts.
 * - Debouncing mechanism to handle DTC transition from 'candidate' to 'active'.
 * - Debouncing mechanism to handle DTC removal once it is considered 'inactive'.
 * - Customizable DTC handling with user-defined callback functions.
 *
 * @authored by Roger da Silva Moschiel
 * @date 1 August 2024
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
#define PRINT_NEW_AND_REMOVED_DTC 0
#define PRINT_WARNINGS 0


// Private variables
static DTC_Info_t candidate_dtcs[MAX_CANDIDATE_DTCS];
static size_t candidate_dtcs_count = 0;
static DTC_Info_t active_dtcs[MAX_ACTIVE_DTCS];
static size_t active_dtcs_count = 0;
static MultiFrameMessage multi_frame_messages[MAX_CONCURRENT_MULTIFRAME];
static UpdatedActiveDTCsCallback updated_active_dtcs_callback = NULL;
static bool changed_dtc_list = false;
static bool dtc_mutex_taken = false;
static DtcParseConfig_t dtcParseCfg = {
    .dtc_active_read_count = 10,
    .dtc_active_time_window = 10,
    .debounce_dtc_inactive_time = 20,
    .timeout_multi_frame = 5
};

// Private function prototypes
static void remove_inactive_dtcs(uint32_t timestamp);
static void add_candidate_dtc(DTC_Info_t DTC_Info);
static void add_active_dtc(DTC_Info_t DTC_Info);
static DTC_Info_t* find_dtc(DTC_Info_t* list, size_t count, uint32_t src, uint32_t spn, uint32_t fmi);
static void update_dtc_status(uint32_t timestamp, uint8_t src, uint32_t spn, uint8_t fmi, uint8_t cm, uint8_t oc, uint8_t mil, uint8_t rsl, uint8_t awl, uint8_t pl);
static void process_dm1_message(uint32_t can_id, uint8_t* data, uint32_t length, uint32_t timestamp);
static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static void handle_tp_dt_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp);
static MultiFrameMessage* find_multi_frame_message(uint32_t message_id_tp_dt);
static void remove_multi_frame_message(uint32_t message_id_tp_dt);
static void remove_incomplete_multi_frame_message(uint32_t timestamp);


// Private functions
static void remove_inactive_dtcs(uint32_t timestamp) {
    // Check candidate DTCs to be removed
    for (size_t i = 0; i < candidate_dtcs_count; ++i) {
        if ((timestamp - candidate_dtcs[i].first_seen) > dtcParseCfg.dtc_active_time_window) {
            // Shift remaining DTCs
            for (size_t j = i; j < candidate_dtcs_count - 1; ++j) {
                candidate_dtcs[j] = candidate_dtcs[j + 1];
            }
            --candidate_dtcs_count;
            --i; // recheck the current index
        }
    }

    // Check active DTCs to be removed
    for (size_t i = 0; i < active_dtcs_count; ++i) {
        if ((timestamp - active_dtcs[i].last_seen) > dtcParseCfg.debounce_dtc_inactive_time) {
            #if PRINT_NEW_AND_REMOVED_DTC
            DTC_Info_t f = active_dtcs[i];
            printf("[%u] Removed DTC -> SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u, LastSeen: %u\n",
                timestamp, f.dtc.src, f.dtc.src, f.dtc.spn, f.dtc.spn, f.dtc.fmi, f.last_seen);
            #endif

            // Shift remaining DTCs
            for (size_t j = i; j < active_dtcs_count - 1; ++j) {
                active_dtcs[j] = active_dtcs[j + 1];
            }
            --active_dtcs_count;
            --i; // recheck the current index
            changed_dtc_list = true;
        }
    }
}

static void add_candidate_dtc(DTC_Info_t DTC_Info) {
    if (candidate_dtcs_count < MAX_CANDIDATE_DTCS) {
        candidate_dtcs[candidate_dtcs_count++] = DTC_Info;
    } else {
        #if PRINT_WARNINGS
        printf("WARNING: Cannot exceed MAX_CANDIDATE_DTCS: %u\n", MAX_CANDIDATE_DTCS);
        #endif
    }
}

static void add_active_dtc(DTC_Info_t f) {
    if (active_dtcs_count < MAX_ACTIVE_DTCS) {
        active_dtcs[active_dtcs_count++] = f;
        changed_dtc_list = true;

        #if PRINT_NEW_AND_REMOVED_DTC
        printf("[%u] New DTC -> SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u\n",
            f.last_seen, f.dtc.src, f.dtc.src, f.dtc.spn, f.dtc.spn, f.dtc.fmi);
        #endif
    } else {
        #if PRINT_WARNINGS
        printf("WARNING: Cannot exceed MAX_ACTIVE_DTCS: %u", MAX_ACTIVE_DTCS);
        #endif
    }
}

static DTC_Info_t* find_dtc(DTC_Info_t* list, size_t count, uint32_t src, uint32_t spn, uint32_t fmi) {
    for (size_t i = 0; i < count; ++i) {
        if (list[i].dtc.src == src && list[i].dtc.spn == spn && list[i].dtc.fmi == fmi) {
            return &list[i];
        }
    }
    return NULL;
}

static void update_dtc_status(uint32_t timestamp, uint8_t src, uint32_t spn, uint8_t fmi, uint8_t cm, uint8_t oc, uint8_t mil, uint8_t rsl, uint8_t awl, uint8_t pl) {
    DTC_Info_t* existing_dtc = find_dtc(active_dtcs, active_dtcs_count, src, spn, fmi);
    if (existing_dtc) {
        // Update if exist on Active list already
        existing_dtc->dtc.oc = oc;
        existing_dtc->dtc.mil = mil;
        existing_dtc->dtc.rsl = rsl;
        existing_dtc->dtc.awl = awl;
        existing_dtc->dtc.pl = pl;
        existing_dtc->last_seen = timestamp;
    } else {
        existing_dtc = find_dtc(candidate_dtcs, candidate_dtcs_count, src, spn, fmi);
        if (existing_dtc) {
            // Update if exist on Candidate list already
            existing_dtc->dtc.oc = oc;
            existing_dtc->dtc.mil = mil;
            existing_dtc->dtc.rsl = rsl;
            existing_dtc->dtc.awl = awl;
            existing_dtc->dtc.pl = pl;
            existing_dtc->read_count += 1;
            existing_dtc->last_seen = timestamp;
        } else {
            // Add new candidate DTC to the DTC list
            DTC_Info_t new_dtc_info = {
                .dtc.src = src, 
                .dtc.spn = spn, 
                .dtc.fmi = fmi, 
                .dtc.cm = cm, 
                .dtc.oc = oc, 
                .dtc.mil = mil, 
                .dtc.rsl = rsl, 
                .dtc.awl = awl, 
                .dtc.pl = pl, 
                .first_seen = timestamp, 
                .last_seen = timestamp, 
                .read_count = 1
            };    
            add_candidate_dtc(new_dtc_info);
        }
    }
    
    // Promote candidate DTCs to active if they meet the criteria
    for (size_t i = 0; i < candidate_dtcs_count; ++i) {
        if ((timestamp - candidate_dtcs[i].first_seen <= dtcParseCfg.dtc_active_time_window) && //Check if is within the window time to become active
            (candidate_dtcs[i].read_count >= dtcParseCfg.dtc_active_read_count)) { // Check if has the minimum amount of read_count
            add_active_dtc(candidate_dtcs[i]);
            // Remove from candidates
            for (size_t j = i; j < candidate_dtcs_count - 1; ++j) {
                candidate_dtcs[j] = candidate_dtcs[j + 1];
            }
            --candidate_dtcs_count;
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

        update_dtc_status(timestamp, src, spn, fmi, cm, oc, mil, rsl, awl, pl);
    }
}

static void handle_tp_cm_message(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    //uint32_t pgn = (data[7] << 16) | (data[6] << 8) | data[5];
    //if (pgn != 0xFECA) return; //If it is not a DTC multiframe, return

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
            multi_frame_messages[k].message_id_tp_dt = (message_id & 0xFF00FFFF) | 0x00EB0000; // TP.DT version of the TP.CM message
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
            if ((timestamp - multi_frame_messages[i].last_seen) > dtcParseCfg.timeout_multi_frame) {
                #if PRINT_WARNINGS
                printf("[%u] WARNING: discard incomplete multiframe, CM: 0x%X, DT: 0x%X, FirstSeen: %u, LastSeen: %u\n", 
                    timestamp, multi_frame_messages[i].message_id, multi_frame_messages[i].message_id_tp_dt, multi_frame_messages[i].first_seen, multi_frame_messages[i].last_seen);
                #endif
                memset(&multi_frame_messages[i], 0, sizeof(MultiFrameMessage));
            }
        }
    }
}

// Public functions
bool take_dtc_mutex() {
    if(dtc_mutex_taken) return false;
    dtc_mutex_taken = true;
    return true;
}

void give_dtc_mutex() {
    dtc_mutex_taken = false;
}

void set_dtc_filtering(uint32_t _dtc_active_read_count_, uint32_t _dtc_active_time_window_, uint32_t _debounce_dtc_inactive_time_, uint32_t _timeout_multi_frame_) {
    if(_dtc_active_read_count_ > 0) dtcParseCfg.dtc_active_read_count = _dtc_active_read_count_;
    if(_dtc_active_time_window_ > 0) dtcParseCfg.dtc_active_time_window = _dtc_active_time_window_;
    if(_debounce_dtc_inactive_time_ > 0) dtcParseCfg.debounce_dtc_inactive_time = _debounce_dtc_inactive_time_;
    if(_timeout_multi_frame_ > 0) dtcParseCfg.timeout_multi_frame = _timeout_multi_frame_;
}

void register_dtc_updated_callback(UpdatedActiveDTCsCallback callback) {
    updated_active_dtcs_callback = callback;
}

void process_dtc_frame(uint32_t can_id, uint8_t data[8], uint32_t timestamp) {
    if(take_dtc_mutex()) {
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
            uint32_t pgn = (data[7] << 16) | (data[6] << 8) | data[5];
            if(pgn == 0xFECA) { // if is DTC multiframe
                uint8_t control_byte = data[0];
                if (control_byte == 0x20) { // BAM message
                    handle_tp_cm_message(can_id, data, timestamp);
                }
            }
        } 
        else if ((can_id & 0x00FF0000) == 0x00EB0000) { // multi frame data
            handle_tp_dt_message(can_id, data, timestamp);  
        }
        give_dtc_mutex();
    }
}

void print_dtcs(const DTC_Info_t* list, const size_t count) {
    for (size_t i = 0; i < count; ++i) {
        const DTC_Info_t *f = &list[i];
        printf("LastSeen: %u, SRC: 0x%02X (%u), SPN: 0x%X (%u), FMI: %u, CM: %u, OC: %u, MIL: %u, RSL: %u, AWL: %u, PL: %u\n", 
            f->last_seen, f->dtc.src, f->dtc.src, f->dtc.spn, f->dtc.spn, f->dtc.fmi, f->dtc.cm, f->dtc.oc, f->dtc.mil, f->dtc.rsl, f->dtc.awl, f->dtc.pl);
    }
}

bool check_dtcs(uint32_t timestamp) {
    bool ret = false; 
    if(take_dtc_mutex()) {
        remove_inactive_dtcs(timestamp);
        remove_incomplete_multi_frame_message(timestamp);
        
        if(changed_dtc_list) {
            // Notify if configured callback function
            if (updated_active_dtcs_callback != NULL) {
                updated_active_dtcs_callback(active_dtcs, active_dtcs_count);
            }
            changed_dtc_list = false;
            ret = true;
        }
        give_dtc_mutex();
    }
    return ret;
}

void clear_dtcs() {
    if(take_dtc_mutex()) {
        candidate_dtcs_count = 0;
        active_dtcs_count = 0;
        memset((void*)candidate_dtcs, 0, sizeof(candidate_dtcs));
        memset((void*)active_dtcs, 0, sizeof(active_dtcs));
        memset((void*)multi_frame_messages, 0, sizeof(multi_frame_messages));
        give_dtc_mutex();
    }
}

bool copy_dtcs(DTC_Info_t* buf_dtc_list, uint16_t buf_size, uint8_t* dtc_count) {
    if(take_dtc_mutex()) {
        uint16_t dtcRequiredBufSize = active_dtcs_count * sizeof(DTC_Info_t);
        if(buf_size >= dtcRequiredBufSize) {
            memcpy((void*)buf_dtc_list, (void*)active_dtcs, dtcRequiredBufSize);
            *dtc_count = active_dtcs_count;
            give_dtc_mutex();
            return true;
        }
    }
    return false;
}

bool dynamic_copy_dtcs(DTC_Info_t **buf_dtc_list, uint8_t* dtc_count) {
    if(take_dtc_mutex()) {
        *buf_dtc_list = (DTC_Info_t*)malloc(active_dtcs_count * sizeof(DTC_Info_t));
        if(*buf_dtc_list != NULL) {
            memcpy((void*)*buf_dtc_list, (void*)active_dtcs, active_dtcs_count * sizeof(DTC_Info_t));
            *dtc_count = active_dtcs_count;
            give_dtc_mutex();
            return true;
        }
        give_dtc_mutex();
    }
    return false;
}

const DTC_Info_t* get_reference_to_dtcs(uint8_t* dtc_count) {
    *dtc_count = active_dtcs_count;
    return (const DTC_Info_t*)active_dtcs;
}