/***
 *
 * InPhase
 *
 */
#ifndef _INPHASE_H_
#define _INPHASE_H_

#ifdef __cplusplus
extern "C" {
#endif

/********* Macro *********/

/*** Command ***/
#define RANGE_REQUEST_START      0x00
#define RANGE_REQUEST            0x01
#define RANGE_ACCEPT             0x02
#define TIME_SYNC_REQUEST        0x11
#define PMU_START                0x12
#define RESULT_REQUEST           0x21
#define RESULT_CONFIRM           0x22
#define NETWORK_TIMEOUT          0xFF

/*** Status ***/
#define RANGE_ACCEPT_STATUS_SUCCESS 0x10
#define RANGE_ACCEPT_STATUS_REJECT  0x12
#define RESULT_TYPE_PMU          0x00
#define RESULT_TYPE_RSSI         0x01

/*** Config ***/
#define RESULT_DATA_LENGTH       100  // number of byte to send in one result frame
#define FRAME_BUFFER_LENGTH      150
/* Method */
#define RANGING_METHOD_PMU       0x01
/* Retransmission */
#define RANGE_REQUEST_RETRANSMISSIONS  2
#define RESULT_REQUEST_RETRANSMISSIONS 2

/*** PMU ***/
//#define PMU_START_FREQUENCY   2400  // start frequency for measurement
#define PMU_START_FREQUENCY   0
//#define PMU_MEASUREMENTS      200   // number of frequencies to measure
#define PMU_MEASUREMENTS      5

/*** Status codes (for SENSORS_READY) ***/
#define DISTANCE_INVALID      0  // no measurement running, idle, ready for measurement
#define DISTANCE_IDLE         1  // measurement is currently running
#define DISTANCE_RUNNING      2  // measurement failed to unknown reasons
#define DISTANCE_FAILED       3  // no AT86RF233 radio, measurement not possible
#define DISTANCE_NO_RF233     4  // communication with reflector timed out
#define DISTANCE_TIMEOUT      5  // reflector doesn't answer to ranging request
#define DISTANCE_NO_REFLECTOR 6  // timing synchronization failed
#define DISTANCE_NO_SYNC      7  // distance calculation returned wrong value
#define DISTANCE_VALUE_ERROR  8  // synced to wrong frame (not the one send by reflector)
#define DISTANCE_WRONG_SYNC   9


/********* Type Definition *********/

typedef enum {
	IDLE,
	
	/* Initiator States */
	RANGING_REQUESTED,
	RESULT_REQUESTED,
	
	/* Reflector States */
	RANGING_ACCEPTED,
	WAIT_FOR_RESULT_REQ,
} fsm_state_t;

// TODO linkaddr_t -> uint16_t
typedef struct {
	uint16_t reflector;		// address of the reflector
	uint16_t initiator;		// address of the initiator that issued the current measurement
	uint8_t raw_output;			// if 1, output PMU data to serial port
	uint8_t allow_ranging;
	uint8_t compute;
	uint8_t interpolate;
	uint16_t offset;
} Settings;

typedef struct {
	uint8_t (*init)(void);
	uint8_t (*send)(uint16_t dest, uint8_t msg_len, void *msg);
	uint8_t (*send_lite)(uint16_t dest, uint8_t msg_len, void *msg);
	uint8_t (*close)(void);
} InphaseConnection;

/*** PMU ***/

typedef enum {
	PMU_MAGIC_ROLE_INITIATOR,
	PMU_MAGIC_ROLE_REFLECTOR
} pmu_magic_role_t;

typedef enum {
	PMU_MAGIC_MODE_CLASSIC
} pmu_magic_mode_t;

/*** Frame ***/

typedef struct {
	uint8_t  ranging_method;
	uint8_t  capabilities;
} frame_range_request_t;

typedef struct {
	uint8_t ranging_accept;
	uint8_t reject_reason;
	uint8_t accepted_ranging_method;
	uint8_t accepted_capabilities;
} frame_range_accept_t;

typedef struct {
	uint8_t result_data_type;
	uint16_t result_start_address;
} frame_result_request_t;

typedef struct {
	uint8_t  result_data_type;
	uint16_t result_start_address;
	uint8_t result_length;
	uint8_t result_data[RESULT_DATA_LENGTH];
} frame_result_confirm_t;

typedef union {
	frame_range_request_t range_request;
	frame_range_accept_t range_accept;
	frame_result_request_t result_request;
	frame_result_confirm_t result_confirm;
} frame_subframe_t;

typedef struct {
	uint8_t frame_type;
	frame_subframe_t content;
} frame_range_basic_t;

/********* Variables *********/
/* device */
extern at86rf2xx_t *pDev;
/* state */
extern volatile fsm_state_t fsm_state;
/* buffer */
extern uint8_t fbRx[FRAME_BUFFER_LENGTH];

/********* Functions *********/
extern void statemachine(uint8_t frame_type, frame_subframe_t *frame);
extern void inphase_receive(const uint16_t *src, uint16_t msg_len, void *msg);

#ifdef __cplusplus
}
#endif

#endif
