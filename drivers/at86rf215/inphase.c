/***
 *
 * InPhase
 *
 */

/*** Base ***/
#include <string.h>

/*** driver ***/
#include "at86rf215.h"
//#include "at86rf215_netdev.h"
//#include "at86rf215_internal.h"
//#include "at86rf215_registers.h"

#define ENABLE_DEBUG (1)
#include "debug.h"
#define PRINTF DEBUG


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
#define PMU_MEASUREMENTS      200  // number of frequencies to measure

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

enum fsm_states {
	IDLE,

	/* Initiator States */
	RANGING_REQUESTED,
	RESULT_REQUESTED,

	/* Reflector States */
	RANGING_ACCEPTED,
	WAIT_FOR_RESULT_REQ,
};

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

/*** device ***/
static at86rf2xx_t *pDev;

/*** state ***/
volatile enum fsm_states fsm_state = IDLE;
Settings settings;
uint8_t status_code;
uint8_t next_status_code;
uint8_t retransmissions;
uint8_t next_result_start;

/*** Buffer ***/
static uint8_t fbRx[FRAME_BUFFER_LENGTH];
/* allocate an array for the measurement results */
uint8_t local_pmu_values[PMU_MEASUREMENTS];
/* reuse buffer to save memory */
int8_t* signed_local_pmu_values = (int8_t*)local_pmu_values;

/********* Functions *********/



/********* Application *******************************************************/

/*** TODO ***/
static uint8_t inphase_connection_init(void)
{
	return 0;
}

/*** TODO ***/
static uint8_t inphase_connection_send(uint16_t dest, uint8_t msg_len, void *msg)
{
	(void)dest;
	DEBUG("[inphase] send\n");
	at86rf2xx_send(pDev, msg, msg_len);
	return 0;
}

/*** TODO ***/
static uint8_t inphase_connection_close(void)
{
	return 0;
}

static const InphaseConnection conn = {
	inphase_connection_init,
	inphase_connection_send,
	inphase_connection_close
};

/********* Communication *********/

static void send_range_request(void)
{
	// send RANGE_REQUEST
	frame_range_basic_t f;
	f.frame_type = RANGE_REQUEST;
	f.content.range_request.ranging_method = RANGING_METHOD_PMU;
	f.content.range_request.capabilities = 0x00;
	conn.send(settings.reflector, sizeof(frame_range_request_t)+1, &f);
}

static void send_range_accept(void)
{
	// send RANGE_ACCEPT
	frame_range_basic_t f;
	f.frame_type = RANGE_ACCEPT;
	f.content.range_accept.ranging_accept = RANGE_ACCEPT_STATUS_SUCCESS;
	f.content.range_accept.reject_reason = 0;
	f.content.range_accept.accepted_ranging_method = RANGING_METHOD_PMU;
	f.content.range_accept.accepted_capabilities = 0x00;
	conn.send(settings.initiator, sizeof(frame_range_accept_t)+1, &f);
}

static void send_time_sync_request(void)
{
	// send TIME_SYNC_REQUEST
	frame_range_basic_t f;
	f.frame_type = TIME_SYNC_REQUEST;
	conn.send(settings.reflector, 1, &f);
}

static void send_result_request(uint16_t start_address)
{
	// send RESULT_REQUEST
	frame_range_basic_t f;
	f.frame_type = RESULT_REQUEST;
	f.content.result_request.result_data_type = RESULT_TYPE_PMU;
	f.content.result_request.result_start_address = start_address;
	conn.send(settings.reflector, sizeof(frame_result_request_t)+1, &f);
}

static void send_result_confirm(uint16_t start_address, uint16_t result_length)
{
	// send RESULT_CONFIRM
	frame_range_basic_t f;
	f.frame_type = RESULT_CONFIRM;
	f.content.result_confirm.result_data_type = RESULT_TYPE_PMU;
	f.content.result_confirm.result_start_address = start_address;
	f.content.result_confirm.result_length = result_length;
	memcpy(&f.content.result_confirm.result_data, &local_pmu_values[start_address], result_length);
	conn.send(settings.initiator, result_length+5, &f);
}

static void active_reflector_subtract(uint16_t last_start,
									  uint8_t *result_data, uint16_t result_length)
{
	uint16_t i;
	for (i = 0; i < result_length; i++) {
		// do basic calculations to save memory
		int16_t v = local_pmu_values[i+last_start]-result_data[i];
		
		if (v > 127) {
			v -= 256;
		} else if (v < -128) {
			v += 256;
		}
		// overwrite data in local array
		signed_local_pmu_values[i+last_start] = (int8_t) v;
	}
}

/********* PMU *********/
static int8_t pmu_magic(pmu_magic_role_t role, pmu_magic_mode_t mode)
{
	(void)role;
	(void)mode;
	return 0;
}

/********* Process *********/

static void reset_statemachine(void)
{
	fsm_state = IDLE;
	status_code = next_status_code;
}

static void statemachine(uint8_t frame_type, frame_subframe_t *frame)
{
	PRINTF("[inphase] frame_type: 0x%x, fsm_state: %u\n", frame_type, fsm_state);

	switch (fsm_state) {
		case IDLE:
		{
			/*** initiator - start ***/
			if (frame_type == RANGE_REQUEST_START) {
				status_code = DISTANCE_RUNNING;
				send_range_request();
				/* maximum allowed retransmissions */
				retransmissions = RANGE_REQUEST_RETRANSMISSIONS;
				//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, trigger_network_timeout, NULL);
				fsm_state = RANGING_REQUESTED;
			}
			/*** reflector - start ***/
			else if (frame_type == RANGE_REQUEST) {
				/* check if ranging is allowed */
				if (!settings.allow_ranging) {
					PRINTF("[inphase] ranging request ignored (ranging not allowed)\n");
					fsm_state = IDLE;
				} else {
					status_code = DISTANCE_RUNNING;
					send_range_accept();
					next_status_code = DISTANCE_TIMEOUT;
					//ctimer_set(&timeout_timer, REFLECTOR_TIMEOUT, reset_statemachine, NULL);
					fsm_state = RANGING_ACCEPTED;
				}
			}
			/* all other frames are invalid here */
			else {
				status_code = DISTANCE_IDLE;
			}
			break;
		}

		/*** initiator states ***/
		case RANGING_REQUESTED:
		{
			if (frame_type == NETWORK_TIMEOUT) {
				if (retransmissions > 0) {
					PRINTF("retransmit RANGE_REQUEST\n");
					send_range_request();
					retransmissions--;
					//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, trigger_network_timeout, NULL);
				} else {
					// too many retransmissions, abort ranging
					next_status_code = DISTANCE_NO_REFLECTOR;
					reset_statemachine();
				}
			} else if (frame_type == RANGE_ACCEPT) {
				//ctimer_stop(&timeout_timer); // stop timer for pmu_magic

				send_time_sync_request();

				// NOTE: pmu_magic at reflector sends sync frame as response to TIME_SYMC_REQUEST
				int8_t pmu_magic_result = pmu_magic(PMU_MAGIC_ROLE_INITIATOR, PMU_MAGIC_MODE_CLASSIC);
				if (pmu_magic_result == -1) {
					next_status_code = DISTANCE_NO_SYNC;
					reset_statemachine(); // DIG2 timed out, abort!
				} else if (pmu_magic_result == -2) {
					next_status_code = DISTANCE_WRONG_SYNC;
					reset_statemachine(); // synced to wrong frame, abort!
				} else {
					next_result_start = 0;
					send_result_request(next_result_start);
					retransmissions = RESULT_REQUEST_RETRANSMISSIONS; // maximum allowed retransmissions
					//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, trigger_network_timeout, NULL);
					fsm_state = RESULT_REQUESTED;
				}
			} else {
				/* all other frames are invalid here */
			}
			break;
		}
		case RESULT_REQUESTED:
		{
			if (frame_type == NETWORK_TIMEOUT) {
				if (retransmissions > 0) {
					PRINTF("retransmit RESULT_REQUEST\n");
					send_result_request(next_result_start);
					retransmissions--;
					//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, trigger_network_timeout, NULL);
				} else {
					// too many retransmissions, abort ranging
					next_status_code = DISTANCE_TIMEOUT;
					reset_statemachine();
				}
			} else if (frame_type == RESULT_CONFIRM) {
				//ctimer_stop(&timeout_timer);

				// get last results from frame
				uint16_t last_start = frame->result_confirm.result_start_address;
				uint16_t result_length = frame->result_confirm.result_length;

				active_reflector_subtract(last_start, frame->result_confirm.result_data, result_length);

				next_result_start = last_start + result_length;

				send_result_request(next_result_start);
				retransmissions = RESULT_REQUEST_RETRANSMISSIONS;

				if (next_result_start < PMU_MEASUREMENTS) {
					// more data to receive
					//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, trigger_network_timeout, NULL);
					fsm_state = RESULT_REQUESTED;
				} else {
					// got all results, finished
					fsm_state = IDLE;
				}
			} else {
				/* all other frames are invalid here */
			}
			break;
		}
			
		/*** reflector states ***/
		case RANGING_ACCEPTED:
		{
			if (frame_type == RANGE_REQUEST) {
				PRINTF("got duplicate RANGE_REQUEST, answering anyway...\n");
				status_code = DISTANCE_RUNNING;
				send_range_accept();
				next_status_code = DISTANCE_TIMEOUT;
				//ctimer_set(&timeout_timer, REQUEST_TIMEOUT, reset_statemachine, NULL);
				fsm_state = RANGING_ACCEPTED;
			} else if (frame_type == TIME_SYNC_REQUEST) {
				//ctimer_stop(&timeout_timer); // stop timer for pmu_magic

				int8_t pmu_magic_result = pmu_magic(PMU_MAGIC_ROLE_REFLECTOR, PMU_MAGIC_MODE_CLASSIC);
				if (pmu_magic_result) {
					next_status_code = DISTANCE_NO_SYNC;
					reset_statemachine(); // DIG2 timed out, abort!
				} else {
					next_status_code = DISTANCE_TIMEOUT;
					//ctimer_set(&timeout_timer, REFLECTOR_TIMEOUT, reset_statemachine, NULL); // magic finished, restart timer
					fsm_state = WAIT_FOR_RESULT_REQ;
				}
			} else {
				/* all other frames are invalid here */
			}
			break;
		}
		case WAIT_FOR_RESULT_REQ:
		{
			if (frame_type == RESULT_REQUEST) {
				next_status_code = DISTANCE_TIMEOUT;
				//ctimer_stop(&timeout_timer); // partner sent valid next frame, stop timer

				if (frame->result_request.result_data_type == RESULT_TYPE_PMU) {
					// send a pmu result frame
					uint16_t start_address = frame->result_request.result_start_address;
					if (start_address >= PMU_MEASUREMENTS) {
						// start address points outside of the pmu data, this indicates that the initiator does not need more data
						fsm_state = IDLE;
						status_code = DISTANCE_IDLE;
					} else {
						// initiator still needs results
						uint8_t result_length;
						if ((PMU_MEASUREMENTS - start_address) > RESULT_DATA_LENGTH) {
							result_length = RESULT_DATA_LENGTH;
						} else {
							result_length = PMU_MEASUREMENTS - start_address;
						}
						
						send_result_confirm(start_address, result_length);
						
						// keep using REFLECTOR_TIMEOUT, because initiator might take longer time to process results
						//ctimer_restart(&timeout_timer); // partner sent valid next frame, reset timer
						
						fsm_state = WAIT_FOR_RESULT_REQ; // wait for more result requests
					}
				} else {
					// this can be an RSSI result...
					fsm_state = IDLE; // no other results allowed, return to idle
					status_code = DISTANCE_IDLE;
				}
			} else if (frame_type == RANGE_REQUEST) {
				// allow new measurement, even when waiting for results to be transmitted
				// maybe we lost the last "invalid" RESULT_REQUEST and the same initiator want to measure again
				// we do the same as in the IDLE state
				
				// check if ranging is allowed
				if (!settings.allow_ranging) {
					PRINTF("DISTANCE: ranging request ignored (ranging not allowed)\n");
					fsm_state = IDLE;
				} else {
					PRINTF("returning early from WAIT_FOR_RESULT_REQ, got new RANGE_REQUEST\n");
					status_code = DISTANCE_RUNNING;
					send_range_accept();
					next_status_code = DISTANCE_TIMEOUT;
					//ctimer_set(&timeout_timer, REFLECTOR_TIMEOUT, reset_statemachine, NULL);
					fsm_state = RANGING_ACCEPTED;
				}
			} else {
				/* all other frames are invalid here */
			}
			break;
		}
			
		default:
			break;
	}
}

static void inphase_receive(const uint16_t *src, uint16_t msg_len, void *msg)
{
	frame_range_basic_t *frame_basic = msg;
	uint8_t msg_accepted = 0;

	PRINTF("[inphase] inphase_receive: message received!\n");

	switch (frame_basic->frame_type) {
		case RANGE_REQUEST:
			if (msg_len == sizeof(frame_range_request_t)+1) {
				// correct message length
				msg_accepted = 1;
				//linkaddr_copy(&settings.initiator, src);
				settings.initiator = *src;
			}
			break;
		case RANGE_ACCEPT:
			if (msg_len == sizeof(frame_range_accept_t)+1) {
				// correct message length
				msg_accepted = 1;
			}
			break;
		case TIME_SYNC_REQUEST:
			if (msg_len == 1) {
				// correct message length
				msg_accepted = 1;
			}
			break;
		case PMU_START:
			if (msg_len == 1) {
				// correct message length
				msg_accepted = 1;
			}
			break;
		case RESULT_REQUEST:
			if (msg_len == sizeof(frame_result_request_t)+1) {
				// correct message length
				msg_accepted = 1;
			}
			break;
		case RESULT_CONFIRM: {
			frame_result_confirm_t *frame_result = &frame_basic->content.result_confirm;
			if (msg_len == (frame_result->result_length + 5)) {
				// correct message length
				msg_accepted = 1;
			}
			break;
			}
		default:
			// message type unknown!
			break;
	}

	/*** process ***/
	if (msg_accepted) {
		statemachine(frame_basic->frame_type, &frame_basic->content);
	}
}

void inphase_isr(at86rf2xx_t *dev)
{
	DEBUG("[inphase] inphase_isr\n");

	uint16_t len = at86rf215_receive(dev, fbRx, FRAME_BUFFER_LENGTH);
	uint16_t src = 0;
	inphase_receive(&src, len, fbRx);
}

void inphase_start(at86rf2xx_t *dev)
{
	DEBUG("[inphase] inphase_start\n");

	pDev = dev;

	fsm_state = IDLE; // reset state machine
	statemachine(RANGE_REQUEST_START, NULL);
}
