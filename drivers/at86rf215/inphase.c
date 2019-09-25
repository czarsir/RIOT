/***
 *
 * InPhase
 *
 */

/*** Base ***/
#include <string.h>
#include <xtimer.h>

/*** Driver ***/

/*** Self ***/
#include "inphase.h"

#define ENABLE_DEBUG (1)
#include "debug.h"
#define PRINTF DEBUG

/********* Variables *********/

/*** state ***/
volatile fsm_state_t fsm_state = IDLE;
static Settings settings;
static uint8_t status_code;
static uint8_t next_status_code;
static uint8_t retransmissions;
static uint16_t next_result_start;

/*** Sync ***/
volatile uint8_t sigSync;
volatile uint8_t sigSync_test = 0;
volatile uint8_t sigSync_i;

/*** Buffer ***/
uint8_t fbRx[FRAME_BUFFER_LENGTH];
/* allocate an array for the measurement results */
static uint8_t local_pmu_values[PMU_MEASUREMENTS];
static uint8_t pmuQF[PMU_MEASUREMENTS];
/* reuse buffer to save memory */
static int8_t* signed_local_pmu_values = (int8_t*)local_pmu_values;

/********* Functions *********/

/*** Communication ***/
extern uint8_t inphase_connection_init(void);
extern uint8_t inphase_connection_send(uint16_t dest, uint8_t msg_len, void *msg);
extern uint8_t inphase_connection_send_lite(uint16_t dest, uint8_t msg_len, void *msg);
extern uint8_t inphase_connection_close(void);
/*** Sync ***/
extern void sync_config_initiator(void);
extern void sync_config_reflector(void);
extern void sync_config_common(void);
extern void sync_state(void);
extern void sync_config_clean(void);
/*** PMU ***/
extern void backup_registers(void);
extern void restore_registers(void);
extern void mode_config(void);
extern void transmission_config(void);
extern void config_clean(void);
extern void setFrequency(uint16_t f, uint8_t offset);
extern void sender_pmu(void);
extern void receiver_pmu(uint8_t* pmu_value, uint8_t* pmuQF);
/*** Timer ***/
extern void init_timer(void);
extern void start_timer(unsigned int value);
extern void wait_for_timer(uint8_t id);
extern void stop_timer(void);
/*** Serial ***/
extern void rs232_send(uint8_t data);

/********* Special *********/

/*** Mediator ***/
static const InphaseConnection conn = {
	inphase_connection_init,
	inphase_connection_send,
	inphase_connection_send_lite,
	inphase_connection_close
};


/********* Application *******************************************************/

/********* Protocol *********/

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
	memcpy(f.content.result_confirm.result_data, &local_pmu_values[start_address], result_length);
	// TODO sizeof(struct) could be unpredictable.
	conn.send(settings.initiator, sizeof(frame_result_confirm_t)+1, &f);
}

/********* Serial Output *********/

static void send_escaped(uint8_t data)
{
	switch(data) {
		case BINARY_FRAME_START:
		case BINARY_FRAME_END:
		case BINARY_ESCAPE_BYTE:
			rs232_send(BINARY_ESCAPE_BYTE);
			data -= BINARY_ESCAPE_ADD;
			// no break here
		default:
			rs232_send(data);
			break;
	}
}

static void binary_start_frame(void)
{
	rs232_send(BINARY_FRAME_START);
}

static void binary_end_frame(void)
{
	rs232_send(BINARY_FRAME_END);
}

static void binary_send_byte(uint8_t data)
{
	send_escaped(data);
}

static void binary_send_short(uint16_t data)
{
	send_escaped((data >> 8) & 0xFF);
	send_escaped(data & 0xFF);
}

static void binary_send_data(uint8_t* data, uint8_t length)
{
	uint8_t i;
	for (i = 0; i < length; i++) {
		send_escaped(data[i]);
	}
}

void binary_send_frame(uint8_t* frame, uint8_t length)
{
	binary_start_frame();
	binary_send_data(frame, length);
	binary_end_frame();
}

static void send_serial(void)
{
	puts("serial:start\n");
	binary_start_frame();

	// send number of samples per frequency
	binary_send_byte(1); // only one sample is transmitted it is already averaged

	// send step size
	binary_send_byte(0); // 0 is parsed as 500 kHz

	// send start frequency
	//binary_send_short(PMU_START_FREQUENCY);
	binary_send_short(2400);

	// send total amount of samples
	binary_send_short(PMU_MEASUREMENTS);

	// send reflector address
	binary_send_short(settings.reflector);

	// send calculated distance meter
	//binary_send_byte(dist_last_meter);
	binary_send_byte(0);

	// send calculated distance centimeter
	//binary_send_byte(dist_last_centimeter);
	binary_send_byte(0);

	// send last quality
	//binary_send_byte(dist_last_quality);
	binary_send_byte(0);

	// send system status
	binary_send_byte(status_code);

	// transmit data
	binary_send_data(local_pmu_values, PMU_MEASUREMENTS);

	binary_end_frame();
	puts("serial:end\n");
}

/********* Data *********/

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

int16_t inphase_calculation_ret(void)
{
	/*** c = 3e8, f = 0.5e6 ***/
	/*** phi*c / (f*2*pi) = 300 * phi / pi ***/
	int16_t phi = (int16_t)(signed_local_pmu_values[1]) - (int16_t)(signed_local_pmu_values[0]);
	int16_t ret = 300 * phi / 3.14;
	return ret;
}

/********* Sync *********/

static int8_t wait_for_sync(void)
{
	DEBUG("[inphase] sync: wait\n");
	while(sigSync) {}
	DEBUG("[inphase] sync: done\n");

	return 0;
}

/********* PMU *********/

static void pmu_magic_mode_classic(pmu_magic_role_t role)
{
	uint8_t i;
	for (i=sigSync_i; i < PMU_MEASUREMENTS; i++) {
		switch (role) {
			case PMU_MAGIC_ROLE_INITIATOR:		// initiator
				setFrequency(PMU_START_FREQUENCY + i, 0);
				receiver_pmu(&local_pmu_values[i], &pmuQF[i]);
				sender_pmu();
				break;
			default:	// reflector
				setFrequency(PMU_START_FREQUENCY + i, 0);
				sender_pmu();
				receiver_pmu(&local_pmu_values[i], &pmuQF[i]);
				break;
		}

		wait_for_timer(5);

		sigSync_i = i+1;
		if ( (sigSync_i%100) == 0 ) {
			break;
		}
	}
}

static int8_t pmu_magic(pmu_magic_role_t role, pmu_magic_mode_t mode)
{
	int8_t ret_val;

	switch (role) {
		case PMU_MAGIC_ROLE_INITIATOR:
			PRINTF("[inphase] entered PMU Initiator\n");
			break;
		case PMU_MAGIC_ROLE_REFLECTOR:
			PRINTF("[inphase] entered PMU Reflector\n");
			break;
		default:	// unknown role
			PRINTF("[inphase] WARNING unknown role selected. Continue as Reflector\n");
			break;
	}
	//PRINTF("[inphase] PMU mode 0x%x\n", (uint8_t) mode);

	/* Boundary */
	//AT86RF233_ENTER_CRITICAL_REGION();

	//watchdog_stop();
	backup_registers();

	/****** Init ******/

	init_timer();

	sigSync_i = 0;
SYNC:
	// TODO ding
	/*** Sync config ***/
	switch (role) {
		case PMU_MAGIC_ROLE_INITIATOR:
			sync_config_initiator();
			break;
		case PMU_MAGIC_ROLE_REFLECTOR:
			sync_config_reflector();
			break;
		default:
			break;
	}
	sync_config_common();
	sigSync = 1;

	/*** Frequency ***/
	// switch to a frequency where we are not likely being disturbed during synchronization
	setFrequency(PMU_START_FREQUENCY, 0);

	/****** Sync ******/

	/*** for Debug ***/
	//gpio_set(GPIO_PIN(PORT_B, 9));

	/* Initiator */
	sync_state();

	/* reflector sends the synchronization frame */
	if (role == PMU_MAGIC_ROLE_REFLECTOR) {
		/*** send PMU start ***/
		frame_range_basic_t f;
		f.frame_type = PMU_START;
		conn.send_lite(settings.initiator, 1, &f);
	}

	/* wait for sync signal */
	if (wait_for_sync()) {
		ret_val = -1; // signal not seen, abort!
		goto BAIL;
	}
	config_clean();
	/*** for Debug ***/
	//sigSync_test = 1;

	/****** Sync (done) ******/

	//start_timer(); // obsolete, but reserve.

	// now in sync with the other node

	if (role == PMU_MAGIC_ROLE_INITIATOR) {
		// check if the initiator got the TIME_SYNC_REQUEST back
		// reflector cannot check if sync was correct, it will do the measurement anyway
		// initiator can choose the next reflector and save time
		// reflector will not disturb the next measurement of the reflector
//		uint8_t fb_data[5];
//		hal_sram_read(12, 5, fb_data);
//
//		uint8_t valid = 1;
//		if (fb_data[0] != settings.reflector.u8[0]) {
//			valid = 0;
//		}
//		if (fb_data[1] != settings.reflector.u8[1]) {
//			valid = 0;
//		}
//		if (fb_data[2] != linkaddr_node_addr.u8[0]) {
//			valid = 0;
//		}
//		if (fb_data[3] != linkaddr_node_addr.u8[1]) {
//			valid = 0;
//		}
//		if (fb_data[4] != TIME_SYNC_REQUEST) {
//			valid = 0;
//		}
//		if (valid == 0) {
//			ret_val = -2; // synchonization was done with wrong frame
//			goto BAIL;
//		}
	}

	mode_config();

	wait_for_timer(1);

	transmission_config();

	//wait_for_timer(2);

	/* measure RSSI (initiator) */

	wait_for_timer(3);

	/* measure RSSI (reflector) */

	/* TODO: set gain according to rssi */

	wait_for_timer(4);

	switch(mode) {
		case PMU_MAGIC_MODE_CLASSIC:
			pmu_magic_mode_classic(role);
			break;
		default:
			pmu_magic_mode_classic(role);
			break;
	}
	config_clean();
	if (sigSync_i < PMU_MEASUREMENTS) {
		stop_timer();
		goto SYNC;
	}
	ret_val = 0;
	/*** test ***/
	sigSync_test = 0;

BAIL:

	stop_timer();
	//at86rf215_reset(pDev);
	restore_registers();
	//watchdog_start();

	/* Boundary */
	//AT86RF233_LEAVE_CRITICAL_REGION();

	return ret_val;
}

/********* Process *********/

static void reset_statemachine(void)
{
	fsm_state = IDLE;
	status_code = next_status_code;
}

/*** Protocol - state machine ***/
void statemachine(uint8_t frame_type, frame_subframe_t *frame)
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
					xtimer_sleep(1);
					next_result_start = 0;
					send_result_request(next_result_start);
					//DEBUG("[inphase] send %u\n", next_result_start);
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

                DEBUG("[inphase] PMU:");
                for(int i=0; i<PMU_MEASUREMENTS; i++) {
                    DEBUG(" %d", local_pmu_values[i]);
                }
                DEBUG("\n");

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
					DEBUG("[inphase] done.\n");
					DEBUG("[inphase] PMU-QF:");
					for(int i=0; i<PMU_MEASUREMENTS; i++) {
						DEBUG(" %d", pmuQF[i]);
					}
					DEBUG("\n");
					send_serial();
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
						DEBUG("[inphase] done. %u\n", start_address);
						DEBUG("[inphase] PMU:");
						for(int i=0; i<PMU_MEASUREMENTS; i++) {
							DEBUG(" %d", local_pmu_values[i]);
						}
						DEBUG("\n");
						DEBUG("[inphase] PMU-QF:");
						for(int i=0; i<PMU_MEASUREMENTS; i++) {
						    DEBUG(" %d", pmuQF[i]);
						}
						DEBUG("\n");

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

void inphase_receive(const uint16_t *src, uint16_t msg_len, void *msg)
{
	frame_range_basic_t *frame_basic = msg;
	uint8_t msg_accepted = 0;

	//PRINTF("[inphase] inphase_receive: 0x%x\n", frame_basic->frame_type);

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
			//frame_result_confirm_t *frame_result = &frame_basic->content.result_confirm;
			if (msg_len == sizeof(frame_result_confirm_t)+1) {
				// correct message length
				msg_accepted = 1;
			}
			break;
			}
		default:
			// message type unknown!
			break;
	}

	/* test */
	settings.allow_ranging = 1;

	/*** process ***/
	if (msg_accepted) {
		statemachine(frame_basic->frame_type, &frame_basic->content);
	} else {
		//PRINTF("[inphase] inphase_receive: discard.\n");
	}
}
