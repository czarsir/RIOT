/***
 *
 * InPhase
 *
 */

/*** Base ***/
#include <string.h>
#include <xtimer.h>
#include "periph/uart.h"

/*** Driver ***/
#include "at86rf215.h"
//#include "at86rf215_netdev.h"
#include "at86rf215_internal.h"
#include "at86rf215_registers.h"

/*** Self ***/
#include "inphase.h"

#define ENABLE_DEBUG (0)
#include "debug.h"
#define PRINTF DEBUG

/********* Variables *********/

/*** device ***/
at86rf2xx_t *pDev;

/*** state ***/
volatile fsm_state_t fsm_state = IDLE;
static Settings settings;
static uint8_t status_code;
static uint8_t next_status_code;
static uint8_t retransmissions;
static uint8_t next_result_start;

/*** Sync ***/
volatile uint8_t sigSync;

/*** Buffer ***/
uint8_t fbRx[FRAME_BUFFER_LENGTH];
/* allocate an array for the measurement results */
static uint8_t local_pmu_values[PMU_MEASUREMENTS];
/* reuse buffer to save memory */
static int8_t* signed_local_pmu_values = (int8_t*)local_pmu_values;

/********* Functions *********/

/*** Connection ***/
extern uint8_t inphase_connection_init(void);
extern uint8_t inphase_connection_send(uint16_t dest, uint8_t msg_len, void *msg);
extern uint8_t inphase_connection_send_lite(uint16_t dest, uint8_t msg_len, void *msg);
extern uint8_t inphase_connection_close(void);
/*** Timer ***/
extern void init_timer(void);
extern void start_timer(void);
extern void wait_for_timer(uint8_t id);
extern void stop_timer(void);

/********* Special *********/

static const InphaseConnection conn = {
	inphase_connection_init,
	inphase_connection_send,
	inphase_connection_send_lite,
	inphase_connection_close
};


/********* Application *******************************************************/

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

/********* Output *********/

static uint8_t bufByte;

static void rs232_send(uint8_t data)
{
	bufByte = data;
	uart_write(UART_DEV(0), (const uint8_t *)&bufByte, (size_t)1);
}

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

void binary_start_frame(void)
{
	rs232_send(BINARY_FRAME_START);
}

void binary_end_frame(void)
{
	rs232_send(BINARY_FRAME_END);
}

void binary_send_byte(uint8_t data)
{
	send_escaped(data);
}

void binary_send_short(uint16_t data)
{
	send_escaped((data >> 8) & 0xFF);
	send_escaped(data & 0xFF);
}

void binary_send_data(uint8_t* data, uint8_t length)
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

/********* Sync *********/

static int8_t wait_for_dig2(void)
{
	DEBUG("[inphase] sync: wait\n");
	while(sigSync) {}
	DEBUG("[inphase] sync: done\n");

	return 0;
}

/********* PMU *********/

/*** State ***/
static uint8_t preState;
/*** Frequency ***/
static uint8_t rfCS;
static uint8_t rfCCF0L;
static uint8_t rfCCF0H;
static uint8_t rfCNL;
/*** Interrupt ***/
static uint8_t bbcIRQ;

static void backup_registers(void)
{
	/*** State ***/
	preState = at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);

	/*** Frequency ***/
	rfCS = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CS);
	rfCCF0L = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CCF0L);
	rfCCF0H = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CCF0H);
	rfCNL = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CNL);

	/*** Interrupt ***/
	bbcIRQ = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__IRQM);
}

static void restore_registers(void)
{
//	at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);
//
//	/*** Frequency ***/
//	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CS, rfCS);
//	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0L, rfCCF0L);
//	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0H, rfCCF0H);
//	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNL, rfCNL);
//	/* channel scheme */
//	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNM, 0);
//
//	/*** Interrupt ***/
//	at86rf215_reg_read(pDev, AT86RF215_REG__BBC0_IRQS);
//	at86rf215_reg_write(pDev, AT86RF215_REG__BBC0_IRQM, bbcIRQ);
//
//	/*** restore State ***/
//	//at86rf215_set_state(pDev, preState);
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_RX);
}

/*** set the frequency in MHz
 * if offset == 1, the frequency is 0.5 Mhz higher
 * frequency must be between 2322 MHz and 2527 MHz
 */
static void setFrequency(uint16_t f, uint8_t offset)
{
//	if (f < 2322) {
//		// frequency is not supported
//	} else if (f < 2434) {
//		// CC_BAND is 0x8
//		hal_register_write(RG_CC_CTRL_1, 0x08);
//		f -= 2306;			// f is now between 0x10 and 0x7F
//	} else if (f < 2528) {
//		// CC_BAND is 0x9
//		hal_register_write(RG_CC_CTRL_1, 0x09);
//		f -= 2434;			// f is now between 0x00 and 0x5D
//	} else {
//		// frequency is not supported
//	}
//	f = f << 1;				// f is now between 0x00 and 0xFE
//	if (offset) {
//		f += 1;				// f is chosen 0.5 MHz higher (0x01 to 0xFF)
//	}

	(void)offset;
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);

	/*** Channel ***/
	/* 0x14 for 2.4G (Scheme 0) */
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CS, 0x14);
	/* 0x8ca0 for 2.4G (Scheme 0) */
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0L, 0xa0);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0H, 0x8c);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNL, f);
	/* channel scheme */
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNM, 0);
}

static void sender_pmu(void)
{
	at86rf215_reg_write(pDev,  pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TXPREP);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TX);
	xtimer_usleep(85);	// wait for receiver to measure
}

static void receiver_pmu(uint8_t* pmu_value)
{
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TXPREP);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_RX);
	xtimer_usleep(45); // wait for sender to be ready

	*pmu_value = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__PMUVAL);
}

static void pmu_magic_mode_classic(pmu_magic_role_t role)
{
	uint8_t i;
	for (i=0; i < PMU_MEASUREMENTS; i++) {
		// use 500 kHz spacing
//		uint8_t f, f_full, f_half;
		switch (role) {
			case PMU_MAGIC_ROLE_INITIATOR:		// initiator
//				f = i;
//				f_full = f >> 1;
//				f_half = f & 0x01;
				//setFrequency(PMU_START_FREQUENCY + f_full, f_half);
				setFrequency(PMU_START_FREQUENCY + i, 0);
				receiver_pmu(&local_pmu_values[i]);
				sender_pmu();
				break;
			default:	// reflector
//				f = i + 1; // reflector is 500 kHz higher
//				f_full = f >> 1;
//				f_half = f & 0x01;
				//setFrequency(PMU_START_FREQUENCY + f_full, f_half);
				setFrequency(PMU_START_FREQUENCY + i, 0);
				sender_pmu();
				receiver_pmu(&local_pmu_values[i]);
				break;
		}
		wait_for_timer(5);
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
	PRINTF("[inphase] PMU mode 0x%x\n", (uint8_t) mode);

	/* Boundary */
	//AT86RF233_ENTER_CRITICAL_REGION();

	//watchdog_stop();
	backup_registers();

	/****** Init ******/

	init_timer();

	//hal_subregister_write(SR_TX_PWR, 0xF);			// set TX output power to -17dBm to avoid reflections
//	hal_subregister_write(SR_TX_PWR, 0x0);			// set TX output power to +4dBm, MAXIMUM POWER
//	hal_register_read(RG_IRQ_STATUS);				// clear all pending interrupts
//	hal_subregister_write(SR_ARET_TX_TS_EN, 0x1);	// signal frame transmission via DIG2
//	hal_subregister_write(SR_IRQ_2_EXT_EN, 0x1);	// enable time stamping via DIG2

	// this line is normally only done at the reflector:
//	hal_subregister_write(SR_TOM_EN, 0x0);			// disable TOM mode (unclear why this is done here)

	// TODO ding
	/*** Sync config ***/
	switch (role) {
		case PMU_MAGIC_ROLE_INITIATOR:
			at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, AT86RF215_BBCn_IRQM__RXFE_M);
			break;
		case PMU_MAGIC_ROLE_REFLECTOR:
			at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, AT86RF215_BBCn_IRQM__TXFE_M);
			break;
		default:
			break;
	}
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC0_IRQS);
	sigSync = 1;

	/*** Frequency ***/
	// switch to a frequency where we are not likely being disturbed during synchronization
	setFrequency(PMU_START_FREQUENCY, 0);

	/****** Sync ******/

	/* Initiator */
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_RX);

	/* reflector sends the synchronization frame */
	if (role == PMU_MAGIC_ROLE_REFLECTOR) {
		// send PMU start

		// TODO: send the correct frame here, just to sleep well...

		frame_range_basic_t f;
		f.frame_type = PMU_START;
		conn.send_lite(settings.initiator, 1, &f);

		//AT86RF233_NETWORK.send(initiator_requested, 1, &f);

		// send PMU start on bare metal, as the normal protocol stack is too slow for us to be able to see the DIG2 signal
		//packetbuf_copyfrom(&f, 1);
		//packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, &initiator_requested);
		//packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
		//packetbuf_compact();

		//xtimer_usleep(2000); // wait for initiator, it needs more time before it listens to DIG2

//		hal_subregister_write(SR_TRX_CMD, CMD_TX_ARET_ON);
//		hal_set_slptr_high(); // send the packet at the latest time possible
//		hal_set_slptr_low();
	}

	/* wait for sync signal */
	if (wait_for_dig2()) {
		// DIG2 signal not found, abort measurement
		// to be honest: if the reflector does not get the DIG2 from its own sending
		// there must be something horribly wrong...
		ret_val = -1; // DIG2 signal not seen, abort!
		goto BAIL;
	}
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, 0);
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC0_IRQS);

	if (role == PMU_MAGIC_ROLE_REFLECTOR) {
		//xtimer_usleep(9.5243);	// DIG2 signal is on average 9.5243 us delayed on the initiator, reflector waits
	} else {
		xtimer_usleep(1470);
	}

	/****** Sync (done) ******/

	start_timer();		// timer counts to 7, we have 244us between synchronization points

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

	switch (role) {
		case PMU_MAGIC_ROLE_INITIATOR:		// initiator
//			hal_subregister_write(SR_TX_RX, 0);				// RX PLL frequency is selected
			break;
		default:	// reflector
//			hal_subregister_write(SR_PMU_IF_INVERSE, 1);	// Inverse IF position
//			hal_subregister_write(SR_TX_RX, 1);				// TX PLL frequency is selected
			break;
	}

//	hal_subregister_write(SR_RX_PDT_DIS, 1);	// RX Path is disabled
//	hal_subregister_write(SR_PMU_EN, 1);		// enable PMU
//	hal_subregister_write(SR_MOD_SEL, 1);		// manual control of modulation data
//	hal_subregister_write(SR_MOD, 0);			// continuous 0 chips for modulation
//	hal_subregister_write(SR_TX_RX_SEL, 1);		// manual control of PLL frequency mode

	// TODO ding
	/*** PMU ***/
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PMUC, 0x1f);  // PUM enable

	wait_for_timer(1);

#define BUFF_LEN 127
	/*** write 0 to buffer ***/
	uint8_t fb_data[BUFF_LEN] = {0};
	//PRINTF("[inphase] fb_data: 0x%x, 0x%x, 0x%x\n", fb_data[2], fb_data[5], fb_data[7]);
	at86rf215_txfb_write(pDev, 0, fb_data, BUFF_LEN);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__TXFLH, 0);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__TXFLL, BUFF_LEN + 2);
	/* antenna diversity control is skipped, we only have one antenna */

	//wait_for_timer(2);

	/* measure RSSI (initiator) */
//	uint8_t rssi;
//	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
//	switch (role) {
//		case PMU_MAGIC_ROLE_INITIATOR:		// initiator
//			hal_register_write(RG_TRX_STATE, CMD_RX_ON);
//			_delay_us(50); // wait some time for sender to be ready...
//			rssi = hal_subregister_read(SR_RSSI);
//			break;
//		default:	// reflector
//			hal_register_write(RG_TRX_STATE, CMD_TX_START);
//			break;
//	}

	wait_for_timer(3);

	/* measure RSSI (reflector) */
//	hal_register_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);
//	switch (role) {
//		case PMU_MAGIC_ROLE_INITIATOR:		// initiator
//			hal_register_write(RG_TRX_STATE, CMD_TX_START);
//			break;
//		default:	// reflector
//			hal_register_write(RG_TRX_STATE, CMD_RX_ON);
//			_delay_us(50); // wait some time for sender to be ready...
//			rssi = hal_subregister_read(SR_RSSI);
//			break;
//	}

	/* TODO: set gain according to rssi */
//	hal_register_write(RG_TST_AGC, 0x09);

	wait_for_timer(4);

	switch(mode) {
		case PMU_MAGIC_MODE_CLASSIC:
			pmu_magic_mode_classic(role);
			break;
		default:
			pmu_magic_mode_classic(role);
			break;
	}
	ret_val = 0;

BAIL:

	stop_timer();
	at86rf215_reset(pDev);
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
					DEBUG("[inphase] send %u\n", next_result_start);
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
					DEBUG("[inphase] done.\n");
					DEBUG("[inphase] PMU:");
//					for(int i=0; i<PMU_MEASUREMENTS; i++) {
//						DEBUG(" %d", signed_local_pmu_values[i]);
//					}
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

	PRINTF("[inphase] inphase_receive: 0x%x\n", frame_basic->frame_type);

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

	/* test */
	settings.allow_ranging = 1;

	/*** process ***/
	if (msg_accepted) {
		statemachine(frame_basic->frame_type, &frame_basic->content);
	} else {
		PRINTF("[inphase] inphase_receive: discard.\n");
	}
}
