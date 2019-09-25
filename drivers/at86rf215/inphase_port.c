/***
 *
 * InPhase
 *
 */

/*** Base ***/
#include <xtimer.h>
#include "periph/uart.h"

/*** Driver ***/
#include "periph/timer.h"
#include "at86rf215.h"
#include "at86rf215_internal.h"
#include "at86rf215_registers.h"

/*** Self ***/
#include "inphase.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define _TIMER            1
#define _TIMER_CHANNEL    0
#define _TIMER_FREQUENCY  (1000000ul)  // 1MHz <-> 1 us
#define _TIMER_VALUE      (1500)       // 1k <-> 1 ms

/********* Variables *********/

/*** device ***/
static at86rf2xx_t *pDev;

/********* Application *******************************************************/

void inphase_isr(at86rf2xx_t *dev)
{
	DEBUG("[inphase] inphase_isr\n");

	pDev = dev;

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

uint8_t inphase_state(void)
{
	return fsm_state;
}

/********* Communication *********/

uint8_t inphase_connection_init(void)
{
	return 0;
}

uint8_t inphase_connection_send(uint16_t dest, uint8_t msg_len, void *msg)
{
	(void)dest;
	DEBUG("[inphase] send\n");
	at86rf215_send(pDev, msg, msg_len);
	return 0;
}

uint8_t inphase_connection_send_lite(uint16_t dest, uint8_t msg_len, void *msg)
{
	(void)dest;
	DEBUG("[inphase] send\n");
	at86rf215_send_no_tail(pDev, msg, msg_len);
	return 0;
}

uint8_t inphase_connection_close(void)
{
	return 0;
}

/********* Sync *********/

void sync_config_initiator(void)
{
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, AT86RF215_BBCn_IRQM__RXFE_M);
}

void sync_config_reflector(void)
{
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, AT86RF215_BBCn_IRQM__TXFE_M);
}

void sync_config_common(void)
{
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC0_IRQS);
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC1_IRQS);
}

void sync_state(void)
{
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_RX);
}

void sync_config_clean(void)
{
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, 0);
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC0_IRQS);
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC1_IRQS);
}

/********* PMU *********/

/*** State ***/
static uint8_t preState;
static uint8_t preMode;
/*** BBC ***/
static uint8_t bbcPC;
/*** RF ***/
static uint8_t rfRXBWC;
static uint8_t rfRXDFE;
/*** Frequency ***/
static uint8_t rfCS;
static uint8_t rfCCF0L;
static uint8_t rfCCF0H;
static uint8_t rfCNL;
/*** Interrupt ***/
static uint8_t bbcIRQ;

void backup_registers(void)
{
	/*** State ***/
	preState = at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);

	preMode = at86rf215_reg_read(pDev, AT86RF215_REG__RF_IQIFC1);

	/*** BBC ***/
	bbcPC = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__PC);

	/*** RF ***/
	rfRXBWC = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__RXBWC);
	rfRXDFE = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__RXDFE);

	/*** Frequency ***/
	rfCS = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CS);
	rfCCF0L = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CCF0L);
	rfCCF0H = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CCF0H);
	rfCNL = at86rf215_reg_read(pDev, pDev->rf|AT86RF215_REG__CNL);

	/*** Interrupt ***/
	bbcIRQ = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__IRQM);
}

void restore_registers(void)
{
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_TRXOFF);

	at86rf215_reg_write(pDev, AT86RF215_REG__RF_IQIFC1, preMode);

	/*** BBC ***/
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PC, bbcPC);

	/*** RF ***/
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__RXBWC, rfRXBWC);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__RXDFE, rfRXDFE);

	/*** Frequency ***/
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CS, rfCS);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0L, rfCCF0L);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CCF0H, rfCCF0H);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNL, rfCNL);
	/* channel scheme */
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CNM, 0);

	/*** Interrupt ***/
	at86rf215_reg_read(pDev, AT86RF215_REG__BBC1_IRQS);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__IRQM, bbcIRQ);

	//	/*** restore State ***/
	//	//at86rf215_set_state(pDev, preState);
	at86rf215_set_state(pDev, AT86RF215_STATE_RF_RX);
}

void mode_config(void)
{
	// TODO ding
	uint8_t tmp;
	at86rf215_reg_write(pDev, AT86RF215_REG__RF_IQIFC1, 0x12);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PC, 0x04);
	tmp = (0x0 << 4) | (0x8); // 0x7: 800 kHz.
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__RXBWC, tmp);
	/* RCUT | - | SR: RX Sample Rate */
	tmp = (0x4 << 5) | (0x4);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__RXDFE, tmp);
	/*** PMU ***/
	/* CCFTS 1 | IQSEL 1 | FED 0 | SYNC 111 | AVG 0 | EN 1 */
	tmp = 0x81;
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PMUC, tmp);
}

void transmission_config(void)
{
	/*** Continuous Transmit ***/
	//at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PC, bbcPC|0x80);
	/*** TX DAC overwrite ***/
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__TXDACI, 0x80|0x7e);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__TXDACQ, 0x80|0x3f);
#define BUFF_LEN 12
	/*** write 0 to buffer ***/
	uint8_t fb_data[BUFF_LEN] = {0};
	//PRINTF("[inphase] fb_data: 0x%x, 0x%x, 0x%x\n", fb_data[2], fb_data[5], fb_data[7]);
	at86rf215_txfb_write(pDev, 0, fb_data, BUFF_LEN);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__TXFLH, 0);
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__TXFLL, BUFF_LEN + 2);
	/* antenna diversity control is skipped, we only have one antenna */
}

void config_clean(void)
{
	/*** PMU ***/
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PMUC, 0);
	/*** Continuous Transmit ***/
	at86rf215_reg_write(pDev, pDev->bbc|AT86RF215_REG__PC, bbcPC);
	/*** TX DAC overwrite ***/
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__TXDACI, 0);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__TXDACQ, 0);
}

void setFrequency(uint16_t f, uint8_t offset)
{
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

void sender_pmu(void)
{
	at86rf215_reg_write(pDev,  pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TXPREP);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TX);
	/*** wait for receiver to measure ***/
	xtimer_usleep(120);
}

void receiver_pmu(uint8_t* pmu_value, uint8_t* pmuQF)
{
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_TXPREP);
	at86rf215_reg_write(pDev, pDev->rf|AT86RF215_REG__CMD, AT86RF215_STATE_RF_RX);
	/*** wait for sender to be ready ***/
	xtimer_usleep(70); // tx_delay + PHR = 297, extra = 50.

	*pmu_value = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__PMUVAL);
	*pmuQF = at86rf215_reg_read(pDev, pDev->bbc|AT86RF215_REG__PMUQF);
}

/********* Timer *********/

static volatile uint8_t timer_lock;

static void timer_callback(void *arg, int chan)
{
	(void)arg;
	(void)chan;
	timer_lock = 0;
}

int init_timer(void)
{
	timer_init(_TIMER, _TIMER_FREQUENCY, timer_callback, NULL);
	timer_stop(_TIMER);
	return 0;
}

void start_timer(unsigned int value)
{
	timer_lock = 1;
	timer_set_absolute(_TIMER, _TIMER_CHANNEL, value);
	TIM5->CNT = 0;
	timer_start(_TIMER);
}

void wait_for_timer(uint8_t id)
{
	(void)id;
	while(timer_lock) {}
	timer_stop(_TIMER);

	/* retart */
	timer_lock = 1;
	timer_set_absolute(_TIMER, _TIMER_CHANNEL, _TIMER_VALUE);
	TIM5->CNT = 0;
	timer_start(_TIMER);
}

void stop_timer(void)
{
	timer_stop(_TIMER);
}

/********* Serial Output *********/

static uint8_t bufByte;

void rs232_send(uint8_t data)
{
	bufByte = data;
	uart_write(UART_DEV(0), (const uint8_t *)&bufByte, (size_t)1);
}
