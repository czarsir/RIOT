/***
 *
 * InPhase
 *
 */

/*** Driver ***/
#include "periph/timer.h"
#include "at86rf215.h"

/*** Self ***/
#include "inphase.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define _TIMER            1
#define _TIMER_CHANNEL    0
#define _TIMER_FREQUENCY  (1000000ul)  // 1MHz <-> 1 us
#define _TIMER_VALUE      (1000)       // 1k <-> 1 ms

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

/********* Connection *********/

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

void start_timer(void)
{
	timer_lock = 1;
	timer_set_absolute(_TIMER, _TIMER_CHANNEL, _TIMER_VALUE);
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
