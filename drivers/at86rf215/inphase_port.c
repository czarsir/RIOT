/***
 *
 * InPhase
 *
 */

/*** Driver ***/
#include "at86rf215.h"

/*** Self ***/
#include "inphase.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

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

uint8_t inphase_connection_close(void)
{
	return 0;
}
