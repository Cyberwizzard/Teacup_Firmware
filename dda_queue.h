#ifndef	_DDA_QUEUE
#define	_DDA_QUEUE

#include	"dda.h"
#include	"timer.h"

#define HEATER_WAIT_TIMEOUT 1000 MS

/*
	variables
*/

// this is the ringbuffer that holds the current and pending moves.
extern uint8_t	mb_head;
extern uint8_t	mb_tail;
extern DDA movebuffer[MOVEBUFFER_SIZE];

/*
	methods
*/

// queue status methods
uint8_t queue_full(void);
uint8_t queue_empty(void);
DDA *queue_current_movement(void);

#if defined(LOOKAHEAD) && defined(LOOKAHEAD_LEVEL) && LOOKAHEAD_LEVEL > 0 // queue traversal methods for lookahead
// get the number of queued moves
uint8_t queue_length(void);
// get the i^th move from the start of the buffer
//DDA* queue_get_move(uint8_t i);
// get the first move in the queue
DDA *queue_get_start(void);
// initialise the queue
void queue_init(void);
// print a dump of all moves in the queue
//void queue_dump(void);
#endif

// take one step
void queue_step(void);

// add a new target to the queue
// t == NULL means add a wait for target temp to the queue
void enqueue_home(TARGET *t, uint8_t endstop_check, uint8_t endstop_stop_cond);

static void enqueue(TARGET *) __attribute__ ((always_inline));
inline void enqueue(TARGET *t) {
  enqueue_home(t, 0, 0);
}

// called from step timer when current move is complete
void next_move(void);

// print queue status
void print_queue(void);

// flush the queue for eg; emergency stop
void queue_flush(void);

// wait for queue to empty
void queue_wait(void);

#endif	/* _DDA_QUEUE */
