/*
 * acq200-gash - test harness to test kernel services
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include <linux/interrupt.h>

#include "acq200_debug.h"

struct workqueue_struct *workqueue;

struct workqueue_struct *my_workqueue ;


static unsigned delta;
static unsigned G_count;

static unsigned get_t1_count(void)
{
	static unsigned old;
	u32 new = *IOP321_GTSR;

	delta = new - old;
	return old = new;
}

#define TMESG G_count = get_t1_count();\
        info( "%s\t%ld \t8%u \t%u", message, jiffies, G_count, delta );


static void my_work_func( void* data )
{
	char *message = (char*)data;
	TMESG;
}

DECLARE_WORK(my_ws1, my_work_func,  "workfunc #1" );
DECLARE_WORK(my_ws2, my_work_func,  "workfunc #2" );


void my_tasklet_work( unsigned long data )
{
	char* message = (char*)data;
	TMESG;
}

DECLARE_TASKLET( my_tasklet, my_tasklet_work, (unsigned long)"my tasklet" );
DECLARE_TASKLET( my_tasklet_hi, my_tasklet_work, (unsigned long)"high priority" );

static void pas_func(void* data);
DECLARE_WORK(play_it_again_sam, pas_func, "work" );

int repcount = 10;

static void pas_func(void* data)
{
	char *message = (char *)data;

	TMESG;

	queue_work(my_workqueue, &my_ws1);
	queue_work(my_workqueue, &my_ws2);
	tasklet_schedule(&my_tasklet);
	tasklet_hi_schedule(&my_tasklet_hi);
	if ( repcount-- ){
		queue_delayed_work(my_workqueue, &play_it_again_sam, 10);
	}

	TMESG;
}


static int hello_init(void)
{
        printk(KERN_ALERT "Hello, world now with delta down");

	my_workqueue = create_workqueue( "gash" );

	pas_func( "init" );

        return 0;
}

static void hello_exit(void)
{
        printk(KERN_ALERT "Goodbye, cruel world");
}

module_init(hello_init);
module_exit(hello_exit);
MODULE_LICENSE("GPL");
