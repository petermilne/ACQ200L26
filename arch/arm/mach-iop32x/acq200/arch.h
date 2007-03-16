/*
 * arch.h - includes for acq2xx architecture
 */


#ifndef __ARCH_H__
#define __ARCH_H__

#error DELETE ME PLEASE
extern void acq200_map_io(void);
extern void __init fixup_acq200(
	struct machine_desc *desc, struct tag *params,
	char **cmdline, struct meminfo *mi);
extern void __init acq200_init_irq(void);
extern void __init acq200_init_machine(void);


extern void acq100_map_io(void);
extern void __init acq100_init_irq(void);
extern void __init acq100_init_machine(void);




#endif

