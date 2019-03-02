obj-m += ti_cpsw_with_cpts.o
ti_cpsw_with_cpts-y := cpsw.o cpts.o

obj-m += dmtimer_pps.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) V=1 EXTRA_CFLAGS="-I$(KERNEL_SRC)" modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
