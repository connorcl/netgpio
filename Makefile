KERNEL := /home/connor/Projects/CMP408/linux
PWD := $(shell pwd)

obj-m += netgpio.o

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KERNEL) M=$(PWD) modules
clean:
	make -C $(KERNEL) M=$(PWD) clean