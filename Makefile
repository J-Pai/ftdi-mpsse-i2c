# Makefile for ftdi i2c driver

ALL: i2c

i2c: i2c.c
	gcc `pkg-config --cflags libftdi`  -o $@  $<  `pkg-config --libs libftdi`

