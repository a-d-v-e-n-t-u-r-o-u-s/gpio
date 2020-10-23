SOURCE += gpio.c

SOURCE_DIR := source
INLCUDE_DIR := include

LIBRARY := gpio

include rules-$(COMPILER).mk
