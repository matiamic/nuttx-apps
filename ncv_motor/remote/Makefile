include $(APPDIR)/Make.defs

# Remote control for PWM motor

MAINSRC = motor_remote.c

# PWM built-in application info

PROGNAME = motor_remote
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_MOTOR_REMOTE)

include $(APPDIR)/Application.mk
