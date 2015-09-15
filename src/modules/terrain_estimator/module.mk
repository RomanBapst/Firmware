MODULE_COMMAND	= terrain_estimator

SRCS		= terrain_estimator_main.cpp \

# Startup handler, the actual app stack size is
# in the task_spawn command
MODULE_STACKSIZE = 1000