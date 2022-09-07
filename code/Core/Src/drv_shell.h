#pragma once

#include "fifo.h"
#include "shell.h"

extern Shell lshell;
extern FIFO shell_fifo;

int myPrintf(const char *formatString, ...);
void shell_init();