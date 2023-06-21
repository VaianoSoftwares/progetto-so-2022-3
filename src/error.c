#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "error.h"

void throw_err(const char *msg)
{
    perror(msg);
    exit(errno ? errno : EXIT_FAILURE);
}