#include "error.h"

void throw_err(const char *msg)
{
    perror(msg);
    if (!errno)
        exit(EXIT_FAILURE);
    exit(errno);
}