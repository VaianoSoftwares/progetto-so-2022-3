#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>

#include "error.h"

void throw_err(const char *fmt, ...)
{
    va_list list;
    va_start(list, fmt);

    if (errno)
    {
        char err_msg[64] = {0};
        vsnprintf(err_msg, sizeof(err_msg), fmt, list);
        va_end(list);

        fprintf(stderr, RED);
        perror(err_msg);
        fprintf(stderr, RESET);

        exit(errno);
    }

    fprintf(stderr, RED);
    vfprintf(stderr, fmt, list);
    fprintf(stderr, "\n" RESET);

    va_end(list);

    exit(EXIT_FAILURE);
}