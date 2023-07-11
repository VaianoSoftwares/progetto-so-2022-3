#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>

#include "lib.h"
#include "error.h"
#include "ecu_connector.h"

int main()
{
    // connessione ad ECU server
    const int client_fd = connect_and_send_info_to_ECU(CMP_INPUT);

    char buf[BUF_SIZE] = {0}, *end_str = NULL;

    // in attessa di un comando dell'utente
    while (fgets(buf, sizeof(buf), stdin))
    {
        // rimozione line feed dal buf
        end_str = strchr(buf, '\n');
        if (end_str)
            *end_str = '\0';

        // invia comando ad ECU server
        if (send(client_fd, buf, strlen(buf) + 1, 0) == -1)
            throw_err("INPUT | send");
    }

    close(client_fd);

    return EXIT_FAILURE;
}