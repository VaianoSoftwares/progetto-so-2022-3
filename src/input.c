#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/socket.h>

#include "headers.h"
#include "error.h"
#include "ecu_connector.h"

int main()
{
    // connessione ad ECU server
    const int client_fd = connect_and_send_info_to_ECU(CMP_INPUT);

    char buf[BUF_SIZE] = {0};

    while (true)
    {
        // in attessa di un comando dell'utente
        if (!fgets(buf, sizeof(buf), stdin))
            continue;

        // invia comando ad ECU server
        if (send(client_fd, buf, strlen(buf), 0) == -1)
            throw_err("INPUT | send");
    }

    close(client_fd);

    return EXIT_SUCCESS;
}