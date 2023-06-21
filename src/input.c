#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "headers.h"
#include "error.h"
#include "ecu_connector.h"

int main()
{
    // connessione ad ECU server
    const int client_fd = connect_to_ECU(INPUT);

    char input_buffer[BUF_SIZE] = {0};

    while (true)
    {
        // in attessa di un comando dell'utente
        if (!fgets(input_buffer, sizeof(input_buffer), stdin))
            continue;

        // invia comando ad ECU server
        if (send(client_fd, input_buffer, strlen(input_buffer), 0) == -1)
            throw_err("INPUT | send");
    }

    close(client_fd);

    return EXIT_SUCCESS;
}