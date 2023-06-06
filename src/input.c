#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "headers.h"
#include "error.h"
#include "input.h"

int main(int argc, char **argv)
{
    const int client_fd = connect_to_ECU();

    char input_buffer[IN_BUFF_SIZE] = {0};

    while (true)
    {
        fgets(input_buffer, sizeof(input_buffer), stdin);

        if (send(client_fd, input_buffer, strlen(input_buffer) + 1, 0) == -1)
            throw_err("HMI-INPUT | send");
    }

    close(client_fd);

    return EXIT_SUCCESS;
}

int connect_to_ECU()
{
    // indirizzo server
    struct sockaddr_un server_addr;
    struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;
    socklen_t server_len = sizeof(server_addr);

    // creazione socket
    int client_fd;
    if ((client_fd = socket(AF_UNIX, SOCK_STREAM, DEFAULT_PROTOCOL)) == -1)
        throw_err("connect_to_ECU | socket");

    // opzioni socket
    server_addr.sun_family = AF_UNIX;
    strcpy(server_addr.sun_path, ECU_SERVER);

    //  INPUT tenta connessione ad ECU
    printf("HMI-INPUT\t| Tentivo di connessione ad ECU (fd=%d).\n", client_fd);
    int connected;
    do
    {
        connected = connect(client_fd, server_addr_ptr, server_len);
        if (connected == -1)
        {
            printf("HMI-INPUT\t| Connessione ad ECU fallita (fd=%d).\n", client_fd);
            sleep(1);
        }
    } while (connected == -1);
    printf("HMI-INPUT\t| Connessione ad ECU stabilita (fd=%d).\n", client_fd);

    return client_fd;
}