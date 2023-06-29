#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "ecu_connector.h"
#include "error.h"

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

    // socket non bloccante
    if (fcntl(client_fd, F_SETFL, fcntl(client_fd, F_GETFL, 0) | O_NONBLOCK) == -1)
        throw_err("connect_to_ECU | fcntl");

    // opzioni socket
    server_addr.sun_family = AF_UNIX;
    strcpy(server_addr.sun_path, SERVER_NAME);

    // client tenta connessione ad ECU
    int connected;
    do
    {
        connected = connect(client_fd, server_addr_ptr, server_len);
        if (connected == -1)
            sleep(1);
    } while (connected == -1);

    return client_fd;
}

int connect_and_send_info_to_ECU(ComponentType component)
{
    int client_fd = connect_to_ECU();

    char buf[BUF_SIZE] = {0};
    snprintf(buf, sizeof(buf), "%d %d", component, getpid());

    // client invia a ECU il proprio pid ed il tipo di componente
    if (send(client_fd, buf, strlen(buf) + 1, 0) == -1)
        throw_err("connect_to_ECU | send");

    return client_fd;
}