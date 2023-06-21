#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "ecu_connector.h"

int connect_to_ECU(component_type_t component)
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
    strcpy(server_addr.sun_path, ECU_SERVER);

    // client tenta connessione ad ECU
    int server_fd;
    do
    {
        server_fd = connect(client_fd, server_addr_ptr, server_len);
        if (server_fd == -1)
            sleep(1);
    } while (server_fd == -1);

    char req_buf[16] = {0};

    snprintf(req_buf, "%d %d", component, getpid());

    // client invia a ECU il proprio pid ed il tipo di componente
    if (send(server_fd, req_buf, strlen(req_buf), 0) == -1)
        throw_err("connect_to_ECU | send");

    return client_fd;
}