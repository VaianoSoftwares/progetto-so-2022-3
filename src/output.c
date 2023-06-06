#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include "headers.h"
#include "output.h"
#include "error.h"
#include "timestamp.h"

const char **exec_modes = {"NORMALE", "ARTIFICIALE"};
const char **allowed_inputs = {"INIZIO", "PARCHEGGIO", "ARRESTO"};
const char **steer_options = {"DESTRA", "SINISTRA"};

void create_components();
void wait_children();
void output();
void central_ECU();
void steer_by_wire();
void throttle_control();
void break_by_wire();
void front_windshield_camera();
void forward_facing_radar();
void park_assist();
void sorround_view_cameras();
int connect_to_ECU();
bool is_HMI_input_valid(char *input);
void update_log(char *filename, char *content);
char *timestamp();

int main(int argc, char **argv)
{
    // prelievo e controllo correttezza argomenti da cmd
    if (argc < 2 || (strcmp(argv[1], exec_modes[0]) && strcmp(argv[1], exec_modes[1])))
        throw_err("MAIN | invalid cmd args");

    // modalità di esecuzione
    char *exec_mode = argv[1];

    pid_t pid;

    // creazione processo output
    if (!(pid = fork()))
        output();
    else if (pid == -1)
        throw_err("MAIN | fork");

    // creazione processo central_ECU
    if (!(pid = fork()))
        central_ECU();
    else if (pid == -1)
        throw_err("MAIN | fork");

    wait_children();

    return EXIT_SUCCESS;
}

// processo padre attende la terminazione dei figli
void wait_children()
{
    while (waitpid(0, NULL, 0) > 0)
        sleep(1);
}

void output()
{
    const int client_fd = connect_to_ECU();

    char response_buffer[16] = {0}, trimmed_response[16] = {0};

    while (true)
    {
        if (recv(client_fd, response_buffer, sizeof(response_buffer), 0) == -1)
            throw_err("output | recv");

        snprintf(trimmed_response, strlen(response_buffer) + 1, "%s", response_buffer);
        puts(trimmed_response);
    }

    close(client_fd);

    exit(EXIT_SUCCESS);
}

void central_ECU()
{
    // creazione directory log
    mkdir(LOG_DIR, 0777);

    // eliminazione file di log pregressi
    unlink(STEER_BY_WIRE_LOG);
    unlink(THROTTLE_CONTROL_LOG);
    unlink(BRAKE_BY_WIRE_LOG);
    unlink(ECU_LOG);

    // inizializzazione struttura dati veicolo
    veicolo_t veicolo = {.velocita = 0, .stato = FERMO};

    create_components();

    wait_children();

    exit(EXIT_SUCCESS);
}

void create_components()
{
    pid_t pid;

    if (!(pid = fork()))
        steer_by_wire();
    else if (pid == -1)
        throw_err("create_components | fork");

    if (!(pid = fork()))
        throttle_control();
    else if (pid == -1)
        throw_err("create_components | fork");

    if (!(pid = fork()))
        break_by_wire();
    else if (pid == -1)
        throw_err("create_components | fork");

    if (!(pid = fork()))
        front_windshield_camera();
    else if (pid == -1)
        throw_err("create_components | fork");

    if (!(pid = fork()))
        forward_facing_radar();
    else if (pid == -1)
        throw_err("create_components | fork");
}

void steer_by_wire()
{
    unlink(STEER_BY_WIRE_LOG);

    int client_fd = connect_to_ECU();

    char response_buffer[16] = {0};

    while (true)
    {
        if (recv(client_fd, response_buffer, sizeof(response_buffer), 0) == -1)
            throw_err("steer_by_wire | recv");

        if (!strcmp(response_buffer, steer_options[0]))
            update_log(STEER_BY_WIRE_LOG, "sto girando a destra");
        else if (!strcmp(response_buffer, steer_options[1]))
            update_log(STEER_BY_WIRE_LOG, "sto girando a sinistra");
        else
            update_log(STEER_BY_WIRE_LOG, "no action");

        sleep(4);
    }

    close(client_fd);

    exit(EXIT_SUCCESS);
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

bool is_HMI_input_valid(char *input)
{
    for (int i = 0; i < 3; i++)
    {
        if (!strcmp(input, allowed_inputs[i]))
            return false;
    }

    return true;
}

void update_log(char *filename, char *content)
{
    // apertura file
    int fd;
    if ((fd = open(filename, O_CREAT | O_WRONLY | O_APPEND, 0666)) == -1)
        throw_err("update_log | open");

    // linea di testo da scrivere su file
    char write_line[64] = {0};

    sprintf(write_line, "%s: %s", timestamp(), content);

    // num byte da scrivere su file
    const size_t line_len = strlen(write_line) * sizeof(char);

    // scrittura file
    if ((write(fd, write_line, line_len)) == -1)
        throw_err("update_train_log | write");

    close(fd);
}

char *timestamp()
{
    const time_t now = time(NULL);
    const struct tm *time_ptr = localtime(&now);
    return asctime(time_ptr);
}