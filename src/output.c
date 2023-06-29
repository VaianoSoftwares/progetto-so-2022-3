#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/wait.h>

#include "headers.h"
#include "output.h"
#include "error.h"
#include "ecu_connector.h"

static const char *EXEC_MODES[] = {"NORMALE", "ARTIFICIALE"};
static const char *STEER_CMDS[] = {"DESTRA", "SINISTRA"};
static const char *RANDOM[] = {"/dev/random", "data/randomARTIFICIALE.binary"};
static const char *URANDOM[] = {"/dev/urandom", "data/urandomARTIFICIALE.binary"};

static ExecModeType exec_mode;

static unsigned short *veh_speed;
static int veh_speed_fd;

static struct pollfd pfds[N_CONN] = {0};
static pid_t comp_pids[N_CONN] = {0};

void create_components();
void wait_children();
void kill_all_children();
void kill_all_components();
int central_ECU();
void steer_by_wire();
void throttle_control();
void brake_by_wire();
void front_windshield_camera();
void forward_facing_radar();
void park_assist();
void sorround_view_cameras();
bool read_and_send_hex(int sock_fd, int data_fd, FILE *log_fp);
bool read_has_failed(ssize_t return_value);
char *timestamp();
int timer_sec_passed(clock_t timer);
bool throttle_is_broken();
void broken_throttle_handler(int sig);
void stop_vehicle_by_brake(int sig);
void ECU_listener(int server_fd);
int create_ECU_server();
void ECU_serve_req(FILE *log_fp);
void ECU_enable_components();
void ECU_disable_components();
void ECU_stop_vehicle(FILE *log_fp);
void ECU_parking(int server_fd, FILE *log_fp);
void send_parking_cmd(int client_fd, FILE *log_fp);
bool reset_parking(unsigned long data);

int main(int argc, char *argv[])
{
    fclose(stdin);

    // prelievo e controllo correttezza argomenti da cmd
    if (argc != 2)
        throw_err("Usage: %s <exec_mode>", argv[0]);
    else if (!(strcmp(argv[1], EXEC_MODES[EM_NORMALE])))
        exec_mode = EM_NORMALE;
    else if (!(strcmp(argv[1], EXEC_MODES[EM_ARTIFICIALE])))
        exec_mode = EM_ARTIFICIALE;
    else
        throw_err("MAIN | invalid exec_mode");

    // assegnazione random seed generator
    srand(time(NULL));

    central_ECU();

    return EXIT_SUCCESS;
}

int central_ECU()
{
    // creazione folder log
    mkdir(LOG_DIR, 0777);

    // assegnazione funzioni handler per segnali
    signal(SIGUSR1, broken_throttle_handler);
    signal(SIGUSR2, stop_vehicle_by_brake);

    // creazione shared memory
    veh_speed_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, OPEN_FILE_MODE);
    if (veh_speed_fd == -1)
        throw_err("central_ECU | shm_open");
    ftruncate(veh_speed_fd, sizeof(*veh_speed));

    // inizializzazione velocità veicolo
    veh_speed = (unsigned short *)mmap(NULL, sizeof(*veh_speed), PROT_READ | PROT_WRITE, MAP_SHARED, veh_speed_fd, 0);
    if (veh_speed == MAP_FAILED)
        throw_err("central_ECU | mmap");
    *veh_speed = 0;

    // creazione processi componenti
    create_components();

    // creazione server
    const int server_fd = create_ECU_server();

    // in attesa che tutti i componenti siano connessi al server
    ECU_listener(server_fd);

    // apertura file di log
    FILE *log_fp = fopen(ECU_LOG, "w");
    if (!log_fp)
        throw_err("central_ECU | fopen");

    ECU_serve_req(log_fp);

    // imposta la velocità a 0
    ECU_stop_vehicle(log_fp);

    // terminazione di tutti i componenti
    kill_all_components();

    // rilascio risorse
    close(veh_speed_fd);
    if (munmap(veh_speed, sizeof(*veh_speed)) == -1)
        throw_err("central_ECU | munmap");
    if (shm_unlink(SHM_NAME) == -1)
        throw_err("central_ECU | shm_unlink");

    ECU_parking(server_fd, log_fp);

    // terminazione di tutti i componenti
    kill_all_children();

    unlink(SERVER_NAME);

    return EXIT_SUCCESS;
}

void create_components()
{
    pid_t pid;

    // processo STEER_BY_WIRE
    if (!(pid = fork()))
    {
        // rilascio risorse
        close(veh_speed_fd);
        if (munmap(veh_speed, sizeof(*veh_speed)) == -1)
            throw_err("central_ECU | munmap");

        steer_by_wire();
    }
    else if (pid == -1)
        throw_err("create_components | fork");

    // processo THROTTLE_CONTROL
    if (!(pid = fork()))
        throttle_control();
    else if (pid == -1)
        throw_err("create_components | fork");

    // processo BRAKE_BY_WIRE
    if (!(pid = fork()))
        brake_by_wire();
    else if (pid == -1)
        throw_err("create_components | fork");

    // processo FRONT_WINDSHIELD_CAMERA
    if (!(pid = fork()))
    {
        // rilascio risorse
        close(veh_speed_fd);
        if (munmap(veh_speed, sizeof(*veh_speed)) == -1)
            throw_err("central_ECU | munmap");

        front_windshield_camera();
    }
    else if (pid == -1)
        throw_err("create_components | fork");

    // processo RADAR
    if (!(pid = fork()))
    {
        // rilascio risorse
        close(veh_speed_fd);
        if (munmap(veh_speed, sizeof(*veh_speed)) == -1)
            throw_err("central_ECU | munmap");

        forward_facing_radar();
    }
    else if (pid == -1)
        throw_err("create_components | fork");
}

int create_ECU_server()
{
    struct sockaddr_un server_addr;
    struct sockaddr *server_addr_ptr = (struct sockaddr *)&server_addr;
    socklen_t server_addr_len = sizeof(server_addr);

    // creazione socket
    int sock_fd;
    if ((sock_fd = socket(AF_UNIX, SOCK_STREAM, DEFAULT_PROTOCOL)) == -1)
        throw_err("create_ECU_server | socket");

    // socket non bloccante
    if (fcntl(sock_fd, F_SETFL, fcntl(sock_fd, F_GETFL, 0) | O_NONBLOCK) == -1)
        throw_err("connect_to_ECU | fcntl");

    // opzioni socket
    server_addr.sun_family = AF_UNIX;
    strcpy(server_addr.sun_path, SERVER_NAME);

    unlink(SERVER_NAME);

    // associazione socket ad indirizzo locale server
    if ((bind(sock_fd, server_addr_ptr, server_addr_len)) == -1)
        throw_err("create_ECU_server | bind");

    // server abilitato a concedere richieste
    if ((listen(sock_fd, N_CONN + 1)) == -1)
        throw_err("create_ECU_server | listen");

    return sock_fd;
}

void ECU_listener(int server_fd)
{
    ComponentType client_component;

    int client_fd;
    pid_t client_pid;

    char buf[BUF_SIZE] = {0};
    ssize_t n_bytes_read;

    for (int i = 0; i < N_CONN; i++)
    {
        // componente si connette a ECU
        do
        {
            client_fd = accept(server_fd, NULL, NULL);
            if (read_has_failed(client_fd))
                throw_err("ECU_listener | accept");
        } while (client_fd < 0);

        // in attesa di ricezione messaggio
        do
        {
            n_bytes_read = recv(client_fd, buf, sizeof(buf), 0);
            if (read_has_failed(n_bytes_read))
                throw_err("ECU_listener | recv");
        } while (n_bytes_read <= 0);

        // ECU server salva sock_fd e pid di client
        if (sscanf(buf, "%d %d", &client_component, &client_pid) < 2)
            throw_err("ECU_listener | sscanf");

        comp_pids[client_component] = client_pid;
        // tutti i componenti inizialmente vengono ignorati ad eccezione di INPUT
        pfds[client_component].fd = client_component == CMP_INPUT ? client_fd : -client_fd;
        // sensori hanno solo evento di lettura, attuatori di scrittura
        switch (client_component)
        {
        case CMP_INPUT:
        case CMP_FRONT_CAMERA:
        case CMP_RADAR:
            pfds[client_component].events = POLLIN;
            break;
        case CMP_STEER_BY_WIRE:
        case CMP_THROTTLE_CONTROL:
        case CMP_BRAKE_BY_WIRE:
            pfds[client_component].events = POLLOUT;
            break;
        default:
            pfds[client_component].events = 0;
        }
    }
}

void ECU_serve_req(FILE *log_fp)
{
    unsigned short target_speed = *veh_speed;
    SteerStateType target_steer = SS_NO_ACTION;

    char str_buf[BUF_SIZE] = {0};
    unsigned long hex_buf = 0UL;
    ssize_t n_bytes_read;

    clock_t speed_timer = clock();

    while (true)
    {
        if (poll(pfds, N_CONN, -1) == -1)
            throw_err("ECU_serve_req | poll");

        for (int i = 0; i < N_CONN; i++)
        {
            if (!pfds[i].revents)
                continue;
            else if (pfds[i].revents & POLLERR)
                throw_err("ECU_serve_req | POLLERR");

            switch (i)
            {
            case CMP_INPUT:
                // in attesa di ricezione messaggio da INPUT
                n_bytes_read = recv(pfds[i].fd, str_buf, sizeof(str_buf), 0);
                if (read_has_failed(n_bytes_read))
                    throw_err("central_ECU | recv");
                else if (n_bytes_read <= 0)
                    break;

                // comando è INIZIO, i componenti vengono abilitati
                if (!strncmp(str_buf, START_CMD, strlen(START_CMD)))
                    ECU_enable_components();
                // comando è PARCHEGGIO
                else if (!strncmp(str_buf, PARKING_CMD, strlen(PARKING_CMD)))
                    return;
                // comando è ARRESTO
                else if (!strncmp(str_buf, STOP_CMD, strlen(STOP_CMD)))
                {
                    // ECU invia segnale di ARRESTO a BRAKE_BY_WIRE
                    kill(comp_pids[CMP_BRAKE_BY_WIRE], SIGUSR2);

                    // aggiornamento log e stampa in OUTPUT
                    fprintf(log_fp, "%.24s:" STOP_CMD "\n", timestamp());
                    fflush(log_fp);
                    printf("%.24s:" STOP_CMD "\n", timestamp());
                }

                printf("received %s (%ld nbytes) from input\n", str_buf, n_bytes_read);

                break;
            case CMP_STEER_BY_WIRE:
                if (target_steer == SS_NO_ACTION)
                    break;

                // ECU invia comando a steer_by_wire
                if ((send(pfds[i].fd, STEER_CMDS[target_steer], strlen(STEER_CMDS[target_steer]), 0)) == -1)
                    throw_err("central_ECU | send");

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%.24s:%s\n", timestamp(), STEER_CMDS[target_steer]);
                fflush(log_fp);
                printf("%.24s:%s\n", timestamp(), STEER_CMDS[target_steer]);

                target_steer = SS_NO_ACTION;

                break;
            case CMP_THROTTLE_CONTROL:
                if (timer_sec_passed(speed_timer) < 1 || (*veh_speed) >= target_speed)
                    break;

                // ECU invia comando INCREMENTO 5 a THROTTLE_CONTROL
                if ((send(pfds[i].fd, THROTTLE_CMD, strlen(THROTTLE_CMD), 0)) == -1)
                    throw_err("central_ECU | send");

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%.24s:" THROTTLE_CMD "\n", timestamp());
                fflush(log_fp);
                printf("%.24s:" THROTTLE_CMD "\n", timestamp());

                speed_timer = clock();

                break;
            case CMP_BRAKE_BY_WIRE:
                if (timer_sec_passed(speed_timer) < 1 || (*veh_speed) <= target_speed)
                    break;

                // ECU invia comando FRENO 5 a BRAKE_BY_WIRE
                if ((send(pfds[i].fd, BRAKE_CMD, strlen(BRAKE_CMD), 0)) == -1)
                    throw_err("central_ECU | send");

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%.24s:" BRAKE_CMD "\n", timestamp());
                fflush(log_fp);
                printf("%.24s:" BRAKE_CMD "\n", timestamp());

                speed_timer = clock();

                break;
            case CMP_FRONT_CAMERA:
                // in attesa di ricezione dati da FRONT_CAMERA
                n_bytes_read = recv(pfds[i].fd, str_buf, sizeof(str_buf), 0);
                if (read_has_failed(n_bytes_read))
                    throw_err("central_ECU | recv");
                else if (n_bytes_read <= 0)
                    break;

                // comando è DESTRA
                if (!strncmp(str_buf, STEER_CMDS[SS_RIGHT], strlen(STEER_CMDS[SS_RIGHT])))
                {
                    target_steer = SS_RIGHT;
                    printf("received %s (%ld nbytes) from camera\n", str_buf, n_bytes_read);
                }
                // comando è SINISTRA
                else if (!strncmp(str_buf, STEER_CMDS[SS_LEFT], strlen(STEER_CMDS[SS_LEFT])))
                    target_steer = SS_LEFT;
                // comando è PERICOLO
                else if (!strncmp(str_buf, DANGER_CMD, strlen(DANGER_CMD)))
                {
                    // ECU invia segnale di ARRESTO a BRAKE_BY_WIRE
                    kill(comp_pids[CMP_BRAKE_BY_WIRE], SIGUSR2);

                    // aggiornamento log e stampa in OUTPUT
                    fprintf(log_fp, "%.24s:" DANGER_CMD "\n", timestamp());
                    fflush(log_fp);
                    printf("%.24s:" DANGER_CMD "\n", timestamp());

                    ECU_disable_components();
                }
                // comando è PARCHEGGIO
                else if (!strncmp(str_buf, PARKING_CMD, strlen(PARKING_CMD)))
                    return;
                // comando è un intero che rappresenta la velocità desiderata
                else
                {
                    sscanf(str_buf, "%hu", &target_speed);
                    printf("current speed: %hu - target speed: %hu\n", *veh_speed, target_speed);
                }

                break;
            case CMP_RADAR:
                // in attesa di ricezione dati da RADAR
                n_bytes_read = recv(pfds[i].fd, &hex_buf, sizeof(hex_buf), 0);
                if (read_has_failed(n_bytes_read))
                    throw_err("central_ECU | recv");

                break;
            }
        }
    }
}

// ECU server abilita il polling per tutti i client
void ECU_enable_components()
{
    for (int i = 1; i < N_CONN; i++)
    {
        pfds[i].fd = pfds[i].fd < 0 ? -pfds[i].fd : pfds[i].fd;
    }
}

// ECU server ignora tutti i clienti tranne INPUT durante il polling
void ECU_disable_components()
{
    for (int i = 1; i < N_CONN; i++)
    {
        pfds[i].fd = pfds[i].fd > 0 ? -pfds[i].fd : pfds[i].fd;
    }
}

void ECU_stop_vehicle(FILE *log_fp)
{
    // imposta velocità auto a 0
    while (*veh_speed > 0)
    {
        // ECU invia comando FRENO 5 a BRAKE_BY_WIRE
        if ((send(pfds[CMP_BRAKE_BY_WIRE].fd, BRAKE_CMD, strlen(BRAKE_CMD), 0)) == -1)
            throw_err("ECU_stop_vehicle | send");

        // aggiornamento log e stampa in OUTPUT
        fprintf(log_fp, "%.24s:" BRAKE_CMD "\n", timestamp());
        fflush(log_fp);
        printf("%.24s:" BRAKE_CMD "\n", timestamp());

        // ECU invia messaggi con frequenza di 1 msg/sec
        sleep(COMPONENT_UPD_SEC_DELAY);
    }
}

void ECU_parking(int server_fd, FILE *log_fp)
{
    // creazione componente PARK_ASSIST
    const pid_t pid = fork();
    if (!pid)
    {
        close(server_fd);
        fclose(log_fp);

        park_assist();
    }
    else if (pid == -1)
        throw_err("ECU_parking | fork");

    // PARK_ASSIST si connette ad ECU
    int client_fd;
    do
    {
        client_fd = accept(server_fd, NULL, NULL);
        if (read_has_failed(client_fd))
            throw_err("ECU_parking | accept");
    } while (client_fd < 0);

    send_parking_cmd(client_fd, log_fp);

    unsigned long hex_buf = 0UL;
    ssize_t n_bytes_read;
    clock_t parking_timer = clock();

    while (timer_sec_passed(parking_timer) < PARKING_TIMEOUT)
    {
        // in attesa di ricezione dati da PARK_ASSIST
        n_bytes_read = recv(client_fd, &hex_buf, sizeof(hex_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("ECU_parking | recv");
        // se riceve dati e corrispondono a parking_values
        // allora ECU invia comando PARCHEGGIO a PARK_ASSIST
        else if (n_bytes_read < N_BYTES || !reset_parking(hex_buf))
            continue;

        send_parking_cmd(client_fd, log_fp);

        parking_timer = clock();
    }

    close(client_fd);
}

void send_parking_cmd(int client_fd, FILE *log_fp)
{
    // ECU invia comando PARCHEGGIO a client
    if ((send(client_fd, PARKING_CMD, strlen(PARKING_CMD), 0)) == -1)
        throw_err("ECU_parking | send");

    // aggiornamento log e stampa in OUTPUT
    fprintf(log_fp, "%.24s:" PARKING_CMD "\n", timestamp());
    fflush(log_fp);
    printf("%.24s:" PARKING_CMD "\n", timestamp());
}

void steer_by_wire()
{
    static const char *steer_log_msgs[] = {"sto girando a destra",
                                           "sto girando a sinistra",
                                           "no action"};

    // apertura file di log
    FILE *log_fp = fopen(STEER_BY_WIRE_LOG, "w");
    if (!log_fp)
        throw_err("steer_by_wire | fopen");

    // connessione ad ECU server
    int client_fd = connect_and_send_info_to_ECU(CMP_STEER_BY_WIRE);

    char res_buf[BUF_SIZE] = {0};
    clock_t steer_timer = 0, log_timer = 0;
    SteerStateType steer_state = SS_NO_ACTION;
    ssize_t n_bytes_read;

    while (true)
    {
        // riceve un comando da ECU server
        n_bytes_read = recv(client_fd, res_buf, sizeof(res_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("steer_by_wire | recv");
        else if (n_bytes_read <= 0)
            continue;

        // stato sterzata può essere aggiornato ogni 4 sec
        if (timer_sec_passed(steer_timer) >= STEER_TIMEOUT)
        {
            // assegnazione stato corrente di steer_by_wire
            if (!strncmp(res_buf, STEER_CMDS[SS_RIGHT], strlen(STEER_CMDS[SS_RIGHT])))
            {
                // aggiornamento timer, sarà possibile riaccedere a questa routine
                // tra minimo 4 secondi
                steer_timer = clock();

                steer_state = SS_RIGHT;
            }
            else if (!strncmp(res_buf, STEER_CMDS[SS_LEFT], strlen(STEER_CMDS[SS_LEFT])))
            {
                // aggiornamento timer, sarà possibile riaccedere a questa routine
                // tra minimo 4 secondi
                steer_timer = clock();

                steer_state = SS_LEFT;
            }
            else
                steer_state = SS_NO_ACTION;
        }

        // aggiornamento log ogni secondo
        if (timer_sec_passed(log_timer) < COMPONENT_UPD_SEC_DELAY)
            continue;

        // aggiornamento log
        fprintf(log_fp, "%.24s:%s\n", timestamp(), steer_log_msgs[steer_state]);
        fflush(log_fp);

        log_timer = clock();
    }

    fclose(log_fp);
    close(client_fd);

    _exit(EXIT_SUCCESS);
}

void throttle_control()
{
    // apertura file di log
    FILE *log_fp = fopen(THROTTLE_CONTROL_LOG, "w");
    if (!log_fp)
        throw_err("throttle_control | fopen");

    // connessione ad ECU server
    int client_fd = connect_and_send_info_to_ECU(CMP_THROTTLE_CONTROL);

    char res_buf[BUF_SIZE] = {0};
    ssize_t n_bytes_read;

    while (true)
    {
        // riceve comando da ECU server
        n_bytes_read = recv(client_fd, res_buf, sizeof(res_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("throttle_control | recv");
        else if (n_bytes_read <= 0)
            continue;

        // verifica se il comando è corretto, altrimenti passa all'iterazione successiva
        if (strncmp(res_buf, THROTTLE_CMD, strlen(THROTTLE_CMD)))
            continue;

        // se l'acceleratore è rotto allora invia un segnale di pericolo a ECU server
        if (throttle_is_broken())
            kill(getppid(), SIGUSR1);

        // aggiornamento velocità veicolo
        *veh_speed += SPEED_DELTA;
        fprintf(log_fp, "%.24s:" THROTTLE_CMD "\n", timestamp());
        fflush(log_fp);
    }

    // rilascio risorse
    if ((munmap(veh_speed, sizeof(*veh_speed))) == -1)
        throw_err("throttle_control | munmap");
    close(veh_speed_fd);

    fclose(log_fp);
    close(client_fd);

    _exit(EXIT_SUCCESS);
}

void brake_by_wire()
{
    // apertura file di log
    FILE *log_fp = fopen(BRAKE_BY_WIRE_LOG, "w");
    if (!log_fp)
        throw_err("brake_by_wire | fopen");

    // connessione ad ECU server
    int client_fd = connect_and_send_info_to_ECU(CMP_BRAKE_BY_WIRE);

    char res_buf[BUF_SIZE] = {0};
    ssize_t n_bytes_read;

    while (true)
    {
        // riceve comando da ECU server
        n_bytes_read = recv(client_fd, res_buf, sizeof(res_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("brake_by_wire | recv");
        else if (n_bytes_read <= 0)
            continue;

        // se comando è FRENO 5 allora decrementa di 5 velocità e aggiorna log
        if (strncmp(res_buf, BRAKE_CMD, strlen(BRAKE_CMD)))
            continue;

        *veh_speed = (*veh_speed < SPEED_DELTA) ? 0 : *veh_speed - SPEED_DELTA;
        fprintf(log_fp, "%.24s:" BRAKE_CMD "\n", timestamp());
        fflush(log_fp);
    }

    // rilascio risorse
    if ((munmap(veh_speed, sizeof(*veh_speed))) == -1)
        throw_err("brake_by_wire | munmap");
    close(veh_speed_fd);

    fclose(log_fp);
    close(client_fd);

    _exit(EXIT_SUCCESS);
}

void front_windshield_camera()
{
    // apertura file di log
    FILE *log_fp = fopen(FRONT_CAMERA_LOG, "w");
    if (!log_fp)
        throw_err("front_windshield_camera | fopen");

    // apertura file dati
    FILE *data_fp = fopen(FRONT_CAMERA_DATA, "r");
    if (!data_fp)
        throw_err("front_windshield_camera | fopen");

    // connessione al server ECU
    int client_fd = connect_and_send_info_to_ECU(CMP_FRONT_CAMERA);

    char *req_buf = NULL;
    size_t req_buf_len = BUF_SIZE;
    ssize_t n_bytes_read;

    while (true)
    {
        // lettura linea del file dati
        // se EOF allora termina esecuzione componente
        n_bytes_read = getline(&req_buf, &req_buf_len, data_fp);
        if (n_bytes_read == -1 && (errno == EINVAL || errno == ENOMEM))
            throw_err("front_windshield_camera | getline");
        // EOF
        else if (n_bytes_read == -1)
            break;

        req_buf[n_bytes_read - 1] = 0;

        // invio a server ECU
        if ((send(client_fd, req_buf, strlen(req_buf) + 1, 0)) == -1)
            throw_err("front_windshield_camera | send");

        // aggiorna log
        fprintf(log_fp, "%.24s:%s\n", timestamp(), req_buf);
        fflush(log_fp);

        // invio dati con frequenza di 1 ciclo/sec
        sleep(COMPONENT_UPD_SEC_DELAY);
    }

    fclose(log_fp);
    fclose(data_fp);
    close(client_fd);
    free(req_buf);

    _exit(EXIT_SUCCESS);
}

void forward_facing_radar()
{
    // apertura file di log
    FILE *log_fp = fopen(RADAR_LOG, "w");
    if (!log_fp)
        throw_err("forward_facing_radar | fopen");

    // apertura file dati
    int data_fd = open(URANDOM[exec_mode], O_RDONLY, OPEN_FILE_MODE);
    if (data_fd == -1)
        throw_err("forward_facing_radar | open");

    // connessione al server ECU
    int client_fd = connect_and_send_info_to_ECU(CMP_RADAR);

    while (true)
    {
        // invio dati e aggiorna log con frequenza di 1 ciclo/sec
        read_and_send_hex(client_fd, data_fd, log_fp);
        sleep(COMPONENT_UPD_SEC_DELAY);
    }

    fclose(log_fp);
    close(data_fd);
    close(client_fd);

    _exit(EXIT_SUCCESS);
}

void park_assist()
{
    // apertura file di log
    FILE *log_fp = fopen(PARK_ASSIST_LOG, "w");
    if (!log_fp)
        throw_err("park_assist | fopen");

    // apertura file dati
    int data_fd = open(URANDOM[exec_mode],
                       O_RDONLY | O_NONBLOCK,
                       OPEN_FILE_MODE);
    if (data_fd == -1)
        throw_err("park_assist | open");

    // connessione al server ECU
    int client_fd = connect_to_ECU();

    char res_buf[BUF_SIZE] = {0};
    ssize_t n_bytes_read;
    bool parking = false;
    clock_t parking_timer = 0, log_timer = 0;
    pid_t child_pid;

    while (!parking || timer_sec_passed(parking_timer) < PARKING_TIMEOUT)
    {
        // riceve un comando da ECU server
        n_bytes_read = recv(client_fd, res_buf, sizeof(res_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("park assist | recv");
        // se riceve PARCHEGGIO allora avvia la procedura di parcheggio
        else if (n_bytes_read > 0 && !strncmp(res_buf, PARKING_CMD, strlen(PARKING_CMD)))
        {
            // viene creato processo sorround_view_cameras
            // solo la prima volta viene ricevuto il comando PARCHEGGIO
            if (!parking)
            {
                if (!(child_pid = fork()))
                {
                    fclose(log_fp);
                    close(data_fd);

                    sorround_view_cameras(client_fd);
                }
                else if (child_pid == -1)
                    throw_err("park_assist | fork");

                parking = true;
            }

            parking_timer = clock();
        }

        // procedura di parcheggio termina dopo 30 secondi
        // ogni secondo leggi dati, invia a ECU server e aggiorna il log
        if (!parking || timer_sec_passed(log_timer) < COMPONENT_UPD_SEC_DELAY)
            continue;

        if (!read_and_send_hex(client_fd, data_fd, log_fp))
            continue;

        log_timer = clock();
    }

    fclose(log_fp);
    close(data_fd);
    close(client_fd);

    // park_assist invia SIGTERM a sorround_view_cameras
    // park_assist attende la terminazione del figlio
    kill(child_pid, SIGTERM);
    wait(NULL);

    _exit(EXIT_SUCCESS);
}

void sorround_view_cameras(int client_fd)
{
    // apertura file di log
    FILE *log_fp = fopen(SORROUND_VIEW_CAMERAS_LOG, "w");
    if (!log_fp)
        throw_err("sorround_view_cameras | fopen");

    // apertura file dati
    int data_fd = open(URANDOM[exec_mode], O_RDONLY, OPEN_FILE_MODE);
    if (data_fd == -1)
        throw_err("sorround_view_cameras | open");

    while (true)
    {
        // ogni secondo leggi dati, invia a ECU server e aggiorna il log
        read_and_send_hex(client_fd, data_fd, log_fp);
        sleep(COMPONENT_UPD_SEC_DELAY);
    }

    fclose(log_fp);
    close(data_fd);
    close(client_fd);

    _exit(EXIT_SUCCESS);
}

bool read_and_send_hex(int sock_fd, int data_fd, FILE *log_fp)
{
    unsigned long hex_buf = 0UL;

    // lettura 8 byte
    const ssize_t n_bytes_read = read(data_fd, &hex_buf, sizeof(hex_buf));
    if (read_has_failed(n_bytes_read))
        throw_err("read_and_send_hex | read");
    else if (n_bytes_read < N_BYTES)
        return false;

    // invia 8 byte a ECU server
    if (send(sock_fd, &hex_buf, sizeof(hex_buf), 0) == -1)
        throw_err("read_and_send_hex | send");

    // aggiornamento log
    fprintf(log_fp, "%.24s:%#lx\n", timestamp(), hex_buf);
    fflush(log_fp);

    return true;
}

// processo padre attende la terminazione dei figli
void wait_children()
{
    while (waitpid(0, NULL, 0) > 0)
        sleep(1);
}

// ammazza tutti i processi figlio del processo invocante
void kill_all_children()
{
    // processo padre ignora SIGQUIT
    signal(SIGQUIT, SIG_IGN);
    // padre invia SIGQUIT a tutti i figli
    // (tutti i processi facenti parte dello stesso gruppo del processo invocante)
    kill(0, SIGQUIT);
    // padre attende per la terminazione di tutti i figli
    wait_children();
    // processo padre non ignora più SIGQUIT
    signal(SIGQUIT, SIG_DFL);
}

void kill_all_components()
{
    for (int i = 0; i < N_CONN; i++)
    {
        close(pfds[i].fd);
        kill(comp_pids[i], SIGKILL);
    }
    wait_children();
}

bool read_has_failed(ssize_t return_value)
{
    return return_value == -1 && errno != EWOULDBLOCK && errno != EAGAIN;
}

char *timestamp()
{
    const time_t now = time(NULL);
    const struct tm *time_ptr = localtime(&now);
    return asctime(time_ptr);
}

int timer_sec_passed(clock_t timer)
{
    return (int)(((double)(clock() - timer)) / CLOCKS_PER_SEC);
}

bool throttle_is_broken()
{
    return !(rand() % PROB_BROKEN_THROTTLE);
}

void broken_throttle_handler(int sig)
{
    // arresto veicolo
    *veh_speed = 0;

    // aggiornamento log
    FILE *log_fp = fopen(ECU_LOG, "a");
    if (!log_fp)
        throw_err("broken_throttle_handler | fopen");

    fprintf(log_fp, "%.24s:terminazione esecuzione\n", timestamp());

    fclose(log_fp);

    printf("%.24s:terminazione esecuzione\n", timestamp());

    // accesso a shm rimosso
    if ((munmap(veh_speed, sizeof(*veh_speed))) == -1)
        throw_err("broken_throttle_handler | munmap");
    close(veh_speed_fd);
    // eliminazione shared memory
    shm_unlink(SHM_NAME);

    // eliminazione server
    unlink(SERVER_NAME);

    // ECU termina tutti processi figlio
    kill_all_children();

    exit(EXIT_FAILURE);
}

void stop_vehicle_by_brake(int sig)
{
    // arresto veicolo
    *veh_speed = 0;

    // aggiornamento log
    FILE *log_fp = fopen(BRAKE_BY_WIRE_LOG, "a");
    if (!log_fp)
        throw_err("stop_vehicle_by_brake | fopen");

    fprintf(log_fp, "%.24s:arresto auto\n", timestamp());

    fclose(log_fp);
}

bool reset_parking(unsigned long data)
{
    static const unsigned short parking_values[] = {0x172A,
                                                    0xD693,
                                                    0x0000,
                                                    0xBDD8,
                                                    0xFAEE,
                                                    0x4300};
    static const size_t arr_size = sizeof(parking_values) / sizeof(parking_values[0]);

    unsigned short curr_u16;

    for (int i = 0; i < 14; i++)
    {
        // a curr_u16 vengono assegnati i 16 bit meno significativi di data
        curr_u16 = (unsigned short)data;

        for (int j = 0; j < arr_size; j++)
        {
            // se una sequenza valida di 4 hex corrisponde a curr_u16
            // allora la procedura di parcheggio verrà riavviata
            if (curr_u16 == parking_values[j])
                return true;
        }

        // il numero a 64 bit viene shiftato a destra di 4 bit (= 1 carattere hex)
        data >>= 4;
    }

    // nessuna sequenza di 4 hex valida trovata
    return false;
}