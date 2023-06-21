#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

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

static const char **EXEC_MODES = {"NORMALE", "ARTIFICIALE"};
static const char **STEER_CMDS = {"DESTRA", "SINISTRA"};
static const char **RANDOM = {"/dev/random", "data/randomARTIFICIALE.binary"};
static const char **URANDOM = {"/dev/urandom", "data/urandomARTIFICIALE.binary"};

exec_modes_t exec_mode;

int vehicle_fd;
vehicle_t *vehicle;

void create_components();
void wait_children();
void kill_all_children();
void central_ECU();
void steer_by_wire();
void throttle_control();
void break_by_wire();
void front_windshield_camera();
void forward_facing_radar();
void park_assist();
void sorround_view_cameras();
bool read_and_send_hex(int sock_fd, int data_fd, FILE *log_fp);
bool read_has_failed(ssize_t return_value);
char *timestamp();
float timer_sec_passed(clock_t timer);
bool throttle_is_broken();
void broken_throttle_handler(int sig);
void stop_vehicle_by_brake(int sig);
void ECU_listener(int server_fd);
int create_ECU_server();
void ECU_serve_req(FILE *log_fp);
void ECU_stop_vehicle(FILE *log_fp);
void ECU_parking(int server_fd, FILE *log_fp);
void send_parking_cmd(int client_fd, FILE *log_fp);
bool reset_parking(unsigned long data);

int main(int argc, char **argv)
{
    // prelievo e controllo correttezza argomenti da cmd
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <exec_mode>\n", argv[0]);
        throw_err("MAIN | invalid cmd args");
    }
    else if (!(strcmp(argv[1], EXEC_MODES[NORMALE])))
        exec_mode = NORMALE;
    else if (!(strcmp(argv[1], EXEC_MODES[ARTIFICIALE])))
        exec_mode = ARTIFICIALE;
    else
        throw_err("MAIN | invalid cmd args");

    // assegnazione random seed generator
    srand(time(NULL));

    central_ECU();

    return EXIT_SUCCESS;
}

void central_ECU()
{
    // creazione folder log
    mkdir(LOG_DIR, 0777);

    // creazione shared memory
    vehicle_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, OPEN_FILE_MODE);
    if (vehicle_fd == -1)
        throw_err("CENTRAL_ECU | shm_open");
    ftruncate(vehicle_fd, sizeof(vehicle_t));

    // inizializzazione struttura dati veicolo
    vehicle = (vehicle_t *)mmap(NULL, sizeof(vehicle_t), PROT_READ | PROT_WRITE, MAP_SHARED, vehicle_fd, 0);
    if (vehicle == MAP_FAILED)
        throw_err("central_ECU | mmap");
    bzero(vehicle, sizeof(vehicle_t));

    // creazione processi componenti
    create_components();

    // creazione server
    const int server_fd = create_ECU_server();

    signal(SIGUSR1, broken_throttle_handler);
    signal(SIGUSR2, stop_vehicle_by_brake);

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
    kill_all_children();

    // rilascio risorse
    close(vehicle_fd);
    if (munmap(vehicle, sizeof(vehicle_t)) == -1)
        throw_err("central_ECU | munmap");
    if (shm_unlink(SHM_NAME) == -1)
        throw_err("central_ECU | shm_unlink");

    ECU_parking(server_fd, log_fp);

    // terminazione di tutti i componenti
    kill_all_children();

    unlink(SERVER_NAME);

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

int create_ECU_server()
{
    struct sockaddr_un server_addr;

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
    if ((bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr))) == -1)
        throw_err("create_ECU_server | bind");

    // server abilitato a concedere richieste
    if ((listen(sock_fd, N_CONN)) == -1)
        throw_err("create_ECU_server | listen");

    return sock_fd;
}

void ECU_listener(int server_fd)
{
    pid_t pid;
    char buf[BUF_SIZE] = {0};
    ssize_t n_bytes_read;
    component_t new_client;
    component_type_t client_component;
    int conn_counter = 0;

    while (conn_counter < N_CONN)
    {
        if ((new_client.sock_fd = accept(server_fd,
                                         (struct sockaddr *)&new_client.addr,
                                         sizeof(new_client.addr))) == -1)
            throw_err("ECU_listener | accept");

        if (!(pid = fork()))
        {
            // in attesa di ricezione messaggio
            do
            {
                n_bytes_read = recv(new_client.sock_fd, buf, sizeof(buf), 0);
                if (read_has_failed(n_bytes_read))
                    throw_err("ECU_listener | recv");
            } while (n_bytes_read <= 0);

            // pid e tipo componente del nuovo client
            sscanf(buf, "%d %d", &client_component, new_client.pid);

            // verifica validità messaggio ricevuto
            // componente deve essere valido
            if (client_component < 0 || client_component > 5)
                throw_err("ECU_listener | invalid component type");

            // nuovo client viene salvato nella shm
            memcpy(&vehicle->components[client_component], &new_client, sizeof(vehicle_t));
            conn_counter++;

            // rilascio risorse
            close(server_fd);
            close(vehicle_fd);
            if (munmap(vehicle, sizeof(vehicle_t)) == -1)
                throw_err("central_ECU | munmap");

            _exit(EXIT_SUCCESS);
        }
        else if (pid == -1)
            throw_err("ECU_listner | fork");
    }

    wait_children();
}

void ECU_serve_req(FILE *log_fp)
{
    unsigned new_speed = 0;
    bool veh_started = false;

    char str_buf[BUF_SIZE] = {0};
    unsigned long hex_buf = 0UL;
    ssize_t n_bytes_read;

    clock_t speed_timer = 0;

    while (true)
    {
        // in attesa di ricezione messaggio da INPUT
        n_bytes_read = recv(vehicle->components[INPUT].sock_fd, str_buf, sizeof(str_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("central_ECU | recv");
        else if (n_bytes_read > 0)
        {
            // comando è INIZIO
            if (!strncmp(str_buf, START_CMD, strlen(START_CMD)))
                veh_started = true;
            // comando è PARCHEGGIO
            else if (!strncmp(str_buf, PARKING_CMD, strlen(PARKING_CMD)))
                return;
            // comando è ARRESTO
            else if (!strncmp(str_buf, STOP_CMD, strlen(STOP_CMD)))
            {
                // ECU invia segnale di ARRESTO a BRAKE_BY_WIRE
                kill(vehicle->components[BRAKE_BY_WIRE].pid, SIGUSR2);

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%s:" STOP_CMD "\n", timestamp());
                printf("%s:" STOP_CMD "\n", timestamp());
            }
        }

        // ECU ignora dati provenienti dai sensori se in modalità IDLE
        if (!veh_started)
            continue;

        // in attesa di ricezione dati da FRONT_CAMERA
        n_bytes_read = recv(vehicle->components[FRONT_CAMERA].sock_fd, str_buf, sizeof(str_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("central_ECU | recv");
        else if (n_bytes_read > 0)
        {
            // comando è DESTRA o SINISTRA
            if (!strncmp(str_buf, STEER_CMDS[RIGHT], strlen(STEER_CMDS[RIGHT])) ||
                !strncmp(str_buf, STEER_CMDS[LEFT], strlen(STEER_CMDS[LEFT])))
            {
                // ECU invia comando a steer_by_wire
                if ((send(vehicle->components[STEER_BY_WIRE].sock_fd, str_buf, strlen(str_buf), 0)) == -1)
                    throw_err("central_ECU | send");

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%s:%s\n", timestamp(), str_buf);
                printf("%s:%s\n", timestamp(), str_buf);
            }
            // comando è PERICOLO
            else if (!strncmp(str_buf, DANGER_CMD, strlen(DANGER_CMD)))
            {
                // ECU invia segnale di ARRESTO a BRAKE_BY_WIRE
                kill(vehicle->components[BRAKE_BY_WIRE].pid, SIGUSR2);

                // aggiornamento log e stampa in OUTPUT
                fprintf(log_fp, "%s:%s\n", timestamp(), DANGER_CMD);
                printf("%s:%s\n", timestamp(), DANGER_CMD);

                veh_started = false;

                continue;
            }
            // comando è PARCHEGGIO
            else if (!strncmp(str_buf, PARKING_CMD, strlen(PARKING_CMD)))
                return;
            // comando è un intero che rappresenta la velocità desiderata
            else
                sscanf(str_buf, "%u", &new_speed);
        }

        // in attesa di ricezione dati da RADAR
        n_bytes_read = recv(vehicle->components[RADAR].sock_fd, &hex_buf, sizeof(hex_buf), 0);
        if (read_has_failed(n_bytes_read))
            throw_err("central_ECU | recv");

        // ECU invia comandi a THROTTLE_CONTROL e BRAKE_BY_WIRE con frequenza di 1 secondo
        if (timer_sec_passed(speed_timer) < 1)
            continue;

        if (vehicle->speed < new_speed)
        {
            // ECU invia comando INCREMENTO 5 a THROTTLE_CONTROL
            if ((send(vehicle->components[THROTTLE_CONTROL].sock_fd, THROTTLE_CMD, strlen(THROTTLE_CMD), 0)) == -1)
                throw_err("central_ECU | send");

            // aggiornamento log e stampa in OUTPUT
            fprintf(log_fp, "%s:" THROTTLE_CMD "\n", timestamp());
            printf("%s:" THROTTLE_CMD "\n", timestamp());

            speed_timer = clock();
        }
        else if (vehicle->speed > new_speed)
        {
            // ECU invia comando FRENO 5 a BRAKE_BY_WIRE
            if ((send(vehicle->components[BRAKE_BY_WIRE].sock_fd, BRAKE_CMD, strlen(BRAKE_CMD), 0)) == -1)
                throw_err("central_ECU | send");

            // aggiornamento log e stampa in OUTPUT
            fprintf(log_fp, "%s:" BRAKE_CMD "\n", timestamp());
            printf("%s:" BRAKE_CMD "\n", timestamp());

            speed_timer = clock();
        }
    }
}

void ECU_stop_vehicle(FILE *log_fp)
{
    // imposta velocità auto a 0
    while (vehicle->speed > 0)
    {
        // ECU invia comando FRENO 5 a BRAKE_BY_WIRE
        if ((send(vehicle->components[BRAKE_BY_WIRE].sock_fd, BRAKE_CMD, strlen(BRAKE_CMD), 0)) == -1)
            throw_err("ECU_stop_vehicle | send");

        // aggiornamento log e stampa in OUTPUT
        fprintf(log_fp, "%s:" BRAKE_CMD "\n", timestamp());
        printf("%s:" BRAKE_CMD "\n", timestamp());

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
    struct sockaddr_un client_addr;
    const int client_fd = accept(server_fd,
                                 (struct sockaddr *)&client_addr,
                                 sizeof(client_addr));
    if (client_fd == -1)
        throw_err("ECU_parking | accept");

    send_parking_cmd(client_fd, log_fp);

    unsigned long hex_buf = 0UL;
    ssize_t n_bytes_read;
    clock_t parking_timer = clock();

    while (timer_sec_passed(parking_timer) < PARKING_TIMEOUT)
    {
        // in attesa di ricezione dati da PARK_ASSIST
        n_bytes_read = recv(client_fd, hex_buf, sizeof(hex_buf), 0);
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
    fprintf(log_fp, "%s:" PARKING_CMD "\n", timestamp());
    printf("%s:" PARKING_CMD "\n", timestamp());
}

void steer_by_wire()
{
    static const char **steer_log_msgs = {"sto girando a destra",
                                          "sto girando a sinistra",
                                          "no action"};

    // apertura file di log
    FILE *log_fp = fopen(STEER_BY_WIRE_LOG, "w");
    if (!log_fp)
        throw_err("steer_by_wire | fopen");

    // connessione ad ECU server
    int client_fd = connect_to_ECU(STEER_BY_WIRE);

    char res_buf[BUF_SIZE] = {0};
    clock_t steer_timer = 0, log_timer = 0;
    steer_state_t steer_state = NO_ACTION;
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
            if (!strncmp(res_buf, STEER_CMDS[RIGHT], strlen(STEER_CMDS[RIGHT])))
            {
                // aggiornamento timer, sarà possibile riaccedere a questa routine
                // tra minimo 4 secondi
                steer_timer = clock();

                steer_state = RIGHT;
            }
            else if (!strncmp(res_buf, STEER_CMDS[LEFT], strlen(STEER_CMDS[LEFT])))
            {
                // aggiornamento timer, sarà possibile riaccedere a questa routine
                // tra minimo 4 secondi
                steer_timer = clock();

                steer_state = LEFT;
            }
            else
                steer_state = NO_ACTION;
        }

        // aggiornamento log ogni secondo
        if (timer_sec_passed(log_timer) < COMPONENT_UPD_SEC_DELAY)
            continue;

        // aggiornamento log
        fprintf(log_fp, "%s:%s\n", timestamp(), steer_log_msgs[steer_state]);

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
    int client_fd = connect_to_ECU(THROTTLE_CONTROL);

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
        vehicle->speed += SPEED_DELTA;
        fprintf(log_fp, "%s:" THROTTLE_CMD "\n", timestamp());
    }

    // rilascio risorse
    if ((munmap(vehicle, sizeof(vehicle_t))) == -1)
        throw_err("throttle_control | munmap");
    close(vehicle_fd);

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
    int client_fd = connect_to_ECU(BRAKE_BY_WIRE);

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

        vehicle->speed = (vehicle->speed < SPEED_DELTA) ? 0 : vehicle->speed - SPEED_DELTA;
        fprintf(log_fp, "%s:" BRAKE_CMD "\n", timestamp());
    }

    // rilascio risorse
    if ((munmap(vehicle, sizeof(vehicle_t))) == -1)
        throw_err("brake_by_wire | munmap");
    close(vehicle_fd);

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
    int client_fd = connect_to_ECU(FRONT_CAMERA);

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

        req_buf[n_bytes_read] = 0;

        // invio a server ECU
        if ((send(client_fd, req_buf, strlen(req_buf), 0)) == -1)
            throw_err("front_windshield_camera | send");

        // aggiorna log
        fprintf(log_fp, "%s:%s\n", timestamp(), req_buf);

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
    int client_fd = connect_to_ECU(RADAR);

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
    int client_fd = connect_to_ECU(PARK_ASSIST);

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
    unsigned long hex_buf;

    // lettura 8 byte
    const ssize_t n_bytes_read = read(data_fd, hex_buf, sizeof(hex_buf));
    if (read_has_failed(n_bytes_read))
        throw_err("read_and_send_hex | read");
    else if (n_bytes_read < N_BYTES)
        return false;

    // invia 8 byte a ECU server
    if (send(sock_fd, &hex_buf, sizeof(hex_buf), 0) == -1)
        throw_err("read_and_send_hex | send");

    // aggiornamento log
    fprintf(log_fp, "%s:%#lx\n", timestamp(), hex_buf);

    return true;
}

// processo padre attende la terminazione dei figli
void wait_children()
{
    while (waitpid(0, NULL, 0) > 0)
        ;
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

float timer_sec_passed(clock_t timer)
{
    return (clock() - timer) / CLOCKS_PER_SEC;
}

bool throttle_is_broken()
{
    return (rand() / RAND_MAX) < PROB_BROKEN_THROTTLE;
}

void broken_throttle_handler(int sig)
{
    // arresto veicolo
    vehicle->speed = 0;

    // aggiornamento log
    FILE *log_fp = fopen(ECU_LOG, "a");
    if (!log_fp)
        throw_err("broken_throttle_handler | fopen");

    fprintf(log_fp, "%s:terminazione esecuzione\n", timestamp());

    fclose(log_fp);

    printf("%s:terminazione esecuzione\n", timestamp());

    // accesso a shm rimosso
    if ((munmap(vehicle, sizeof(vehicle_t))) == -1)
        throw_err("broken_throttle_handler | munmap");
    close(vehicle_fd);
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
    vehicle->speed = 0;

    // aggiornamento log
    FILE *log_fp = fopen(BRAKE_BY_WIRE_LOG, "a");
    if (!log_fp)
        throw_err("stop_vehicle_by_brake | fopen");

    fprintf(log_fp, "%s:arresto auto\n", timestamp());

    fclose(log_fp);
}

bool reset_parking(unsigned long data)
{
    static const unsigned short *parking_values = {0x172A,
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