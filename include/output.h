#pragma once

#define LOG_DIR "log"
#define STEER_BY_WIRE_LOG "log/steer.log"
#define THROTTLE_CONTROL_LOG "log/throttle.log"
#define BRAKE_BY_WIRE_LOG "log/brake.log"
#define ECU_LOG "log/ECU.log"
#define THROTTLE_MSG_FORMAT "INCREMENTO %d"
#define BRAKE_MSG_FORMAT "FRENO %d"

typedef enum stato_veicolo_t
{
    FERMO,
    NORMALE,
    PARCHEGGIO,
} stato_veicolo_t;

typedef struct veicolo_t
{
    int velocita;
    stato_veicolo_t stato;
} veicolo_t;