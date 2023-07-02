#pragma once

#define DEFAULT_PROTOCOL 0
#define SERVER_NAME "/tmp/ecu_server"
#define BUF_SIZE 16

typedef enum ComponentType
{
    CMP_UNKNOWN = -1,
    CMP_INPUT,
    CMP_STEER_BY_WIRE,
    CMP_THROTTLE_CONTROL,
    CMP_BRAKE_BY_WIRE,
    CMP_FRONT_CAMERA,
    CMP_RADAR,
    CMP_PARK_ASSIST,
    CMP_ECU,
} ComponentType;