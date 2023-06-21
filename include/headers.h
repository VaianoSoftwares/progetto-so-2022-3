#define DEFAULT_PROTOCOL 0
#define ECU_SERVER "/tmp/ecu_server"
#define BUF_SIZE 16

typedef enum component_type_t
{
    UNKNOWN = -1,
    INPUT,
    STEER_BY_WIRE,
    THROTTLE_CONTROL,
    BRAKE_BY_WIRE,
    FRONT_CAMERA,
    RADAR,
    PARK_ASSIST,
    ECU = 255,
} component_type_t;