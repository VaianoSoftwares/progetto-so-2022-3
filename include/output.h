#define LOG_DIR "log"
#define STEER_BY_WIRE_LOG "log/steer.log"
#define THROTTLE_CONTROL_LOG "log/throttle.log"
#define BRAKE_BY_WIRE_LOG "log/brake.log"
#define FRONT_CAMERA_LOG "log/camera.log"
#define RADAR_LOG "log/radar.log"
#define PARK_ASSIST_LOG "log/assist.log"
#define SORROUND_VIEW_CAMERAS_LOG "log/cameras.log"
#define ECU_LOG "log/ECU.log"
#define SHM_NAME "veh_speed_shm"
#define FRONT_CAMERA_DATA "data/frontCamera.data"
#define THROTTLE_CMD "INCREMENTO 5"
#define BRAKE_CMD "FRENO 5"
#define PARKING_CMD "PARCHEGGIO"
#define START_CMD "INIZIO"
#define STOP_CMD "ARRESTO"
#define DANGER_CMD "PERICOLO"
#define N_CONN 6
#define N_BYTES 8
#define STEER_TIMEOUT 4
#define SPEED_DELTA 5
#define COMPONENT_UPD_SEC_DELAY 1
#define PARKING_TIMEOUT 30
#define OPEN_FILE_MODE 0666
#define PROB_BROKEN_THROTTLE 10000

typedef enum ExecModeType
{
    EM_NORMALE,
    EM_ARTIFICIALE
} ExecModeType;

typedef enum SteerActionType
{
    SS_RIGHT,
    SS_LEFT,
    SS_NO_ACTION
} SteerActionType;