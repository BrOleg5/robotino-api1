/* C interface of robotino_api1, for ctypes and other foreign function interfaces.
 *
 * All functions are thread-safe. Functions returning rapi1_error return RAPI1_OK (0) on success.
 * Callbacks run on the library's background threads.
 */
#ifndef ROBOTINO_API1_C_API_H
#define ROBOTINO_API1_C_API_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#if defined(ROBOTINO_API1_C_BUILDING)
#define RAPI1_API __declspec(dllexport)
#else
#define RAPI1_API __declspec(dllimport)
#endif
#else
#define RAPI1_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RAPI1_NUM_MOTORS 3
#define RAPI1_NUM_DISTANCE_SENSORS 9
#define RAPI1_NUM_DIGITAL_INPUTS 8
#define RAPI1_NUM_DIGITAL_OUTPUTS 8
#define RAPI1_NUM_ANALOG_INPUTS 8
#define RAPI1_NUM_RELAYS 2

typedef enum rapi1_error {
    RAPI1_OK = 0,
    RAPI1_ERROR_HOST_NOT_FOUND,
    RAPI1_ERROR_CONNECTION_REFUSED,
    RAPI1_ERROR_TIMEOUT,
    RAPI1_ERROR_CONNECTION_CLOSED,
    RAPI1_ERROR_CONNECTION_BUSY,
    RAPI1_ERROR_PROTOCOL,
    RAPI1_ERROR_SOCKET,
    RAPI1_ERROR_NOT_CONNECTED,
    RAPI1_ERROR_INVALID_ARGUMENT
} rapi1_error;

typedef enum rapi1_connection_state {
    RAPI1_DISCONNECTED = 0,
    RAPI1_CONNECTING,
    RAPI1_CONNECTED
} rapi1_connection_state;

typedef struct rapi1_client rapi1_client;

typedef struct rapi1_options {
    const char* host;        /* default "172.26.1.0" */
    uint16_t port;           /* default 80 */
    uint32_t cycle_ms;       /* default 20 */
    uint32_t timeout_ms;     /* default 1000 */
    int auto_reconnect;      /* default 1 */
    uint32_t command_timeout_ms; /* default 0 (off) */
    uint16_t image_port;     /* default 8080 */
    double base_radius;      /* m, default 0.125 */
    double wheel_radius;     /* m, default 0.040 */
    double gear_ratio;       /* default 16 */
} rapi1_options;

typedef struct rapi1_sensor_state {
    uint64_t sequence;
    double motor_velocity[RAPI1_NUM_MOTORS];    /* rad/s at the motor shaft */
    int32_t motor_position[RAPI1_NUM_MOTORS];   /* encoder ticks */
    double motor_current[RAPI1_NUM_MOTORS];     /* A */
    double distance_voltage[RAPI1_NUM_DISTANCE_SENSORS]; /* V */
    double analog_input[RAPI1_NUM_ANALOG_INPUTS];        /* V */
    int digital_input[RAPI1_NUM_DIGITAL_INPUTS];
    int bumper;
    double battery_voltage;      /* V */
    double battery_current;      /* A */
    double power_output_current; /* A */
    int32_t encoder_input_position;
    int32_t encoder_input_velocity;
    uint16_t serial_frequency;
    uint32_t board_time;
    uint32_t firmware_version;
    int passive_mode;
} rapi1_sensor_state;

typedef struct rapi1_pose {
    double x, y, theta; /* m, m, rad */
} rapi1_pose;

typedef struct rapi1_image {
    int type; /* 0 JPEG, 2 raw */
    uint32_t width, height;
    uint8_t channels, bits_per_channel;
    const uint8_t* data; /* valid only during the callback */
    size_t size;
} rapi1_image;

typedef void (*rapi1_update_callback)(const rapi1_sensor_state* state, void* user);
typedef void (*rapi1_connection_callback)(rapi1_connection_state state, void* user);
typedef void (*rapi1_error_callback)(rapi1_error error, const char* message, void* user);
typedef void (*rapi1_image_callback)(const rapi1_image* image, void* user);

RAPI1_API const char* rapi1_version(void);
RAPI1_API const char* rapi1_error_string(rapi1_error error);

/* Fills options with the defaults. */
RAPI1_API void rapi1_options_init(rapi1_options* options);
/* options may be NULL for the defaults. Returns NULL on invalid options. */
RAPI1_API rapi1_client* rapi1_client_create(const rapi1_options* options);
RAPI1_API void rapi1_client_destroy(rapi1_client* client);

RAPI1_API rapi1_error rapi1_connect(rapi1_client* client);
RAPI1_API void rapi1_disconnect(rapi1_client* client);
RAPI1_API rapi1_connection_state rapi1_connection_state_get(const rapi1_client* client);
/* The message of the last error returned by rapi1_connect() or rapi1_camera_streaming(). */
RAPI1_API const char* rapi1_last_error_message(const rapi1_client* client);

RAPI1_API void rapi1_set_motor_velocities(rapi1_client* client, const double rad_s[RAPI1_NUM_MOTORS]);
RAPI1_API rapi1_error rapi1_set_motor_velocity(rapi1_client* client, unsigned motor, double rad_s);
RAPI1_API void rapi1_set_velocity(rapi1_client* client, double vx, double vy, double omega);
RAPI1_API void rapi1_stop(rapi1_client* client);
RAPI1_API rapi1_error rapi1_set_pid(rapi1_client* client, unsigned motor, uint8_t kp, uint8_t ki, uint8_t kd);
RAPI1_API rapi1_error rapi1_set_brake(rapi1_client* client, unsigned motor, int engaged);
RAPI1_API void rapi1_reset_motor_positions(rapi1_client* client);
RAPI1_API rapi1_error rapi1_set_digital_output(rapi1_client* client, unsigned output, int value);
RAPI1_API rapi1_error rapi1_set_relay(rapi1_client* client, unsigned relay, int value);
RAPI1_API void rapi1_set_power_output(rapi1_client* client, double duty);
RAPI1_API void rapi1_reset_encoder_input(rapi1_client* client);
RAPI1_API void rapi1_request_shutdown(rapi1_client* client);

RAPI1_API void rapi1_get_sensors(const rapi1_client* client, rapi1_sensor_state* state);
/* Returns 1 if a new state arrived within timeout_ms, 0 otherwise. */
RAPI1_API int rapi1_wait_for_update(rapi1_client* client, uint32_t timeout_ms);
/* Copies the robot's info text (NUL-terminated, truncated to size). Returns the full length. */
RAPI1_API size_t rapi1_get_info(const rapi1_client* client, char* buffer, size_t size);
RAPI1_API void rapi1_get_pose(const rapi1_client* client, rapi1_pose* pose);
RAPI1_API void rapi1_set_pose(rapi1_client* client, const rapi1_pose* pose);

/* resolution: 0 QVGA, 1 VGA; compression: 0 high, 1 low, 2 none. */
RAPI1_API void rapi1_set_camera_settings(rapi1_client* client, int resolution, int compression, uint8_t brightness,
                                     uint8_t contrast, int auto_white_balance);
RAPI1_API rapi1_error rapi1_camera_streaming(rapi1_client* client, int enable);

RAPI1_API void rapi1_on_update(rapi1_client* client, rapi1_update_callback callback, void* user);
RAPI1_API void rapi1_on_connection_changed(rapi1_client* client, rapi1_connection_callback callback, void* user);
RAPI1_API void rapi1_on_error(rapi1_client* client, rapi1_error_callback callback, void* user);
RAPI1_API void rapi1_on_image(rapi1_client* client, rapi1_image_callback callback, void* user);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTINO_API1_C_API_H */
