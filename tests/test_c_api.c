/* Checks that the C API header is valid C and the basic calls work without a robot. */
#include <stdio.h>
#include <string.h>

#include "robotino_api1/c_api.h"

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            return 1;                                                  \
        }                                                              \
    } while (0)

int main(void) {
    rapi1_options options;
    rapi1_client* client;
    rapi1_sensor_state state;
    rapi1_error error;

    rapi1_options_init(&options);
    CHECK(strcmp(options.host, "172.26.1.1") == 0);
    CHECK(options.port == 80);

    /* Nothing listens on port 1: the connection is refused quickly. */
    options.host = "127.0.0.1";
    options.port = 1;
    options.timeout_ms = 500;
    client = rapi1_client_create(&options);
    CHECK(client != NULL);
    error = rapi1_connect(client);
    CHECK(error == RAPI1_ERROR_CONNECTION_REFUSED || error == RAPI1_ERROR_TIMEOUT);
    CHECK(strlen(rapi1_last_error_message(client)) > 0);
    CHECK(rapi1_connection_state_get(client) == RAPI1_DISCONNECTED);
    CHECK(rapi1_set_motor_velocity(client, 3, 1.0) == RAPI1_ERROR_INVALID_ARGUMENT);
    CHECK(rapi1_set_motor_velocity(client, 2, 1.0) == RAPI1_OK);
    rapi1_get_sensors(client, &state);
    CHECK(state.sequence == 0);
    CHECK(strcmp(rapi1_error_string(RAPI1_ERROR_TIMEOUT), "timeout") == 0);
    rapi1_client_destroy(client);

    options.cycle_ms = 0;
    CHECK(rapi1_client_create(&options) == NULL);
    puts("C API tests passed");
    return 0;
}
