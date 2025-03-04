#ifndef STATES
#define STATES

enum State {
    OFF,
    INITIALISE,
    READY,
    BOOSTING,
    COASTING,
    APOGEE,
    DESCENT,
    LANDING,
    RECOVERY
};

#endif