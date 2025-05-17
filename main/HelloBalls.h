/* This file contains all states declaration for HelloBalls Robot */
#ifndef HELLOBALLS_H
#define HELLOBALLS_H

typedef enum {
    STARTING,
    MACHINE_IDLE,
    STANDBY,
    SEARCHING_BALL,
    SEARCHING_HUMAN,
    RETRIEVING_BALL,
    STOPPED,
    ERROR
} State_t;

#endif // HELLOBALLS_H