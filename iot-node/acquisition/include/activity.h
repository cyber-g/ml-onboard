#ifndef ACTIVITY_H
#define ACTIVITY_H

#define NB_ACTIVITIES 6
#define NO_ACTIVITY (-1)

extern volatile int current_activity;

const char *activity_name(int activity);

#endif // ACTIVITY_H