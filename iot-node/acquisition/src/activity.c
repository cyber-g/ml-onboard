#include "activity.h"

volatile int current_activity = NO_ACTIVITY;

const char *activity_name(int activity) {
  switch (activity) {
  case 0:
    return "Walking";
  case 1:
    return "Jogging";
  case 2:
    return "Standing";
  case 3:
    return "Sitting";
  case 4:
    return "Upstairs";
  case 5:
    return "Downstairs";
  default:
    return "<ERROR>";
  }
}
