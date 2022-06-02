#ifndef BUTTON_H
#define BUTTON_H

void button_init();

// Return 0 for short press, 1 for long press
int wait_for_button_press();

// Wait until button is stable for ms milliseconds
void stabilize_button(int ms);

#endif // BUTTON_H