#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} rgb_color;

void matrix_init(int prio);
rgb_color *get_empty_matrix(TickType_t delay);
void display_matrix(rgb_color *matrix);
void dispose_matrix(rgb_color *matrix);
void set_pixel(rgb_color *matrix, int r, int c, const rgb_color *p);
void add_char(rgb_color *matrix, int r, int c, char s, const rgb_color *p);

extern const rgb_color RED, GREEN, BLUE, WHITE;

#endif // MATRIX_H