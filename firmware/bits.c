#include <stdint.h>

// sets a given 'bit' of a bit string to 1 (high)
void set(volatile uint8_t *bits, uint8_t bit)
{
  *bits |= (1 << bit);
}

// resets a given 'bit' of a bit string to 0 (low)
void reset(volatile uint8_t *bits, uint8_t bit)
{
  *bits &= ~(1 << bit);
}

// toggles a given 'bit' of a bit string
void toggle(volatile uint8_t *bits, uint8_t bit)
{
  *bits ^= 1 << bit;
}

