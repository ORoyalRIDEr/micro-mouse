#ifndef STR_H
#define STR_H

#include <stdint.h>

uint32_t strlen(char *str);
uint8_t strncmp(char *str1, char *str2, uint32_t n);
int strcmp(char *str1, char *str2);

#endif // STR_H