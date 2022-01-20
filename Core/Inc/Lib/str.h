#ifndef STR_H
#define STR_H

#include <stdint.h>

uint32_t strlen(char *str);
uint8_t strncmp(char *str1, char *str2, uint32_t n);
uint8_t strcmp(char *str1, char *str2);
int32_t atoi(const char *str);

#endif // STR_H