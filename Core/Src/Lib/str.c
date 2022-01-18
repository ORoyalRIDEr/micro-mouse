#include <Lib/str.h>

uint32_t strlen(char *str)
{
    uint32_t len=0;
    while (str[len] != 0)
        len++;
    
    return len;
}

uint8_t strncmp(char *str1, char *str2, uint32_t n)
{
    for (uint32_t i=0; i<n; i++) {
        if (str1[i] != str2[i])
            return 0;
    }

    return 1;
}

int strcmp(char *str1, char *str2)
{
    uint32_t str1_len = strlen(str1);
    uint32_t str2_len = strlen(str2);

    if (str1_len != str2_len)
        return 0;

    return strncmp(str1, str2, str1_len);
}