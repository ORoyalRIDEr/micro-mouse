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

uint8_t strcmp(char *str1, char *str2)
{
    uint32_t str1_len = strlen(str1);
    uint32_t str2_len = strlen(str2);

    if (str1_len != str2_len)
        return 0;

    return strncmp(str1, str2, str1_len);
}

int32_t atoi(const char *str)
{
    uint8_t sign = 0;
    uint8_t buf[10] = {0}; // longest pos. number is 4.294.967.295 / 2

    uint8_t p=0;
    if (str[0] == '-') {
        sign = 1;
        p++;
    }

    uint8_t i=0;
    while ((str[p] >= '0') && (str[p] <= '9')) {
        buf[i] = str[p] - '0';
        p++;
        i++;
    }

    int32_t ret=0;
    for (uint8_t j=0; j<i; j++) {
        uint32_t digit = buf[i-j-1];
        for (uint8_t k=0; k<j; k++)
            digit *= 10;
        ret += digit;
    }

    if (sign) ret = -ret;
    
    return ret;
}