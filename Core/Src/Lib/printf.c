#include <Lib/printf.h>
#include <Drivers/ZS040.h>

#include <stdint.h>
#include <stdarg.h>
#include <math.h>


uint32_t absi(int32_t input) 
{
    if (input > 0)
        return input;
    else
        return -input;
}

int32_t powi(int32_t base, uint32_t exp)
{
    if (base==0) return 0;
    else if ((base==1) || (exp==0)) return 1;

    int32_t val=base;
    /* init i with 1, because value already contains x^1 */
    for (uint32_t i=1;i<exp;i++) {
        val *= base;
    }
    
    return val;
}

int32_t divide_ceil(int32_t numerator, int32_t denominator)
{
    int32_t result = numerator / denominator;
    if (result*denominator < numerator)
        result++;

    return result;
}


uint32_t num_digits(uint32_t input, uint32_t base)
{
    /* returns number of digits when input is converted to a
    string using base representation */
    uint32_t tmp = input;
    uint32_t n_chars = 0;

    do {
        tmp /= base;
        n_chars++;
    } while (tmp);

    return n_chars;
}

void numtostr(char *buf, uint32_t input, uint8_t base, int32_t min_width, char paddingc)
{
    /* 
    Transforms number input into string and stores the result in buf. 
    
    base: Base of number representation (typically 10->dec or 16->hex)
    min_width: minimum number of charackters that should be filled before string
        termination charackter occures 
    paddingc: charackter which is used to fill up the chars before number starts,
        when the number of charackters in the input is less tha min_width
    */


    /* determine number of chars needed to display number */
    int32_t n_chars = (int32_t) num_digits(input, base);

    /* determine start index of number string */
    uint32_t i_num_start;
    if (n_chars > min_width)
        i_num_start = 0;
    else
        i_num_start = min_width - n_chars;

    /* fill up chars until start of number string */
    for (uint32_t i=0;i<i_num_start;i++) {
        buf[i] = paddingc;
    }

    /* place cnum at lowest digit and put zero terminator behind it */
    uint32_t rest;
    uint32_t cnum = i_num_start+n_chars-1;
    buf[cnum+1] = '\0';

    uint32_t tmp = input;
    do {
        rest = tmp % base;
        if (rest < 10)
            buf[cnum]= (char) rest + '0';
        else
            buf[cnum]= (char) (rest-10) + 'A';
        tmp /= base;
        cnum--;
    } while (tmp);
}

void intodec(char *buf, int32_t input, uint32_t min_width, char paddingc){
    const uint32_t base = 10;
    const uint8_t is_neg = (input < 0);
    uint32_t n_chars;
    uint8_t limit_buffer = 0;

    if (is_neg) {
        n_chars = num_digits(absi(input), base);

        /* when the first char needs to be a minus, the 'numtostr' function
        must not use it to store data. This is ensured by the following code. */
        if ((n_chars+1) > min_width)
            limit_buffer = 1;
    }

    if (limit_buffer)
        numtostr(&buf[1], absi(input), base, min_width-1, paddingc);
    else
        numtostr(buf, absi(input), base, min_width, paddingc);

    /* add minus at correct position */
    if (is_neg) {
        /* that is what we reserved the first buffer element for */
        if (limit_buffer) {
            buf[0] = '-';
        }
        else {
            if (paddingc == '0') {
                buf[0] = '-';
            }
            else {
                uint32_t pos_first_char = min_width-n_chars;
                buf[pos_first_char-1] = '-';
            }
        } 
    }
}

void uintodec(char *buf, uint32_t input, uint32_t min_width, char paddingc)
{
    const uint32_t base = 10;
    numtostr(buf, input, base, min_width, paddingc);
}

void hexakonv(char *buf, uint32_t input, uint32_t min_width, char paddingc)
{
    const uint32_t base = 16;
    numtostr(buf, input, base, min_width, paddingc);
}

int32_t fieldwidth(char *str){
    int32_t p=0;
    int32_t width=0;

    if((*str<'0')||(*str>'9')) 
        return -1;
    
    while((*str>='0')&&(*str<='9')){
        str++;
        p++;
    }

    str -= 1;
    for(int32_t i=0;i<p ;i++){
        width += (*str + '0') * powi(10,i);
        str--;
    }

    return width;
}

__attribute__((format(printf, 1, 2)))
void cprintf(char *fmt, ...)
{
    va_list argl;
    va_start(argl,fmt);
    
    const uint32_t buff_len = num_digits(__UINT32_MAX__, 2)+1;
    char int_buf[buff_len];

    int32_t i = 0;
    while (fmt[i] != '\0') {
        char paddingc = ' ';
        int32_t pwidth = 0;

        if (fmt[i] == '%') {
            /* look for filling charackter */
            i++;
            if(fmt[i] == '0') {
                paddingc = '0';
                i++;
            }

            /* look for minimum field with */
            int32_t tmp = fieldwidth(&fmt[i]);
            if (tmp > 0) {
                pwidth = tmp;
                i += num_digits(pwidth,10);
            }
            
            /* apply replacement */
            switch(fmt[i]){
                case 'c':;
                    char buf[] = {va_arg(argl,int), 0};
                    ZS040_print(buf);
                    break;
                case 's':
                    ZS040_print(va_arg(argl,char*));
                    break;
                case 'x':
                    hexakonv(int_buf, va_arg(argl, unsigned int), pwidth, paddingc);
                    ZS040_print(int_buf);
                    break;
                case 'i':
                    intodec(int_buf, va_arg(argl,  int), pwidth, paddingc);
                    ZS040_print(int_buf);
                    break;
                case 'u':
                    uintodec(int_buf, va_arg(argl, unsigned int), pwidth, paddingc);
                    ZS040_print(int_buf);
                    break;
                case 'p':
                    hexakonv(int_buf, (unsigned int) va_arg(argl, void*), pwidth, ' ');
                    ZS040_print("0x");
                    ZS040_print(int_buf);
                    break;
                case '%':
                    ZS040_print("%");
                    break;
                default:
                    ZS040_print("error: invalid replacement format string: '");
                    ZS040_print(fmt);
                    ZS040_print("'\n");
                    return;
            }
        }
        else {
            char buf[] = {fmt[i], 0};
            ZS040_print(buf);
        }

        i++;
    }
    va_end(argl);
}
