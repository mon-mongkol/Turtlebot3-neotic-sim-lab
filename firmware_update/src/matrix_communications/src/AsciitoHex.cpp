#include <stdio.h>
#include <string.h>
 
void hex(unsigned char a, char* buf) {
    // hex lookup table
    char data[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    int i = 2;
    while (a) {
        buf[i ++] = data[a % 16];
        a /= 16;
    }    
    int j = 2;
    -- i;
    // reverse i..j
    while (j < i) {
        char t = buf[j];
        buf[j] = buf[i];
        buf[i] = t;
        i --;
        j ++;
    }
}
 
int main(int argc, char* argv[]) {
    if (argc == 0) {
        return 0;
    }
    for (int i = 1; i < argc; ++ i) {
        for (int j = 0; j < strlen(argv[i]); ++ j) {
            char buf[4];
            unsigned char cur = argv[i][j] & 0xFF;
            hex(cur, buf);
            printf("\"%s\", ", buf);
        }
    }
    printf("\n");
    return 0;
}