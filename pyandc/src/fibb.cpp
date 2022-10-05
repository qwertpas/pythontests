#include <iostream>


char *MemoryCopy(char *src, size_t len){
    char *dst = (char *)malloc(len);
    if(!src || !dst){
        return NULL;
    }
    for(size_t i = 0; i < len; i++){
        dst[i] = src[i];
    }
    return dst;
}


char src[] = "GeeksforGeeks"; 

int main() {

    char *dst = MemoryCopy(src, 3);


    printf("%c", *dst);
    printf("%c", *(dst+1));
    printf("%c", *dst+2);

}