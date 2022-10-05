
#include <stdio.h>

int main(){

    char seed_str[] = "1234uwu", post[4];
    int seed;
    int ret_val = sscanf(seed_str, "%d%3s", &seed, post);

    
    printf("ret_val: %d \n", ret_val);
    printf("seed: %d \n", seed);

    for(int i = 0; i < sizeof(post)/sizeof(*post); i++){
        printf("post %d: %c \n", i, post[i]);
    }

}