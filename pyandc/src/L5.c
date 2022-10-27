#include <stdio.h>

void innocent_code(int* input){
    int i;
    for(i = 0; i < 3; i++){
        printf("input[%d]: %d \n", i, input[i]);
        input[i] = 0;
    }
}

int main(){
    int secrets[3] = {4, 2, 8};
    int sec = 3;
    innocent_code(&sec);

    printf("sec: %d \n", sec);

    int i;
    for(i=0; i < 3; i++){
        printf("secrets[%d]: %d \n", i, secrets[i]);
        if(i == 1) break;
    }
    printf("i: %d", i);
}