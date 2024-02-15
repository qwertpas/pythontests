
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


int32_t unwrap(int32_t curr, int32_t prev){
    int32_t max = 360;
    int revs = prev - (prev%max);

    printf("revs: %d\n", revs);
    printf("curr: %d\n", curr);
    printf("prev: %d\n", prev);
    printf("prevmod: %d\n", prev%max);

    if ((prev%max <= -max/2) && (curr >= max/2)) { // detect angle wraparound and increment a revolution
        printf("-rev\n");
        return curr + revs - 2*max;
    } else if ((curr <= -max/2) && (prev%max >= max/2)) {
        printf("+rev\n");
        return curr + revs + 2*max;
    } else{
        printf("0rev\n");
        return curr + revs;
    }
}

int32_t unwrap2(int32_t curr, int32_t prev){
    int32_t max = 360;
    int revs = prev - (prev%max);

    int diff = curr - (prev+max)%(2*max) - max; 

    printf("revs: %d\n", revs);
    printf("prev: %d\n", prev);
    printf("curr: %d\n", curr);
    printf("diff: %d\n", diff);

    if ((prev%max <= -max/2) && (curr >= max/2)) { // detect angle wraparound and increment a revolution
        printf("-rev\n");
        return prev + diff - 2*max;
    } else if ((curr <= -max/2) && (prev%max >= max/2)) {
        printf("+rev\n");
        return prev + diff + 2*max;
    } else{
        printf("0rev\n");
        return prev + diff;
    }
}

int32_t unwrap3(int32_t curr, int32_t prev){
    int32_t max = 0xFF800000;
    curr += max;
    prev += max;

    int revs = prev - (prev%(2*max));

    if (prev%(2*max) < max/2 && curr > 3*max/2) { // detect angle wraparound and increment a revolution
        revs -= 2*max;
    } else if (curr < max/2 && prev%(2*max) > 3*max/2) {
        revs += 2*max;
    }

    return revs + curr - max;
}



int32_t unwrap4(int32_t curr, int32_t prev_raw, int32_t prev_unwrapped){
    int32_t max = 360;
    prev_raw = prev_unwrapped;
    while(prev_raw > max) 
    prev_raw = prev_unwrapped - ((prev_unwrapped+max/2)/max) - max/2;
    int32_t diff = curr - prev_raw;
    printf("prev_raw: %d\n", prev_raw);
    printf("diff: %d\n", diff);
    if(diff > max/2){
        diff = (curr - max) + (-max - prev_raw);
    }else if(diff < -max/2){
        diff = (curr - -max) + (max - prev_raw);
    }
    prev_unwrapped += diff;
    return prev_unwrapped;
}


int main(int argc, char const *argv[])
{
    int val = 0;
    int prev = 0;

    //0 -> max
    printf("++++++++++++++++++++\n");
    for(int i = 0; i < 360; i+= 45){
        printf("input: %d\n", i);
        prev = val;
        val = unwrap4(i, prev, val);
        printf("return: %d\n\n", val);
    }

    //min -> max
    printf("++++++++++++++++++++\n");
    for(int i = -359; i < 360; i+= 45){
        printf("input: %d\n", i);
        prev = val;
        val = unwrap4(i, prev, val);
        printf("return: %d\n\n", val);
    }

    // //min -> max
    // printf("++++++++++++++++++++\n");
    // for(int i = -359; i < 360; i+= 45){
    //     printf("input: %d\n", i);
    //     val = unwrap3(i, val);
    //     printf("return: %d\n\n", val);
    // }

    // //max -> min
    // printf("----------------------\n");
    // for(int i = 359; i > -360; i-= 45){
    //     printf("input: %d\n", i);
    //     val = unwrap3(i, val);
    //     printf("return: %d\n\n", val);
    // }

    // //max -> min
    // printf("----------------------\n");
    // for(int i = 359; i > -360; i-= 45){
    //     printf("input: %d\n", i);
    //     val = unwrap3(i, val);
    //     printf("return: %d\n\n", val);
    // }

    // //max -> 0
    // printf("----------------------\n");
    // for(int i = 359; i > 0; i-= 45){
    //     printf("input: %d\n", i);
    //     val = unwrap3(i, val);
    //     printf("return: %d\n\n", val);
    // }
    

    return 0;
}
