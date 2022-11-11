#include <stdlib.h>
#include <stdio.h>
int main()
{
    FILE *in_file;
    /* open in_matrix.txt for read */
    in_file = fopen("maze1.txt", "r");
    if (in_file == NULL)
        return -1;
    /* read matrix dimensions from file */
    int r, c;
    fscanf(in_file, "%d %d", &r, &c);

    char firstline[5];
    fgets(firstline, r, in_file);
    
    printf("%d %d", r, c);
    puts(firstline);



}