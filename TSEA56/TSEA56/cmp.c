#include <stdio.h>
#include <stdlib.h>

int cmpfunc (const void * a, const void * b)
{
	return ( *(int*)a - *(int*)b );
}

qsort(sensor, 25, sizeof(double), cmpfunc);