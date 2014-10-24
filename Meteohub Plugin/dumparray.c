#include <stdlib.h>
#include <stdio.h>
void print_array(int *values, int n, char *msg)
{
	int i = 0;
	printf("\nprint_array from %s", msg);
	for(i = 0; i < n; i++)
		printf("\ni = %d, value = %d", i, values[i]);
}
// read array of ints from a file
int read_array(int *values, int n, char *filename)
{
	FILE *fp;
	int retval = 0;
	print_array(values, 10, "read_array");
	fp = fopen(filename, "rb");
	if(!fp)
		retval = -1;
	else
	{
		printf("\nsizeof_value[0] = %d, n = %d", sizeof(values[0]), n);
		retval = fread(values, sizeof(values[0]), n, fp);
		fclose(fp);
	}
	return retval;
}
#define NUMVALS 10
int main(int argc, char *argv[])
{
	int values[NUMVALS];
	int i;
	for(i = 0; i < NUMVALS; i++)
		values[i] = 0;
	printf("\nread %d ints", read_array(values, NUMVALS, argv[1]));
	print_array(values, NUMVALS, "dump");
	printf("\n");
}