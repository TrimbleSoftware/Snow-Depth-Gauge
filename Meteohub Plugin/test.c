//#define DEBUG
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <malloc.h>
#include <math.h>
void print_array(int *values, int n, char *msg)
{
	int i = 0;
	printf("\nprint_array from %s", msg);
	for(i = 0; i < n; i++)
		printf("\ni = %d, value = %d", i, values[i]);
}
// write array of int to a file 
int write_array(int *values,int n, char *filename)
{
	FILE *fp;
	int retval = 0;
	//print_array(values, 10, "write_array");
	fp = fopen(filename, "wb");
	if(!fp)
		retval = -1;
	else
	{
		retval = fwrite(values, sizeof(values[0]), n, fp);
		fflush(fp);
		fclose(fp);
	}
	return retval;
}

// read array of ints from a file
int read_array(int *values, int n, char *filename)
{
	FILE *fp;
	int retval = 0;
	//print_array(values, 10, "read_array");
	fp = fopen(filename, "rb");
	if(!fp)
		retval = -1;
	else
	{
		retval = fread(values, sizeof(values[0]), n, fp);
		fclose(fp);
	}
	return retval;
}

// calculate the moving average of an array of n integer values with the addition of new_value
// used to smooth snow depth readings
float moving_average(int *values, int n, int new_value)
{
	int i = 0;;
	int sum = 0;
	for(i = 1; i < n; i++) // move old values over 1 slot
	{
		values[i - 1] = values[i];
	}
	values[n - 1] = new_value; // place new value into last slot
	for(i = 0; i < n; i++) // sum of old values & new value
	{
		sum += values[i];
	}
	//print_array(values, 10, "moving_average");
	return(sum / n); // average of old values & new value
}

float average(const int *values, int n)
{
	int sum = 0;
	int i = 0;
	for(i = 0; i < n; i++)
		sum += values[i];
	//print_array(values, 10, "average");
	return(sum / n);
}
// calculate standard deviation of an array of n integar values
// used to filter out outlyer readings from snow depth sensor
float standard_deviation(const int *values, int n)
{
	int i = 0;
	float sum = 0;
	float average;

	for(i = 0; i < n; i++)
		sum += values[i];
	average = sum / n;

	sum = 0;
	
	for(i = 0; i < n; i++)
		sum += (values[i] - average) * (values[i] - average);
	//print_array(values, 10, "standard_deviation");
	//printf("\nstddev = %f", sqrt(sum / n));; 
	return sqrt(sum / n);
}
int main (void)
{
	char fname[] = "./test.dat";
	int values[] = {80,77,81,83,84,75,76,84,77,76};
	int new_values[] = {79,3353,77,78,87,79,87,78,80,77};

	write_array(values, 10, fname);
	int i, j, k;
	int  newval = 0;
	int sma = 0;
	int avg = 0;
	for(i = 0; i < 10; i++)
	{
		newval = new_values[i];
		//printf("\n\ni = %d, newval = %d",i, newval);
		avg = (int)(average(values, 10) + 0.5);
		if(abs(newval) >=  (3 * standard_deviation(values, 10)) + avg) // if more that 3 stddev away from average, use average
		{
			newval = avg;
			//printf("\nnewval out of range changed to average = %d", newval);
		}
		sma = (int)(moving_average(values, 10, newval) + 0.5);
		//printf("\ni = %d, New = %d, sma = %d", i, newval, sma);
		printf("\n%d\t%d", i, sma);
		write_array(values, 10, fname);
		read_array(values, 10, fname);
	}
}

