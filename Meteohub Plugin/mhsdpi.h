/*

	mhsdpi.h

*/
/*
	includes
*/
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

#include "fdget.h" // fd based lib that uses poll() for tty  I/O
/*
	defines
*/
#define true 1
#define false 0
///#define MAXREADINGS 10 // number of readings to use for moving average smoothing
#define MAXREADINGS 5 // number of readings to use for moving average smoothing

// commands
#define CMD_GET_ABOUT 'A'
#define CMD_RESTART 'B' // restart teensey CPU
#define CMD_SET_CALIBRATE 'C'
#define CMD_GET_DEPTH 'D'
#define CMD_GET_CALIBRATION 'G'
#define CMD_GET_RANGE 'R'
#define CMD_SET_MANUAL_CALIBRATE 'S'
#define CMD_GET_CHARGER_STATUS 'T'
#define CMD_GET_VOLTAGE 'V'

/*
	constants
*/

/*
	typedefs
*/
typedef unsigned char boolean;

/*
	structs
*/
struct args_t
{
	boolean restart_remote_sensor; 	// -B switch
	boolean close_tty_file;			// -C switch
	char *device;					// -d value
	boolean set_auto_datum;			// -D switch
	boolean write_log;				// -L switch
	uint16_t manual_datum;			// -s value
	boolean  set_manual_datum;
	uint16_t sleep_seconds;			// -t value.
};

struct config_t
{
	boolean restart_remote_sensor;
	boolean close_tty_file;
	char device[FILENAME_MAX];
	boolean set_auto_datum;
	boolean write_log;
	char log_file_name[FILENAME_MAX];
	char readings_file_name[FILENAME_MAX];
	uint16_t manual_datum;
	boolean  set_manual_datum;
	uint16_t sleep_seconds;
	uint16_t stdev_filter;
	uint16_t retry_count;
};

/*
	function prototypes
*/
int get_initial_sensor (int *values, int datum, int fd, uint16_t retry_count);
int write_array(const int *values, int n, char *filename);
int read_array(int *values,int n, char *filename);
float moving_average(int *values, int n, int new_value);
float average(const int *values, int n);
float standard_deviation(const int *values, int n);
void print_firmware_version(int fd, char *logfilename, char *myname);
int get_calibration_value(int fd, uint16_t retry_count);
int set_calibration_value(int fd, uint16_t retry_count);
int set_manual_calibration_value(int fd, int value);
int get_depth_value(int fd, int retry_count);
int get_range_value(int fd, uint16_t retry_count);
int get_battery_voltage(int fd, int retry_count);
int get_charger_status(int fd, int retry_count);
boolean restart_sensor(int fd);

int set_tty_port(int ttyfile, char *device, char* myname, char *log_file_name, boolean writetolog);
uint32_t get_seconds_since_midnight (void);
void writelog (char *logfilename, char *process_name, char *message);
void display_usage(char *myname);
int get_configuration(struct config_t *config, char *path);
