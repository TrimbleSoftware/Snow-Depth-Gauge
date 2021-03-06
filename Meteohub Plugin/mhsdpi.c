/*

mhsdpi.c

meteohub "plug-in" weather station to read and log data from a 
Trimble Ultrasonic Snow Depth Gauge - Ver 1.4

Written:	7-May-2014 by Fred Trimble ftt@smtcpa.com

Modified:	22-Sep-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.0 Correct sensor number incerment when no snow depth data is found
			22-Oct-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.1 Added both 3 standard deviation filtering and simple moving average smoothing to sensor readings
			10-Nov-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.2 Changed to read standard deviation range filtering value from .conf file
			1-Dec-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.3a Changed to read sma array values from sensor at startup instead from saved file. Added logic to get_depth_value
				and get_range_value to allow better reporting on specific sensor errors.
			1-Dec-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.4 Added retry logic to depth, range and auto calibrate functions
			4-Dec-2014 by Fred Trimble ftt@smtcpa.com
				Ver 1.5 Added retry count to .conf file
			23-Feb-2015 by Fred Trimble ftt@smtcpa.com
				Ver 1.6 Added reread of sma array when stddev is out of range
			06-Sept-2015 by Fred Trimble ftt@smtcpa.com
				Ver 1.7 Added LiPo charger status output for Snow Depth Gauge Version 2B-1.5
			10-Jan-2017 by Fred Trimble ftt@smtcpa.com
				Ver 1.7a Added code to set tty file to non buffered and added mandentory delay when sensor restart is configed, minor bug fixes for sensor readings file config and writing.
			24-Jan-2017 by Fred Trimble ftt@smtcpa.com
				Ver 2.0 changed i/o to fd non-buffered using poll() and looping to read/write tty data and added retry logic to some sensor reading functions.
			31-Jan-2017 by Fred Trimble ftt@smtcpa.com
				Ver 2.0a added main loop exit via return codes propigated through sensor readings. To allow meteohub to restart plugin when comm errors occure.
			18-Feb-2017 by Fred Trimble ftt@smtcpa.com
				Ver 2.0b bug fixes in avarage amd moving average functions and initial sensor readings functions.

*/

// includes
#include "mhsdpi.h"

// defines
//#define DEBUG
#define VERSION "2.0b"
#define WAKEUPDELAY 10
#define GETDEPTHREADINGDELAY 15
#define TTYWRITETIMEOUT 500
#define TTYREADTIMEOUT 500

/*
main program
*/
int main (int argc, char *argv[])
{
	char log_file_name[FILENAME_MAX] = "";
	char config_file_name[FILENAME_MAX] = "";
	char readings_file_name[FILENAME_MAX] = "";
	strcpy(config_file_name, argv[0]);
	strcat(config_file_name, ".conf");
	static const char *optString = "BCd:h?Ls:t:";
	int datum = 0;
	int snowdepth = -1;
	int chargerStatus = -1;
	int snowdepth_sma = 0; // filtered Simple Moving Average snow depth
	int batteryVolts = -1;
	int readings[MAXREADINGS];
	int new_average = 0;
	uint32_t seconds_since_midnight = 0;
	int rc = 0;

	struct config_t config;
	struct termios oldsettings;

	// set default values for command line/config options
	config.restart_remote_sensor = false;
	config.close_tty_file = false;
	strcpy(config.device,"");
	config.write_log = false;
	strcpy(log_file_name, argv[0]);
	strcat(log_file_name,".log");
	strcpy(config.log_file_name, log_file_name);
	strcpy(readings_file_name, argv[0]);
	strcat(readings_file_name, ".dat");
	strcpy(config.readings_file_name, readings_file_name);
	config.sleep_seconds = 3660; // 1 hr is default sleep time;
	config.set_auto_datum = false;
	config.manual_datum = 5000;  // 5000 mm is default mounting datum height of sensor
	config.set_manual_datum = false;
	config.stdev_filter = 6;
	config.retry_count = 10;

	int ttyfile;

	uint32_t mh_data_id = 0;
	int i = 0;
	for(i = 0; i < MAXREADINGS; i++) // initilize readings history array elements to zero
		readings[i] = 0;

	const char mh_data_fmt[] = "data%d %d\n";

	char *message_buffer;
	char *message_buffer2;

	int set_tty_error_code = 0;

	// get cofig options
	if(!get_configuration(&config, config_file_name))
		fprintf(stderr,"\nNo readable .conf file found, using values from command line arguments");


	// get command line options, will override values read from .conf file
	int opt = 0;
	while ((opt = getopt(argc, argv ,optString)) != -1)
	{
		switch(opt)
		{
			case 'B':
				config.restart_remote_sensor = true;
				break;
			case 'C':
				config.close_tty_file = true;
				break;
			case 'd':
				strcpy(config.device, optarg);
				break;
			case 'D':
				config.set_auto_datum = true;
				break;
			case 'h':
			case '?':
				display_usage(argv[0]);
				break;
			case 'L':
				config.write_log = true;
				break;
			case 's':
				config.manual_datum = (uint16_t)atoi(optarg);
				config.set_manual_datum = true;
				break;
			case 't':
				config.sleep_seconds = (uint16_t)atoi(optarg);
				break;

		}
	}

	message_buffer = (char *)malloc(sizeof(char) * 256);
	message_buffer2 = (char *)malloc(sizeof(char) * 512);
	if (message_buffer == NULL || message_buffer2 == NULL)
	{
		fprintf(stderr, "can't allocate dynamic memory for buffers\n");
		return -2;
	}

	sprintf(message_buffer, "mhsdpi Version %s - Meteohub Plug-In for snow depth gauge", VERSION);
	if(config.write_log)
		writelog(config.log_file_name, argv[0], message_buffer);
	else
		fprintf(stderr, "%s.\n", message_buffer);

	if(strlen(config.device) == 0) // can't run when no device is specified
	{
		display_usage(argv[0]);
		return -1;
	}

	ttyfile = open(config.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	tcgetattr(ttyfile, &oldsettings); // save old tty settings
	
	if (!isatty(ttyfile))
	{
		if(config.write_log)
		{
			sprintf(message_buffer, "%s is not a tty", config.device);
			writelog(config.log_file_name, argv[0], message_buffer);
		}
		return 1;
	}
	// set tty port
	if((set_tty_error_code = set_tty_port(ttyfile, config.device, argv[0], config.log_file_name, config.write_log)))
	{
		if(config.write_log)
		{
			sprintf(message_buffer, "Error setting serial port: %d", set_tty_error_code);
			writelog(config.log_file_name, argv[0], message_buffer);
		}
		return 2;
	}

	tcflush(ttyfile, TCIOFLUSH);

	// restart remote sensor if called for
	if(config.restart_remote_sensor)
	{
		if(restart_sensor(ttyfile))
			sprintf(message_buffer, "Issued remote restart of sensor command");
		else
			sprintf(message_buffer, "Remote restart of sensor failed");

		if(config.write_log)
			writelog(config.log_file_name, argv[0], message_buffer);

		sleep(2*60); // delay to allow sensor to reboot and be ready to accept input
	}
	else
		sleep(WAKEUPDELAY); // minimal delay to make sure sensor is ready

	// log sensor firmware version
	if(config.write_log)
		print_firmware_version(ttyfile, config.log_file_name, argv[0]);

	if(config.set_manual_datum && !config.set_auto_datum)
	{
		if(config.manual_datum == set_manual_calibration_value(ttyfile, config.manual_datum))
			sprintf(message_buffer, "Set sensor datum to: %d", config.manual_datum);
		else
			sprintf(message_buffer, "Set sensor datum failed");
			
		if(config.write_log)
			writelog(config.log_file_name, argv[0], message_buffer);
	}

	if(config.set_auto_datum && !config.set_manual_datum)
	{
		datum = set_calibration_value(ttyfile, config.retry_count);
		if(config.write_log)
		{
			sprintf(message_buffer, "Auto sensor datum set to: %d", datum);
			writelog(config.log_file_name, argv[0], message_buffer);
		}
	}

	if(!config.set_auto_datum && !config.set_manual_datum)
	{
		datum = get_calibration_value(ttyfile, config.retry_count);
		if(datum >= 0)
		{
			// log sensor datum value
			if(config.write_log)
			{
				sprintf(message_buffer,"Datum value: %d", datum);
				writelog(config.log_file_name, argv[0], message_buffer);
			}
		}
		else
			writelog(config.log_file_name, argv[0], "Error getting datum value from sensor");
	}

	if(get_initial_sensor (readings, datum, ttyfile, config.retry_count) == 0)
		writelog(config.log_file_name, argv[0], "Error getting initial sensor values");
	else
		write_array(readings, MAXREADINGS, config.readings_file_name);

	seconds_since_midnight = get_seconds_since_midnight();

	if(config.sleep_seconds - (seconds_since_midnight % config.sleep_seconds) > 0)
	{
		sprintf(message_buffer, "Initial sleep: %d", config.sleep_seconds - (seconds_since_midnight % config.sleep_seconds));
		writelog(config.log_file_name, argv[0], message_buffer);
		sleep(config.sleep_seconds - (seconds_since_midnight % config.sleep_seconds)); // start polling on an even boundry of the specified polling interval
	}

	do // main plug-in loop
	{
		mh_data_id = 0;

		snowdepth = get_depth_value(ttyfile, config.retry_count); // read sensor value for snow depth via xBee Explorer on USB
		batteryVolts = get_battery_voltage(ttyfile, config.retry_count); // read sensor value for battery volts via xBee Explorer on USB
		chargerStatus = get_charger_status(ttyfile, config.retry_count); // read LiPo charger status

		if(snowdepth >= 0)
		{
			new_average = (int)(average(readings, MAXREADINGS) + 0.5);
			
			if(snowdepth == datum)
				snowdepth = new_average;

			if(abs(snowdepth) >= ((config.stdev_filter * standard_deviation(readings, MAXREADINGS)) + abs(new_average))) // if the sample is more than config.stdev_filter standard deviations away from the average
			{
				writelog(config.log_file_name, argv[0],"Snow depth reading out of range. Reinitializing sensor");;
				if(get_initial_sensor (readings, datum, ttyfile, config.retry_count) == 0)
				{
					writelog(config.log_file_name, argv[0], "Error getting initial sensor values");
				}
				new_average = (int)(average(readings, MAXREADINGS) + 0.5);
				sprintf(message_buffer,"Snow depth: %d reading out of range per filtering rules. Using new average: %d", snowdepth, new_average);
				writelog(config.log_file_name, argv[0],message_buffer);
				snowdepth = new_average; // use the prior readings average
			}

			snowdepth_sma = (int)(moving_average(readings, MAXREADINGS, snowdepth) + 0.5); // smooth the sensor readings

			write_array(readings, MAXREADINGS, config.readings_file_name);

			fprintf(stdout, mh_data_fmt, mh_data_id++, snowdepth_sma * 100);
			rc = 0;
			//sprintf(message_buffer,"Snow depth reading: %d", snowdepth_sma);
			//writelog(config.log_file_name, argv[0], message_buffer);
		}
		else
		{
			sprintf(message_buffer,"Error reading raw snow depth: %d", snowdepth);
			writelog(config.log_file_name, argv[0], message_buffer);
			mh_data_id++;
			rc = -2;
		}

		if(batteryVolts >= 0)
		{
			fprintf(stdout, mh_data_fmt, mh_data_id++, batteryVolts); // battery voltage is already *100 comming from sensor
			rc = 0;
		}
		else
		{
			sprintf(message_buffer,"Error reading raw battery volts: %d", batteryVolts);
			writelog(config.log_file_name, argv[0], message_buffer);
			mh_data_id++;
			rc = -3;
		}

		if(chargerStatus >= 0)
		{
			fprintf(stdout, mh_data_fmt, mh_data_id++, chargerStatus * 100);
			rc = 0;
		}
		else
		{
			sprintf(message_buffer,"Error reading raw charger status: %d", chargerStatus);
			writelog(config.log_file_name, argv[0], message_buffer);
			mh_data_id++;
			rc = -4;
		}

		fflush(stdout);

		if(config.close_tty_file) // close tty file
		{
			tcsetattr(ttyfile, TCSANOW, &oldsettings); // put old tty port setting back
			close(ttyfile);
		}

		seconds_since_midnight = get_seconds_since_midnight();
		sleep(config.sleep_seconds - (seconds_since_midnight % config.sleep_seconds)); // sleep just the right amount to keep on boundry

		if(config.close_tty_file) // open tty back up
		{
			ttyfile = open(config.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
			tcgetattr(ttyfile, &oldsettings); // save old tty settings
			// set tty port
			if((set_tty_error_code = set_tty_port(ttyfile, config.device, argv[0], config.log_file_name, config.write_log)))
			{
				if(config.write_log)
				{
					sprintf(message_buffer,"Error setting serial port: %d", set_tty_error_code);
					writelog(config.log_file_name, argv[0], message_buffer);
				}
				rc = -1;
			}
			else
				rc = 0;

			tcflush(ttyfile, TCIOFLUSH);
		}
		//read_array(readings, MAXREADINGS, readings_file_name); 
	}
	while(rc >= 0);

	tcsetattr(ttyfile, TCSANOW, &oldsettings); // put old tty port setting back
	close(ttyfile);
	free(message_buffer);

	return rc;
}

/*
function bodies
*/

// read sensor vaules into intial array used for sma smoothing
int get_initial_sensor (int *values, int datum, int fd, uint16_t retry_count)
{
	int retval = 1;
	int i = 0;
	int depth = 0;
	for(i = 0; i < MAXREADINGS; i++) // initilize the sma values array with current sensor readings
	{
		depth = get_depth_value(fd, retry_count);
		if(depth >= 0 && depth != datum) // don't use error values
			values[i] = depth;
		else
		{
			depth = (int)(average(values, i + 1) + 0.5); // backfill sensor reading errors with average of prior readings
			if(depth >= 0)
				values[i] = depth;
			else // error
				retval = 0;
		}
	}
	return retval;
}


// write array of int to a file 
int write_array(const int *values, int n, char *filename)
{
	FILE *fp;
	int retval = 0;
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
	for(i = 1; i < n; i++) // move old values over 1 slot
		values[i - 1] = values[i];

	values[n - 1] = new_value; // place new value into last slot

	return(average(values, n)); // average of old values & new value
}

float average(const int *values, int n)
{
	int sum = 0;
	int i = 0;
	for(i = 0; i < n; i++)
		sum += values[i];

	return(sum / n);
}

// calculate standard deviation of an array of n integar values
// used to filter out outlyer readings from snow depth sensor
float standard_deviation(const int *values, int n)
{
	int i = 0;
	float sum = 0;
	float avg;

	avg = average(values, n);

	for(i = 0; i < n; i++)
		sum += (values[i] - avg) * (values[i] - avg);

	return sqrt(sum / n);
}

// read Snow Depth Sensor firmware version, 7 lines
void print_firmware_version(int fd, char *logfilename, char *myname)
{
#define BUFSIZE 100
	char buf[BUFSIZE];
	int i = 0;
	
	tcflush(fd, TCIOFLUSH);
	fdputc_poll(CMD_GET_ABOUT, fd, TTYWRITETIMEOUT);
	sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
	for(i = 0; i < 7; i++)
	{
		memset(buf, NUL, sizeof(buf));
		fdgets_poll(buf, sizeof(buf), fd, TTYREADTIMEOUT);
#ifdef DEBUG
		int j = 0;
		for (j = 0; j < strlen(buf); j++)
			fprintf(stderr, "%c - 0x%x\n", buf[j],  buf[j]);;
#endif
		if(strlen(buf) > 0)
		{
			buf[strlen(buf) - 1] = '\0'; // get rid of CR-LF at end of string
			writelog(logfilename, myname, buf);
		}
	}
}

// read Snow Depth Sensor calibration height value
int get_calibration_value(int fd, uint16_t retry_count)
{
	char message_buffer[7];
	int retvalue = -1;
	do
	{
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_GET_CALIBRATION, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
#ifdef DEBUG
		int i = 0;
		for (i = 0; i < strlen(message_buffer); i++)
		fprintf(stderr, "%c - 0x%x\n", message_buffer[i],  message_buffer[i]);;
#endif
		if(message_buffer[0] == CMD_GET_CALIBRATION && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);

	return retvalue;
}

// set auto Snow Depth Sensor calibration height value
int set_calibration_value(int fd, uint16_t retry_count)
{
	char message_buffer[7];
	int retvalue = -1;

	do
	{
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_SET_CALIBRATE, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
#ifdef DEBUG
		int i = 0;
		for (i = 0; i < strlen(message_buffer); i++)
			fprintf(stderr, "%c - 0x%x\n", message_buffer[i],  message_buffer[i]);;
#endif
		if(message_buffer[0] == CMD_SET_CALIBRATE && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		else if(message_buffer[0] == CMD_SET_CALIBRATE && message_buffer[1] == '-' && (message_buffer[2] >= '0' && message_buffer[2] <= '9'))
			retvalue = -(((message_buffer[2] - '0') * 1000) + ((message_buffer[3] - '0') * 100) + ((message_buffer[4] - '0') * 10) + (message_buffer[5] - '0'));
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);
	
	return retvalue;
}

// manually set Snow Depth Sensor calibration value datum (aka mounting height above terra firma)
int set_manual_calibration_value(int fd, int value)
{
	char message_buffer[7];
	char command_buffer[7];
	int retvalue = -1;

	tcflush(fd, TCIOFLUSH);
	memset(command_buffer, NUL, sizeof(command_buffer));
	sprintf(command_buffer, "%c%04d\n", CMD_SET_MANUAL_CALIBRATE, value);
	fdputs_poll(command_buffer, fd, TTYWRITETIMEOUT);
	fsync(fd);
	fdputc_poll(CMD_SET_MANUAL_CALIBRATE, fd, TTYWRITETIMEOUT);
	memset(message_buffer, NUL, sizeof(message_buffer));
	sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
	fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
#ifdef DEBUG
	int i = 0;
	for (i = 0; i < strlen(command_buffer); i++)
		fprintf(stderr, "%c - 0x%x\n", command_buffer[i],  command_buffer[i]);;
	for (i = 0; i < strlen(message_buffer); i++)
		fprintf(stderr, "%c - 0x%x\n", message_buffer[i],  message_buffer[i]);;
#endif
	if(message_buffer[0] == CMD_SET_MANUAL_CALIBRATE && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
		retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		
	return retvalue;
}

// read Snow Depth Sensor snow depth value
int get_depth_value(int fd, int retry_count)
{
	char message_buffer[7];
	int retvalue = -1;
	do
	{
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_GET_DEPTH, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(GETDEPTHREADINGDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
		if(message_buffer[0] == CMD_GET_DEPTH && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
		{
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		}
		else if(message_buffer[0] == CMD_GET_DEPTH && message_buffer[1] == '-' && (message_buffer[2] >= '0' && message_buffer[2] <= '9'))
			retvalue = -(((message_buffer[2] - '0') * 1000) + ((message_buffer[3] - '0') * 100) + ((message_buffer[4] - '0') * 10) + (message_buffer[5] - '0'));
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);

	return retvalue;
}

// read Snow Depth Sensor range value
int get_range_value(int fd, uint16_t retry_count)
{
	char message_buffer[7];
	int retvalue = -1;
	do
	{
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_GET_RANGE, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
		if(message_buffer[0] == CMD_GET_RANGE && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		else if(message_buffer[0] == CMD_GET_RANGE && message_buffer[1] == '-' && (message_buffer[2] >= '0' && message_buffer[2] <= '9'))
			retvalue = -(((message_buffer[2] - '0') * 1000) + ((message_buffer[3] - '0') * 100) + ((message_buffer[4] - '0') * 10) + (message_buffer[5] - '0'));
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);

	return retvalue;
}

// read remote battery voltage
int get_battery_voltage(int fd, int retry_count)
{
	char message_buffer[7];
	int retvalue = -1;
	do
	{
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_GET_VOLTAGE, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
		if(message_buffer[0] == CMD_GET_VOLTAGE && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);
	
	return retvalue;
}

// read LiPo batter charger status from remote sensor
int get_charger_status(int fd, int retry_count)
{
	char message_buffer[7];
	int retvalue = -1;
	do
	{	
		tcflush(fd, TCIOFLUSH);
		fdputc_poll(CMD_GET_CHARGER_STATUS, fd, TTYWRITETIMEOUT);
		memset(message_buffer, NUL, sizeof(message_buffer));
		sleep(WAKEUPDELAY); // inital delay to let XBee catch-up
		fdgets_poll(message_buffer, sizeof(message_buffer), fd, TTYREADTIMEOUT);
		if(message_buffer[0] == CMD_GET_CHARGER_STATUS && (message_buffer[1] >= '0' && message_buffer[1] <= '9'))
			retvalue = ((message_buffer[1] - '0') * 1000) + ((message_buffer[2] - '0') * 100) + ((message_buffer[3] - '0') * 10) + (message_buffer[4] - '0');
		else
			retvalue = -1;

		retry_count--;
	}
	while(retry_count >= 0 && retvalue < 0);

	return retvalue;
}

// send command to restart CPU on remote Teensey 3.1/3.2 microcontroller
boolean restart_sensor(int fd)
{
	boolean retvalue = true;
	tcflush(fd, TCIOFLUSH);
	fdputc_poll(CMD_RESTART, fd, TTYWRITETIMEOUT);

	return retvalue;
}

// set serial port to communicate with Snow Depth sensor via xBee in transparent mode at 34800 baud
int set_tty_port(int ttyfile, char *device, char* myname, char *log_file_name, boolean writetolog)
{
	struct termios config;
	char *message_buffer;

	message_buffer = (char *)malloc(sizeof(char) * 256);
	if (message_buffer == NULL)
	{
		fprintf(stderr, "can't allocate dynamic memory for buffers\n");
		return -1;
	}

	if (tcgetattr(ttyfile, &config) < 0)
	{
		if(writetolog)
		{
			sprintf(message_buffer,"could not get termios attributes for %s", device);
			writelog(log_file_name, myname, message_buffer);
		}
		return -2;
	}

	// set serial port to 38400 baud, 8 bits, no parity
	config.c_iflag = (IGNPAR | IGNBRK | ICRNL);
	config.c_oflag = 0;
	config.c_cflag = (CREAD | CLOCAL | CS8);
	config.c_lflag = 0;
	cfmakeraw(&config);
	cfsetispeed(&config,B38400);
	cfsetospeed(&config,B38400);
	config.c_cc[VMIN] = 0; // non-blocking read
	config.c_cc[VTIME] = 0;
	tcflush(ttyfile, TCIOFLUSH);

	if(tcsetattr(ttyfile, TCSANOW, &config) < 0)
	{
		if(writetolog)
		{
			sprintf(message_buffer, "could not set termios attributes for %s", device);
			writelog(log_file_name, myname, message_buffer);
		}
		return -3;
	}
	else
	{
		return 0;
	}
}

// get seconds since midnight local time
uint32_t get_seconds_since_midnight (void)
{
	time_t t;
	struct tm *localtm;

	t = time(NULL);
	localtm = localtime(&t);

	return localtm->tm_sec + localtm->tm_min * 60 + localtm->tm_hour * 3600;
}

// write formatted messages to a log file named in logfilename
void writelog (char *logfilename, char *process_name, char *message)
{
	char timestamp[25];
	time_t t;
	struct tm *localtm;
	FILE *stream;

	t = time(NULL);
	localtm = localtime(&t);

	strftime(timestamp, sizeof(timestamp), "%d.%m.%Y %T", localtm);

	stream = fopen(logfilename, "a");
	fprintf(stream, "%s (%s): %s.\n", process_name, timestamp, message);
	fprintf(stderr, "%s (%s): %s.\n", process_name, timestamp, message);
	fclose(stream);
}

// display command line usage parameters
void display_usage(char *myname)
{
	fprintf(stderr, "mhsdpi Version %s - Meteohub Plug-In for snow depth gauge.\n", VERSION);
	fprintf(stderr, "Usage: %s -d tty_device [-C] [-L] [-t sleep_time]\n", myname);
	fprintf(stderr, "  -d tty_device  /dev/tty[x] device name where USB XBee adapter is connected.\n");
	fprintf(stderr, "  -C             Close/reopen tty device between polls.\n");
	fprintf(stderr, "  -L             Write messages to log file.\n");
	fprintf(stderr, "  -t sleep_time  Number of seconds to sleep between polling the snow depth sensor.\n");
	exit(EXIT_FAILURE);
}
