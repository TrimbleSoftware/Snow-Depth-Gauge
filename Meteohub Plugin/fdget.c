/*

	fdget.h

*/

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stropts.h>
#include <termios.h>
#include <unistd.h>

#include "fdget.h"

// returns total number of bytes sucessfully written from *s to fd using timeout milliseconds
int fdputc_poll(byte c, int fd, int timeout)
{
#define NUMRETRYFDPUTC 6 // number of times to retry write

	struct pollfd fds[1];
	size_t i = 0;
	int rc = 0, pr, total = 0, ec = 0, ic = 0;
	size_t count = 1;

	fds[0].events = POLLWRNORM;
	fds[0].fd = fd;

	while(total < count && ic < NUMRETRYFDPUTC) // iterative read with poll on each
	{
		pr = poll(fds, 1, timeout);

		if(pr > 0) // sucess
		{
			if(fds[0].revents & POLLWRNORM)
			{
				i = write(fd, &c, 1); // write only one char
				total += i;
				if(i == 1) // char was written
					rc = total;
				else
				{
					rc = EOF; // error
					ic++; // inc iterations for retry logic
					continue;
				}
			}
			else
				rc = fds[0].revents;
		}
		else if(pr == 0) // timeout
		{
			rc = pr;
			ic++; // inc iterations for retry logic
			continue;
		}
		else if(pr < 0) // error
		{
			rc = -1;
			ic++; // inc iterations for retry logic
			ec = errno;
			continue;
		}
	}

	if(ec != 0)
		rc = ec; // return errno on error
	else
		fsync(fd);

	return rc;
}

// returns total number of bytes sucessfully written from *s to fd using timeout milliseconds
int fdputs_poll(const char *s, int fd, int timeout)
{
#define NUMTOWRITEFDPUTS 1 // number of bytes to output per call to write()
#define NUMRETRYFDPUTS 5 // number of times to retry output

	struct pollfd fds[1];
	size_t i = 0;
	int rc = 0, pr, total = 0, ec = 0, ic = 0;
	char buf[NUMTOWRITEFDPUTS + 1]; // add one for trailing NUL char
	size_t count = 0;

	fds[0].events = POLLWRNORM;
	fds[0].fd = fd;
	count = strlen(s);

	while(total < count && ic < NUMRETRYFDPUTS) // iterative read with poll on each
	{

		pr = poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if(pr > 0) // sucess
		{
			if(fds[0].revents & POLLWRNORM)
			{
				strncpy(buf, &s[total], NUMTOWRITEFDPUTS); // extract one chunk
				i = write(fd, buf, NUMTOWRITEFDPUTS); // write one chunk
				total += i;
				if(i == NUMTOWRITEFDPUTS) // string was written
					rc = total;
				else
				{
					rc = EOF; // error
					ic++; // inc iterations for retry logic
					continue;
				}
			}
			else
				rc = fds[0].revents;
		}
		else if(pr == 0) // timeout
		{
			rc = pr;
			ic++; // inc iterations for retry logic
			continue;
		}
		else if(pr < 0) // error
		{
			rc = -1;
			ic++; // inc iterations for retry logic
			ec = errno;
			continue;
		}
	}

	if(ec != 0)
		rc = ec; // return errno on error
	else
		fsync(fd);

	return rc;
}

// returns total number of bytes sucessfully read into *s from fd using timeout milliseconds
int fdgets_poll(char *s, size_t count, int fd, int timeout)
{

#define NUMTOREADFDGETS 1 // should always be kept at 1! number of bytes to input per call to read()
#define NUMRETRYFDGETS 5 // number of times to retry input to get count bytes

	struct pollfd fds[1];
	size_t i = 0;
	int rc = 0, pr, total = 0, ec = 0, ic = 0;
	byte buf[NUMTOREADFDGETS];

	fds[0].events = POLLRDNORM;
	fds[0].fd = fd;

	while(total < count && ic < NUMRETRYFDGETS) // iterative read with poll on each
	{
		pr = poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if(pr > 0) // sucess
		{
			if(fds[0].revents & POLLRDNORM)
			{
				i = read(fd, buf, NUMTOREADFDGETS); // read one chunk
				total += i;
				if(i == NUMTOREADFDGETS)
				{
					strncat(s, buf, NUMTOREADFDGETS);
					if(s[strlen(s) - 1] == '\n') // found newline [only can work properly if reading 1 char at a  time]
						break; // done reading, exit
				}
				else
				{
					rc = EOF;
					ic++; // inc iterations for retry logic
					continue;
				}

			}
			else
				rc = fds[0].revents;
		}
		else if(pr == 0) // timeout
		{
			rc = pr;
			ic++; // inc iterations for retry logic
			continue;
		}
		else if(pr < 0) // error
		{
			rc = -1;
			ic++; // inc iterations for retry logic
			ec = errno;
			continue;
		}

	}

	rc = strlen(s);

	if(rc <= 0)
	{
		s = NULL;
		if(ec != 0)
			rc = ec; // return errno on error
	}

	return rc;
}
