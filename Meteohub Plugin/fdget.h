/*

	fdget.h

*/

// defines
#define NUL '\0'
#ifndef POLLRDNORM 
#define	POLLRDNORM	0x0040		/* non-OOB/URG data available */
#endif
#ifndef POLLWRNORM
#define	POLLWRNORM	POLLOUT
#endif

// typedefs
typedef char byte;

// returns total number of bytes sucessfully written from *s to fd using timeout milliseconds
int fdputc_poll(byte c, int fd, int timeout);
// returns total number of bytes sucessfully written from *s to fd using timeout milliseconds
int fdputs_poll(const char *s, int fd, int timeout);
// returns total number of bytes sucessfully read into *s from fd using timeout milliseconds
int fdgets_poll(char *s, size_t count, int fd, int timeout);
