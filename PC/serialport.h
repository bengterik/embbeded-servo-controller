#ifndef SERIALPORT_H
#define SERIALPORT_H

/*
 * Open serial port. Open in canonical mode if canonical is non-zero
 * Returns file descriptor
 */
int serial_init(char *modemdevice, int canonical);

/* 
 * restore the old port settings 
 */
void serial_cleanup(int fd);

#endif
