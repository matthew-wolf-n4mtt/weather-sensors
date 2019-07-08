/* weather-sensors.c
 * Linux daemon program that uses the Linux kernels Industrial I/O
 * subsystem (IIO) to get data from temperature, humidity, and pressure
 * sensors. The sensor data is output via a pseudo-terminal in the Peet Bros.
 * ULTIMETER serail data format[1].
 *
 * Version: 3.0.0
 * Author:  Matthew J Wolf
 * Date:    28-MAY-2019
 *
 * This Daemon Uses IIO Kernel Modules
 * -----------------------------------
 * chipcap2 = Amphenol / Telaire ChipCap 2 humidity and temperature sensor, [2].
 * mpl3115  = Freescale MPL3115A2 pressure and temperature sensor, included in
 *            Linux kernel source tree.
 *
 * Program Run Time Options and Defaults
 * ----------------------------
 * -d Debug   Does not demonize and displays message.
 * -l         File system location of symlink to pseudo-terminal
 *              Default: /tmp/weather-sensors-serial
 *--help      Display the program usage details
 *
 * Required Libraries
 * ------------------
 * Core C library
 * libiio = Linux Industrial I/O
 * - Linux distribution package
 *   or
 *   https://github.com/analogdevicesinc/libiio
 *
 * References
 * ----------
 *
 * [1] Home Weather Stations : Weather Observation Equipment :
 *     ULTIMETER Weather Stations Reviews.
 *     [Online].
 *     Available: https://www.peetbros.com/shop/custom.aspx?recid=29.
 *     [Accessed: 08-Jul-2019].
 *
 * [2] matthew-wolf-n4mtt, “matthew-wolf-n4mtt/chipcap2-linux-iio-module,” GitHub.
 *     [Online].
 *     Available: https://github.com/matthew-wolf-n4mtt/chipcap2-linux-iio-module.
 *     [Accessed: 08-Jul-2019].
 *
 * This file is part of Weather-Sensors.
 * By Matthew J. Wolf <matthew.wolf@speciosus.net>
 *
 * Copyright 2019 Matthew J. Wolf
 *
 * Weather-Sensors is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * the Free Software Foundation,either version 2 of the License,
 * or (at your option) any later version.
 *
 * Weather-Sensors is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Powermate-mpd.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <errno.h>
#include <fcntl.h>
#include <iio.h>
#include <math.h>
#include <pty.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "weather-sensors.h"

int debug = 0;

int fd_pty_master = -1;
int fd_pty_slave = -1;

char pty_symlink_name[1024] = "/tmp/weather-sensors-serial";

pid_t pid, sid;
FILE *pidfile;

/*
 * Fuction : get_iio_channel_by_name
 * Desc    : Find the IIO channel.
 * Inputs  :
 *          *iio_device_name - ext string of IIO device id.
 *          *iio_ctx - IIO context structure.
 *          *iio_ch_name - Text string of IIO channel type.
 * Outputs :
 *          iio_channel - IIO channel structure for *iio_ch_name
 *                        on device *iio_device_name.
 */
struct iio_channel * get_iio_channel_by_name(char *iio_device_name,
                                             struct iio_context *iio_ctx,
                                             char *iio_ch_name ) {
   struct iio_channel *local_iio_ch = NULL;
   struct iio_channel *return_iio_ch = NULL;
   const struct iio_device *iio_dev;

   iio_dev = iio_context_find_device(iio_ctx,iio_device_name);
   if (iio_dev != NULL) {
      local_iio_ch = iio_device_find_channel(iio_dev,iio_ch_name,0);
      if (local_iio_ch != NULL) {
         return_iio_ch = local_iio_ch;
      }
   }

   return ( return_iio_ch );
}
/*
 * Fuction : get_iio_value
 * Desc    : Get a IIO channel value.
 * Inputs  :
 *          *iio_ch - A IIO channel structure.
 * Outputs :
 *          The IIO channel value.
 */
double iio_get_value(struct iio_channel *iio_ch) {
   int ra,rb,rc;
   char buf[1024];
   double return_value = -1000;

   ra = iio_channel_attr_read(iio_ch, "raw", buf, sizeof( buf ));
   if (ra > 0) {
      return_value = strtod(buf,NULL);
      rb = iio_channel_attr_read(iio_ch, "scale", buf, sizeof( buf ));
      if (rb > 0) {
         return_value *= strtod(buf,NULL);
         rc = iio_channel_attr_read(iio_ch, "offset", buf, sizeof( buf ));
         if (rc > 0) {
            return_value += strtod(buf,NULL);
         }
      }
   }

   return( return_value );
}

/*
 * Fuction : set_pty_termios
 * Desc    : Configure a pseudo-terminal's termios structure
 * Inputs  :
 *          *tty  - pseudo-terminal's file descriptor
 *          speed - RS-232 Baud Rate
 *          party - RS-232 Parity bit
 *          block - I/O Blocking Mode
 * Outputs :
 *          0 - No Errors
            Less than 0 - A Error Occurred
 */
int set_pty_termios (struct termios *tty, int speed, int parity, int block) {
   int rc = 0;

   // Set the output baud rate
   if (cfsetospeed (tty, speed) != 0) {
      fprintf(stderr,"set_interface_attribs: Error setting output baud rate - %s\n",
              strerror (errno));
      syslog(LOG_ERR,"set_interface_attribs: Error setting output baud rate - %s",
             strerror (errno));
      rc = -1;
   }

   // Set input baud rate
   if (rc == 0) {
      if ( cfsetispeed (tty, speed) != 0) {
         fprintf(stderr,"set_interface_attribs: Error setting input baud rate - %s\n",
                 strerror (errno));
         syslog(LOG_ERR,"set_interface_attribs: Error setting input baud rate - %s",
                strerror (errno));
         rc = -2;
      }
   }

   // Change terminal attributes
   if (rc == 0) {
      // Modify the termios structure
      tty->c_cflag = ( tty->c_cflag & ~CSIZE ) | CS8; // 8-bit chars
      // disable IGNBRK for mismatched speed tests; otherwise receive break
      // as \000 chars
      tty->c_iflag &= ~IGNBRK;                // ignore break signal
      tty->c_lflag = 0;                       // no signaling chars, no echo,
                                              // no canonical processing
      tty->c_oflag = 0;                       // no remapping, no delays
      tty->c_cc[VMIN] = block ? 1 : 0;
      tty->c_cc[VTIME] = 5;                   // 0.5 seconds read timeout

      tty->c_iflag &= ~( IXON | IXOFF | IXANY ); // shut off xon/xoff ctrl

      tty->c_cflag |= ( CLOCAL | CREAD );     // ignore modem controls,
                                              // enable reading
      tty->c_cflag &= ~( PARENB | PARODD );   // shut off parity
      tty->c_cflag |= parity;
      tty->c_cflag &= ~CSTOPB;
      tty->c_cflag &= ~CRTSCTS;

   }

   return ( rc );
}

/*
 * Fuction : signal_handler
 * Desc    : The signal handler fuction that is registered with system kernel
 *           via a sigaction structure.
 * Inputs  : int signal - The system signal sent to the running process.
 * Outputs : Process termination value.
 */
void signal_handler(int signal) {
   switch (signal) {
   case SIGTERM:
      syslog(LOG_NOTICE,"Received SIGTERM: Exiting");
      close(fd_pty_master);
      close(fd_pty_slave);
      unlink(pty_symlink_name);
      unlink(LOCKFILE);
      exit(EXIT_SUCCESS);
      break;
   case SIGINT:
      syslog(LOG_NOTICE,"Received SIGINT: Exiting");
      close(fd_pty_master);
      close(fd_pty_slave);
      unlink(pty_symlink_name);
      unlink(LOCKFILE);
      exit(EXIT_SUCCESS);
      break;
   case SIGKILL:
      syslog(LOG_NOTICE,"Received SIGKILL: Exiting");
      close(fd_pty_master);
      close(fd_pty_slave);
      unlink(pty_symlink_name);
      unlink(LOCKFILE);
      exit(EXIT_SUCCESS);
      break;
   }

}

/*
 * Fuction : daemonize
 * Desc    : A fuction the daemonizes the process.
 * Inputs  : None
 * Outputs : None - A child process.
 */
static void daemonize() {
   int lf_fd;
   char buf[16];
   pid_t pid, sid;

   struct flock lf_flock;
   struct rlimit rl;
   struct sigaction sa;

   // Change the file mode mask
   umask(0);

   // Get the max limit of file descriptors
   if( getrlimit(RLIMIT_NOFILE, &rl) < 0 ) {
      syslog(LOG_ERR,"Can get file limit: %s",strerror(errno));
      exit(EXIT_FAILURE);
   }

   // Add comment
   if ( ( pid = fork()) < 0 ) {
      syslog(LOG_ERR,"Unable to create child process: %s",
             strerror(errno));
      exit(EXIT_FAILURE);
   }

   if (pid > 0) {      // Parent Exit
      exit(EXIT_SUCCESS);
   }

   // Get new session ID for child process.
   // Become session lead to drop controlling TTY.
   if ( ( sid = setsid()) < 0 ) {
      syslog(LOG_ERR,"Child session ID error: %s",strerror(errno));
      exit(EXIT_FAILURE);
   }

   // Chnage working directory to root. This do that the daemon process
   // will not be able to make and file system changes or unmount any file
   // system.
   if (chdir("/") < 0 ) {
      syslog(LOG_ERR,"Can change working directory to /: %s",
             strerror(errno));
      exit(EXIT_FAILURE);
   }

   // Setup signal handler with system kernel.
   // Clear the signal mask so that no new TTYs will be opened.
   sa.sa_handler = signal_handler;
   sigemptyset(&sa.sa_mask);
   sa.sa_flags = 0;
   sigaction(SIGTERM,&sa,NULL);
   sigaction(SIGINT,&sa,NULL);
   sigaction(SIGKILL,&sa,NULL);

   // Create lock / pid file.
   lf_fd = open(LOCKFILE, O_RDWR | O_CREAT,
                S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
   if (lf_fd < 0) {
      syslog(LOG_ERR,"Can not open lock file %s: %s",LOCKFILE,strerror(errno));
      exit(EXIT_FAILURE);
   }

   // Lock the lock / pid file
   lf_flock.l_type = F_WRLCK;
   lf_flock.l_start = 0;
   lf_flock.l_whence = SEEK_SET;
   lf_flock.l_len = 0;

   if ( fcntl(lf_fd, F_SETLK, &lf_flock) < 0 ) {
      if (errno == EACCES || errno == EAGAIN) {
         syslog(LOG_ERR,"Lock file access issue %s: %s",LOCKFILE,strerror(errno));
         close(lf_fd);
         exit(EXIT_FAILURE);
      }
      syslog(LOG_ERR,"Can not lock %s: %s",LOCKFILE,strerror(errno));
      close(lf_fd);
      exit(EXIT_FAILURE);
   }

   // Write the process PID into the lock file.
   ftruncate(lf_fd,0);
   sprintf(buf,"%ld", (long)getpid());
   write(lf_fd,buf,strlen(buf) + 1);

   // Close out the standard file descriptors
   close(STDIN_FILENO);
   close(STDOUT_FILENO);
   close(STDERR_FILENO);

   syslog(LOG_NOTICE,"Start Up");

   return;
}

int main(int argc, char *argv[]) {
   int i = -1;
   int ret = -1;
   int day_min = -1;

   char buf[1024];
   char pty_slave_name[1024];

   double cc2_humidity;
   double cc2_temp;
   double mpl_pres;

   time_t rawtime;
   struct tm * sys_tm;

   struct termios term_pty;

   struct iio_context *iio_ctx = NULL;
   struct iio_channel *iio_ch_cc2_humidity = NULL;
   struct iio_channel *iio_ch_cc2_temp = NULL;
   struct iio_channel *iio_ch_mpl = NULL;

   for ( i = 1; i < argc; i++ ) {
      // Enable debug mode
      if (!strcmp("-d",argv[i])) {
         debug = 1;
      }
      // The pseudoterminal symlink
      if (!strcmp("-l",argv[i])) {
         strcpy(pty_symlink_name,argv[i + 1]);
      }
      if (!strcmp("--help",argv[i])) {
         // Display Usage
         printf("Usage : weather-sensors -d -l <name> --help\n"
                "----------------------------------------------\n"
                "-d      Debug\n"
                "          Does not daemonize and displays messages\n"
                "-l      Filesystem location of symlink to pseudoterminal\n"
                "          Default: /tmp/weather-sensors-serial\n"
                "--help  Display the program usage details\n\n");
         exit(EXIT_SUCCESS);
      }
   }

   openlog("weather-sensors",LOG_PID, LOG_DAEMON);

   // Setup pseudoterminal termios
   // Set speed to 9600 bps, 8n1 (no parity), non-blocking
   ret = set_pty_termios(&term_pty, B9600,0,0);
   if (ret != 0) {
      fprintf(stderr,"Error setting pty termios attributes: %d\n", ret);
      syslog(LOG_ERR,"Error setting pty termios attributes: %d", ret);
      exit(EXIT_FAILURE);
   }

   // Open pseudoterminal
   ret = openpty(&fd_pty_master,&fd_pty_slave,pty_slave_name,&term_pty,NULL);
   if (ret == -1) {
      printf("Fail to open pty: %s\n", strerror (errno) );
      syslog(LOG_ERR,"Fail to open pty: %s", strerror (errno) );
      exit(EXIT_FAILURE);
   }

   if (debug) {
      printf("PTY: %s\n", pty_slave_name);
   }

   // Create symlink to the pty
   ret = symlink(pty_slave_name,pty_symlink_name);
   if ( ret == -1 ) {
      printf("Fail to create pty symlink: %s\n", strerror (errno) );
      syslog(LOG_ERR,"Fail to create pty symlink: %s", strerror (errno) );
   }

   iio_ctx = iio_create_default_context();
   if (!iio_ctx) {
      iio_strerror(errno, buf, sizeof( buf ));
      fprintf(stderr,"Unable to create IIO context: %s\n",buf);
      syslog(LOG_ERR,"Unable to create IIO context: %s",buf);
      exit(EXIT_FAILURE);
   }

   // Get the IIO channels
   iio_ch_cc2_humidity = get_iio_channel_by_name(CHIPCAP2_NAME,iio_ctx,CHIPCAP2_HUMIDITY_VALUE);
   if (iio_ch_cc2_humidity == NULL) {
      iio_strerror(errno, buf, sizeof( buf ));
      fprintf(stderr,"Unable to find the ChipCap 2 IIO channel: %s\n",buf);
      syslog(LOG_ERR,"Unable to find the ChipCap 2 IIO channel: %s",buf);
   }

   iio_ch_cc2_temp = get_iio_channel_by_name(CHIPCAP2_NAME,iio_ctx,CHIPCAP2_TEMP_VALUE);
   if (iio_ch_cc2_temp == NULL) {
      iio_strerror(errno, buf, sizeof( buf ));
      fprintf(stderr,"Unable to find the ChipCap 2 IIO channel: %s\n",buf);
      syslog(LOG_ERR,"Unable to find the ChipCap 2 IIO channel: %s",buf);
   }

   iio_ch_mpl = get_iio_channel_by_name(MPL3115A2_NAME,iio_ctx,MPL3115A2_VALUE);
   if (iio_ch_mpl == NULL) {
      iio_strerror(errno, buf, sizeof( buf ));
      fprintf(stderr,"Unable to find the MPL3115A2 IIO channel: %s\n",buf);
      syslog(LOG_ERR,"Unable to find the MPL3115A2 IIO channel: %s",buf);
   }

   // Fork Daemon
   if (!debug) {
      daemonize();
   }

   while(1) {

      cc2_humidity = 0;
      if (iio_ch_cc2_humidity != NULL) {
         cc2_humidity = iio_get_value(iio_ch_cc2_humidity);
         if (debug) {
            printf("CC2 \tH: %0.2lf %%\n",cc2_humidity);
         }
      }

      cc2_temp = -1000;
      if (iio_ch_cc2_temp != NULL) {
         cc2_temp = iio_get_value(iio_ch_cc2_temp);
         if (debug) {
            printf("CC2 \tT: c: %0.2lf f: %0.2lf k: %0.2lf\n",cc2_temp,
                   ( cc2_temp * 9 / 5 ) + 32, cc2_temp + 273.15 );
         }
      }

      // The mpl311 iio kernel module returns kilopascals.
      // Conversions:
      // - Hectopascals and millbars, hPa or mb = 10 * kPa
      // - Inches Of Mercury inHg = kPa / 3.386
      mpl_pres = 0;
      if (iio_ch_mpl != NULL) {
         mpl_pres = iio_get_value(iio_ch_mpl);
         if (debug) {
            printf("mpl \tP: %0.2lf kPa %0.2lf hPa/mb %0.2lf inHg\n",
                   mpl_pres, 10 * mpl_pres, mpl_pres / 3.386);
         }
      }

      // Get localtime
      time(&rawtime);
      sys_tm = localtime(&rawtime);

      day_min = ( sys_tm->tm_hour * 60 ) + sys_tm->tm_min;

      // Current outdoor temperature (reported as 0.1 deg F increments)
      // Current Barometer (reported in 0.1 mbar increments)
      // Current Outdoor Humidity (reported in 0.1% increments)
      // Date (day of year since January 1)
      // Time (minute of day)
      if (debug) {
         printf("$ULTW00000000%04lX0000%04lX000000000000%04lX%04X%04X00000000\n\n",
                lround( 10 * (( cc2_temp * 9 / 5 ) + 32 ) ), // Temperature
                lround(100 * mpl_pres),                      // Barometer
                lround(10 * cc2_humidity),                   // Humidity
                ( sys_tm->tm_yday ) + 1,                     // Day of the year
                day_min);                                    // Minute of day
      }

      // Print data string to pseudoterminal
      dprintf(fd_pty_master,"$ULTW00000000%04lX0000%04lX000000000000%04lX%04X%04X00000000\n",
              lround( 10 * (( cc2_temp * 9 / 5 ) + 32 ) ),         // Temperature
              lround(100 * mpl_pres),                             // Barometer
              lround(10 * cc2_humidity),                          // Humidity
              ( sys_tm->tm_yday ) + 1,                            // Day of the year
              day_min);

      sleep(60);
   }

   close(fd_pty_master);
   close(fd_pty_slave);
   unlink(pty_symlink_name);
   exit(EXIT_SUCCESS);
}
