/*
** RTEMS Kernel Image shell command setup
**
*/
#include "rki_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <bsp.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <rtems/shell.h>
#include <rtems/bdbuf.h>
#include <rtems/error.h>

/*
** External functions
*/

int  hello_command( int argc, char *argv[]);
int  task_command( int argc, char *argv[]);
int  dhrystone_command( int argc, char *argv[]);
int  whetstone_command( int argc, char *argv[]);
void rki_add_dl_commands(void);
void rki_add_target_cmds(void);

int rki_mpu6050_read_raw_command( int argc, char *argv[]);
int rki_mpu6050_read_proc_command( int argc, char *argv[]);
int rki_mpu6050_open_command( int argc, char *argv[]);

int rki_genuC_send_command( int argc, char *argv[]);
int rki_genuC_open_command( int argc, char *argv[]);

/*
**
** Start the RTEMS Shell.
*/
void shell_start ( void )
{
   rtems_status_code sc;

   printf ("Starting shell....\n\n");

   sc = rtems_shell_init ("shell0", 20 * 1024, 100, "/dev/console", 0, 1,NULL);
   if (sc != RTEMS_SUCCESSFUL)
   {
      printf ("error: starting shell: %s (%d)\n", rtems_status_text(sc), sc);
   }
}

/*
**
** Run the /shell-init script.
*/
void shell_init_script (void)
{
  rtems_status_code sc;
  printf ("Running /shell-init.\n\n");
  sc = rtems_shell_script ("initscr",
                            60 * 1024,    /* Stack size */
                            20,           /* Priority */
                            RKI_SHELL_INIT, /* the Script file to run */
                            "stdout",      /* Where to redirect the output */
                            0,            /* Run once and exit */
                            1,            /* Wait ? */
                            1);           /* Verbose/echo */

  if (sc != RTEMS_SUCCESSFUL)
    printf ("error: running shell script: %s (%d)\n", rtems_status_text (sc), sc);
}

/*
** function to start the shell and add new commands.
*/
int rki_add_local_cmds(void)
{
   /*
   ** Add commands
   */

   rki_add_dl_commands();

   rtems_shell_add_cmd("hello","misc","Say hello RTEMS!",hello_command);

   rtems_shell_add_cmd("taskdemo","misc","Run a set of tasks",task_command);

   rtems_shell_add_cmd("dhrystone","misc","Run the Dhrystone Benchmark",dhrystone_command);

   rtems_shell_add_cmd("whetstone","misc","Run the Whetstone Benchmark",whetstone_command);

   rtems_shell_add_cmd("mpu6050open","misc","Register and open the MPU6050 driver",rki_mpu6050_open_command);

   rtems_shell_add_cmd("mpu6050readraw","misc","Read raw data from the MPU6050 accelerometer and gyroscope",rki_mpu6050_read_raw_command);

   rtems_shell_add_cmd("mpu6050readproc","misc","Read processed data from the MPU6050 accelerometer and gyroscope",rki_mpu6050_read_proc_command);

   rtems_shell_add_cmd("genucopen","misc","Register and open the driver for the generic uC",rki_genuC_open_command);

   rtems_shell_add_cmd("genuctest","misc","Send 0x03 0x06 0x09 to a generic uC",rki_genuC_send_command);

   /*
   ** Add the target specific commands
   ** Find this function in the targets src directory
   ** By default it does not have any commands, but can be used to add
   ** commands that only work on the target (RPI, BeagleBone, etc)
   */
   rki_add_target_cmds();

   shell_init_script();

   /*
   ** Setup the shell
   */
   shell_start ();

   return(0);

}
