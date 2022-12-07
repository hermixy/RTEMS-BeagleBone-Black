#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "gen-uC.h"

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/i2c-2";
static const char genuC_path[] = "/dev/i2c-2.genuC-0";

rtems_task genuC_send(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_uC(
    &bus_path[0],
    &genuC_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",genuC_path);

  fd = open(&genuC_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Send Test
  rv = uC_send_test(fd);
  printf("Send test made...\n");

  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task genuC_open(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_uC(
    &bus_path[0],
    &genuC_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",genuC_path);

  fd = open(&genuC_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");
  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_genuC_send_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Generic uC Send Test ***\n" );
	printf( "Send 0x03 0x06 0x09 to a generic uC in the address 0x36\n\n" );

	task_name = rtems_build_name( 'G', 'U', 'C', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Generic uC Send Test\n");
	}

	status = rtems_task_start(task_id, genuC_send, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Generic uC Send Test\n");
	}

	return(0);
}

int rki_genuC_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Generic uC Dev Registration ***\n" );
	printf( "Register and open the driver for the generic uC\n\n" );

	task_name = rtems_build_name( 'G', 'U', 'C', '0' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Generic uC Dev Registration\n");
	}

	status = rtems_task_start(task_id, genuC_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Generic uC Dev Registration\n");
	}

	return(0);
}
