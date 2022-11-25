#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "LoRa.h"

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/spi-0";


rtems_task ra02_read_regVer(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = spi_bus_register_ra02();
  if(rv == 0)
    printf("Bus registered correctly at %s\n",bus_path);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");


  //Data reading
  int16_t *regVer_buff;

  printf("\n*** Register Version reading ***\n");

  rv = module_ra02_begin(fd);
  printf("Starting LoRa status = %d",rv);

  regVer_buff = NULL;
  readRegister(REG_VERSION, &regVer_buff);
  printf("The register version is: %02x",*regVer_buff);
  if (*regVer_buff != 0x12) {
    return 0;
  }

  close(fd);
  free(regVer_buff);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_ra02_read_regVer_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Ra-02 Reg Version ***\n" );
	printf( "Read the reg version register value to test the SPI communication\n\n" );

	task_name = rtems_build_name( 'R', 'F', '0', '2' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating RF Reg Version\n");
	}

	status = rtems_task_start(task_id, ra02_read_regVer, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting RF Reg Version\n");
	}

	return(0);
}

rtems_task ra02_open(rtems_task_argument unused){

  int rv;
  int fd;

  // Bus registration
  rv = spi_bus_register_ra02();
  if(rv == 0)
    printf("Bus registered correctly at %s\n",bus_path);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_ra02_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Ra-02 Bus Registration ***\n" );
	printf( "Register and open the bus for the LoRa Ra-02 module\n\n" );

	task_name = rtems_build_name( 'R', 'F', '0', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating RF Bus Registration\n");
	}

	status = rtems_task_start(task_id, ra02_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting RF Bus Registration\n");
	}

	return(0);
}
