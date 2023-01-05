#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "sensor-aht10.h"

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/i2c-2";
static const char aht10_path[] = "/dev/i2c-2.aht10-0";


rtems_task aht10_read_data(rtems_task_argument unused){

  int fd;

  //Data reading
  fd = open(&aht10_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  printf("Reading data...\n");
  sensor_aht10_read(fd);
  close(fd);

  float temp_buff = sensor_aht10_get_temp();
  float humd_buff = sensor_aht10_get_humid();

  printf("Temperature: %f +-0.3C\n",temp_buff);
  printf("Humidity: %f +-2%%\n",humd_buff);


	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task aht10_open(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_aht10(
    &bus_path[0],
    &aht10_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",aht10_path);

  fd = open(&aht10_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Device initialization
  rv = sensor_aht10_begin(fd);
  if(rv == 0)
    printf("Device AHT10 initialized...\n");

  close(fd);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_aht10_read_proc_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** AHT10 Data ***\n" );
	printf( "Read the temperature and humidity values\n\n" );

	task_name = rtems_build_name( 'A', 'H', 'T', '2' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating AHT10 Data\n");
	}

	status = rtems_task_start(task_id, aht10_read_data, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting AHT10 Data\n");
	}

	return(0);
}

int rki_aht10_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** AHT10 Dev Registration ***\n" );
	printf( "Register and open the driver for the sensor AHT10\n\n" );

	task_name = rtems_build_name( 'A', 'H', 'T', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating AHT10 Dev Registration\n");
	}

	status = rtems_task_start(task_id, aht10_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting AHT10 Dev Registration\n");
	}

	return(0);
}
