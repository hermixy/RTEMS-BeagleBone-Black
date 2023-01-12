#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "sensor-mpl3115a2.h"

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/i2c-2";
static const char mpl3115a2_path[] = "/dev/i2c-2.mpl3115a2-0";

rtems_task mpl3115a2_read_altitude(rtems_task_argument unused){

  printf("\n*** Altimeter reading ***\n");
  printf("Setting sea level pressure to 1013.26 hPA...\n");
  sensor_mpl3115a2_setSeaPressure(1013.26);

  float altitude;
  printf("Reading 5 times the altimeter...\n");
  for (int i = 0; i < 5; i++) {
    altitude = sensor_mpl3115a2_getAltitude();
    printf("Altitude: %.3f m\n",altitude);
    rtems_task_wake_after( 0.25 * rtems_clock_get_ticks_per_second() );
  }

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpl3115a2_read_temp(rtems_task_argument unused){

  printf("\n*** MPL3115A2 Temperature reading ***\n");

  float temperature;
  temperature = sensor_mpl3115a2_getTemperature();
  printf("Temperature: %.3f +-1C\n",temperature);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpl3115a2_open(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_mpl3115a2(
    &bus_path[0],
    &mpl3115a2_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",mpl3115a2_path);

  fd = open(&mpl3115a2_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Device configuration
  rv = sensor_mpl3115a2_begin(fd);
  if(rv != 0){
    printf("Could not find sensor. Check wiring.\n");
  }else{
    printf("Device configured correctly...\n");
  }

  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_mpl3115a2_read_alt_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPL3115A2 Altitude Data ***\n" );
	printf( "Read the altitude data\n\n" );

	task_name = rtems_build_name( 'A', 'L', 'T', '2' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating MPL3115A2 Altitude Data\n");
	}

	status = rtems_task_start(task_id, mpl3115a2_read_altitude, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting MPL3115A2 Altitude Data\n");
	}

	return(0);
}

int rki_mpl3115a2_read_temp_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPL3115A2 Temperature Data ***\n" );
	printf( "Read the temperature data\n\n" );

	task_name = rtems_build_name( 'A', 'L', 'T', '3' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating MPL3115A2 Temperature Data\n");
	}

	status = rtems_task_start(task_id, mpl3115a2_read_temp, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting MPL3115A2 Temperature Data\n");
	}

	return(0);
}

int rki_mpl3115a2_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPL3115A2 Dev Registration ***\n" );
	printf( "Register and open the driver for the Pressure sensor\n\n" );

	task_name = rtems_build_name( 'A', 'L', 'T', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating MPL3115A2 Dev Registration\n");
	}

	status = rtems_task_start(task_id, mpl3115a2_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting MPL3115A2 Dev Registration\n");
	}

	return(0);
}
