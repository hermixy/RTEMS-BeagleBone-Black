#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "sensor-mpu6050.h"
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/i2c-2";
static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";


rtems_task mpu6050_read_raw(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_mpu6050(
    &bus_path[0],
    &mpu6050_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",mpu6050_path);

  fd = open(&mpu6050_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Device configuration
  rv = sensor_mpu6050_set_conf(fd);
  printf("Device configured correctly...\n");

  close(fd);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

  //Data reading
  int16_t *accel_buff;
  int16_t *gyro_buff;

  printf("\n*** Accelerometer reading ***\n");
  for (int i = 0; i < 50; i++) {
    accel_buff = NULL;
    rv = sensor_mpu6050_get_accel(&accel_buff);
    printf("Ax = %d \t Ay = %d \t Az = %d\n",accel_buff[0], accel_buff[1], accel_buff[2]);

    rv = rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }
  free(accel_buff);

  printf("\n\n*** Gyroscope reading ***\n");
  for (int i = 0; i < 50; i++) {
    gyro_buff = NULL;
    rv = sensor_mpu6050_get_gyro(&gyro_buff);
    printf("Gx = %d \t Gy = %d \t Gz = %d\n",gyro_buff[0], gyro_buff[1], gyro_buff[2]);

    rv = rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }
  free(gyro_buff);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpu6050_read_proc(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_mpu6050(
    &bus_path[0],
    &mpu6050_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",mpu6050_path);

  fd = open(&mpu6050_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Device configuration
  rv = sensor_mpu6050_set_conf(fd);
  printf("Device configured correctly...\n");

  close(fd);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

  //Data reading
  int16_t *accel_buff;
  int16_t *gyro_buff;

  printf("\n*** Accelerometer reading in m/s2***\n");
  for (int i = 0; i < 50; i++) {
    accel_buff = NULL;
    rv = sensor_mpu6050_get_accel(&accel_buff);

    float ax_m_s2 = accel_buff[0] * (9.81/16384.0);
    float ay_m_s2 = accel_buff[1] * (9.81/16384.0);
    float az_m_s2 = accel_buff[2] * (9.81/16384.0);
    printf("Ax = %.2f \t Ay = %.2f \t Az = %.2f\n",ax_m_s2, ay_m_s2, az_m_s2);

    rv = rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }
  free(accel_buff);

  printf("\n\n*** Gyroscope reading in deg/s***\n");
  for (int i = 0; i < 50; i++) {
    gyro_buff = NULL;
    rv = sensor_mpu6050_get_gyro(&gyro_buff);

    float gx_deg_s = gyro_buff[0] * (250.0/32768.0);
    float gy_deg_s = gyro_buff[1] * (250.0/32768.0);
    float gz_deg_s = gyro_buff[2] * (250.0/32768.0);
    printf("Gx = %.2f \t Gy = %.2f \t Gz = %.2f\n",gx_deg_s, gy_deg_s, gz_deg_s);

    rv = rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }
  free(gyro_buff);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpu6050_open(rtems_task_argument unused){

  int rv;
  int fd;

  // Device registration
  rv = i2c_dev_register_sensor_mpu6050(
    &bus_path[0],
    &mpu6050_path[0]
  );
  if(rv == 0)
    printf("Device registered correctly at %s\n",mpu6050_path);

  fd = open(&mpu6050_path[0], O_RDWR);
  if(fd >= 0)
    printf("Device opened correctly...\n");

  // Device configuration
  rv = sensor_mpu6050_set_conf(fd);
  printf("Device configured correctly...\n");

  close(fd);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_mpu6050_read_raw_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Raw Data ***\n" );
	printf( "Read the accelerometer and gyroscope values\n\n" );

	task_name = rtems_build_name( 'A', 'X', 'E', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Accelerometer Raw Data\n");
	}

	status = rtems_task_start(task_id, mpu6050_read_raw, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Accelerometer Raw Data\n");
	}

	return(0);
}

int rki_mpu6050_read_proc_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Processed Data ***\n" );
	printf( "Read the accelerometer and gyroscope values\n\n" );

	task_name = rtems_build_name( 'A', 'X', 'E', '2' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Accelerometer Processed Data\n");
	}

	status = rtems_task_start(task_id, mpu6050_read_proc, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Accelerometer Processed Data\n");
	}

	return(0);
}

int rki_mpu6050_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Dev Registration ***\n" );
	printf( "Register and open the driver for the accelerometer\n\n" );

	task_name = rtems_build_name( 'A', 'X', 'E', '3' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Accelerometer Dev Registration\n");
	}

	status = rtems_task_start(task_id, mpu6050_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Accelerometer Dev Registration\n");
	}

	return(0);
}
