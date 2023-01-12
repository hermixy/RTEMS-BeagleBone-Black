#include <stdio.h>
#include <string.h>
#include <bsp.h>

#include "sensor-mpu6050.h"

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

static const char bus_path[] = "/dev/i2c-2";
static const char mpu6050_path[] = "/dev/i2c-2.mpu6050-0";

rtems_task mpu6050_read_angles(rtems_task_argument unused){
  SENSOR_MPU6050_Data_t mpu;

  //Data reading
  printf("Calculating offsets, do not move MPU6050...\n");
  sensor_mpu6050_calcOffsets();
  printf("Done calculating offsets...\n");
  for (int i = 0; i < 100; i++) {
    sensor_mpu6050_read_data();
    mpu = sensor_mpu6050_get_data();
    printf("X = %.4f \t Y = %.4f \t Z = %.4f\n",mpu.angleX, mpu.angleY, mpu.angleZ);
    rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpu6050_read_accel_gyro(rtems_task_argument unused){
  SENSOR_MPU6050_Data_t mpu;

  //Data reading
  printf("\n*** MPU6050 Accelerometer reading ***\n");
  for (int i = 0; i < 50; i++) {
    sensor_mpu6050_read_data();
    mpu = sensor_mpu6050_get_data();
    printf("Ax = %.4f \t Ay = %.4f \t Az = %.4f\n",mpu.accX, mpu.accY, mpu.accZ);
    rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }

  printf("\n\n*** MPU6050 Gyroscope reading ***\n");
  for (int i = 0; i < 50; i++) {
    sensor_mpu6050_read_data();
    mpu = sensor_mpu6050_get_data();
    printf("Gx = %.4f \t Gy = %.4f \t Gz = %.4f\n",mpu.gyroX, mpu.gyroY, mpu.gyroZ);
    rtems_task_wake_after( 0.1 * rtems_clock_get_ticks_per_second() );
  }

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task mpu6050_read_temp(rtems_task_argument unused){

  //Data reading
  printf("\n*** MPU6050 Temperature reading ***\n");

  float temp = sensor_mpu6050_get_temp(); // This function does the reading
  printf("Temperature = %.3f +-1C\n",temp);

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
  rv = sensor_mpu6050_begin(fd);
  printf("Device initialized correctly...\n");

  close(fd);

  printf("Calculating offsets, do not move MPU6050...\n");
  sensor_mpu6050_calcOffsets();
  printf("Done calculating offsets...\n");

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_mpu6050_read_angles_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Angles RPY ***\n" );
	printf( "Read the RPY angles calculated from the MPU6050\n\n" );

	task_name = rtems_build_name( 'I', 'M', 'U', '3' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating IMU Angles RPY\n");
	}

	status = rtems_task_start(task_id, mpu6050_read_angles, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting IMU Angles RPY\n");
	}

	return(0);
}

int rki_mpu6050_read_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Data ***\n" );
	printf( "Read the accelerometer and gyroscope values\n\n" );

	task_name = rtems_build_name( 'I', 'M', 'U', '2' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating IMU Data\n");
	}

	status = rtems_task_start(task_id, mpu6050_read_accel_gyro, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting IMU Data\n");
	}

	return(0);
}

int rki_mpu6050_read_temp_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Temperature ***\n" );
	printf( "Read the temperature registered by the MPU6050\n\n" );

	task_name = rtems_build_name( 'I', 'M', 'U', '4' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating IMU Temperature\n");
	}

	status = rtems_task_start(task_id, mpu6050_read_temp, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting IMU Temperature\n");
	}

	return(0);
}

int rki_mpu6050_open_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** MPU6050 Dev Registration ***\n" );
	printf( "Register and open the driver for the IMU\n\n" );

	task_name = rtems_build_name( 'I', 'M', 'U', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating IMU Dev Registration\n");
	}

	status = rtems_task_start(task_id, mpu6050_open, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting IMU Dev Registration\n");
	}

	return(0);
}
