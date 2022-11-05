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


rtems_task accel_read(rtems_task_argument unused){

  int status;
  int fd;

  // Registro del sensor
  status = i2c_dev_register_sensor_mpu6050(
    &bus_path[0],
    &mpu6050_path[0]
  );
  printf("El status al registrar es = %d\n",status);

  fd = open(&mpu6050_path[0], O_RDWR);
  if(fd >= 0)
    printf("Se ha abierto el dispositivo correctamente\n");

  // Configuracion del sensor
  status = sensor_mpu6050_set_conf(fd);
  printf("El status al configurar es = %d\n",status);

  printf("Ahora va a imprimir 2 bytes en GYRO_XOUT_H\n");
  status = read_bytes(fd, (uint16_t) 0x68, (uint8_t) GYRO_XOUT_H, 2);

  close(fd);

  //Lectura de los registros
  // int16_t accel_buff[3];
  // int16_t gyro_buff[3];
  //
  // printf("Voy a leer el accelerometer\n");
  //
  // status = sensor_mpu6050_get_accel(accel_buff);
  // printf("Ax = %d \t Ay = %d \t Az = %d\n",accel_buff[0], accel_buff[1], accel_buff[2]);
  // status = sensor_mpu6050_get_gyro(gyro_buff);
  // printf("Gx = %d \t Gy = %d \t Gz = %d\n",gyro_buff[0], gyro_buff[1], gyro_buff[2]);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

int rki_accel_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Accelerometer Demo ***\n" );
	printf( "\n\n  Read the accelerometer and gyroscope values one time\n" );

	task_name = rtems_build_name( 'A', 'X', 'E', 'L' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Accelerometer Demo\n");
	}

	status = rtems_task_start(task_id, accel_read, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Accelerometer Demo\n");
	}

	return(0);
}
