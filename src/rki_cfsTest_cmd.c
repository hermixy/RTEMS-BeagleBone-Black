
#include "cfs-test.h"

/*
** global data
*/
ALTITUDE_APP_HkTlm_Payload_t Telemetry_Payload;

/*
** static function prototypes
*/
static void printPayload();
static void genuC_driver_open();
static void mpu6050_read_data();

rtems_task genuC_telemetry_test(rtems_task_argument unused){

  int rv;

  // Telemetry_Payload initialization
  Telemetry_Payload.AppID[0] = 0x08;
  Telemetry_Payload.AppID[1] = 0xE0;
  Telemetry_Payload.CommandErrorCounter = 0;
  Telemetry_Payload.CommandCounter = 100;

  printf("Opening the genuC driver...\n");
  genuC_driver_open();

  printf("Opening the mpu6050 driver and reading the gyroscope and accelerometer...\n");
  mpu6050_read_data();

  printPayload();

  uint8_t *val;
  val = NULL;
  val = malloc(PAYLOAD_BYTES * sizeof(uint8_t));

  val[0] = Telemetry_Payload.AppID[0];
  val[1] = Telemetry_Payload.AppID[1];
  val[2] = Telemetry_Payload.CommandErrorCounter;
  val[3] = Telemetry_Payload.CommandCounter;

  uint8_t *aux_array;
  for(int i=0;i<3;i++){
      aux_array = NULL;
      aux_array = malloc(4 * sizeof(uint8_t));
      aux_array = (uint8_t*)(&Telemetry_Payload.AccelRead[i]);
      for(int j=0;j<4;j++){
          val[(i+1)*4+j+2] = aux_array[j];
      }
  }

  for(int i=0;i<3;i++){
      aux_array = NULL;
      aux_array = malloc(4 * sizeof(uint8_t));
      aux_array = (uint8_t*)(&Telemetry_Payload.GyroRead[i]);
      for(int j=0;j<4;j++){
          val[(i+4)*4+j+2] = aux_array[j];
      }
  }


  // Send the telemetry payload
  printf("Sending telemetry payload sent correctly...\n");
  rv = uC_set_bytes(UC_ADDRESS, &val, PAYLOAD_BYTES);
  if(rv >= 0){
    printf("Telemetry payload sent correctly...\n");
  }else{
    printf("There was an ERROR...\n");
  }

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

static void printPayload(){
  printf("Telemetry_Payload:\n");
  printf("\t[AppID] = 0x%x%x\n", Telemetry_Payload.AppID[0], Telemetry_Payload.AppID[1]);
  printf("\t[CommandErrorCounter] = %d\n", Telemetry_Payload.CommandErrorCounter);
  printf("\t[CommandCounter] = %d\n", Telemetry_Payload.CommandCounter);
  printf("\t[spare][0] = \n");
  printf("\t[spare][1] = \n");
  printf("\t[AccelRead][0] = %f\n", Telemetry_Payload.AccelRead[0]);
  printf("\t[AccelRead][1] = %f\n", Telemetry_Payload.AccelRead[1]);
  printf("\t[AccelRead][2] = %f\n", Telemetry_Payload.AccelRead[2]);
  printf("\t[GyroRead][0] = %f\n", Telemetry_Payload.GyroRead[0]);
  printf("\t[GyroRead][1] = %f\n", Telemetry_Payload.GyroRead[1]);
  printf("\t[GyroRead][2] = %f\n", Telemetry_Payload.GyroRead[2]);
}

static void genuC_driver_open(){

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

}

static void mpu6050_read_data(){

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
  printf("Device configured correctly...\n");

  close(fd);

  fd = open(&bus_path[0], O_RDWR);
  if(fd >= 0)
    printf("Bus opened correctly...\n");
  close(fd);

  //Data reading
  sensor_mpu6050_read_data();
  SENSOR_MPU6050_Data_t mpu = sensor_mpu6050_get_data();
  Telemetry_Payload.AccelRead[0] = mpu.accX;
  Telemetry_Payload.AccelRead[1] = mpu.accY;
  Telemetry_Payload.AccelRead[2] = mpu.accZ;
  Telemetry_Payload.GyroRead[0] = mpu.gyroX;
  Telemetry_Payload.GyroRead[1] = mpu.gyroY;
  Telemetry_Payload.GyroRead[2] = mpu.gyroZ;
}

int rki_genuC_telemetry_test_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** Telemetry over I2C test ***\n" );
	printf( "Send a Telemetry payload via I2C\n\n" );

	task_name = rtems_build_name( 'T', 'P', 'T', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating Telemetry over I2C test\n");
	}

	status = rtems_task_start(task_id, genuC_telemetry_test, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting Telemetry over I2C test\n");
	}

	return(0);
}
