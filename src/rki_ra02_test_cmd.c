#include <dev/spi/spi.h>
// #include <bsp/spi.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <bsp.h> /* for device driver prototypes */
#include <bsp/gpio.h>

//define some constant variables
#define PATH "/dev/spi-0"

static	uint16_t delay =0;
static	uint32_t speed = 100000;
static uint8_t bits = 8;
static uint8_t mode = 0;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define TASK_ATTRIBUTES RTEMS_FLOATING_POINT

void write_data(int fd){

	int ret;

	char tx[]= {0x08,0x01,0x04};

	char rx[ARRAY_SIZE(tx)] = {0, };

	struct spi_ioc_transfer tr ={
		.tx_buf = (unsigned long) tx,
		.rx_buf = (unsigned long) rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd,SPI_IOC_MESSAGE(1),&tr);
	if (ret<0){
		printf("SPI_IOC_MESSAGE");
	}
}

rtems_task spi_test(rtems_task_argument unused){

  int ret,fd;

	fd = open(PATH,O_RDWR);
	if (fd<0){
		printf("Bad fd open - check PATH");}

	//setup SPI mode
	ret = ioctl(fd,SPI_IOC_WR_MODE,&mode);
	if(ret<0){
		printf("SPI_IOC_WR_MODE");}

	ret = ioctl(fd,SPI_IOC_RD_MODE,&mode);
	if(ret<0){
		printf("SPI_IOC_RD_MODE");}

	ret = ioctl(fd,SPI_IOC_WR_BITS_PER_WORD,&bits);
	if(ret<0){
		printf("SPI_IOC_WR_BITS_PER_WORD");}

	ret = ioctl(fd,SPI_IOC_RD_BITS_PER_WORD,&bits);
	if(ret<0){
		printf("SPI_IOC_RD_BITS_PER_WORD");}

	ret = ioctl(fd,SPI_IOC_WR_MAX_SPEED_HZ,&speed);
	if(ret<0){
		printf("SPI_IOC_WR_MAX_SPEED_HZ");}

	ret = ioctl(fd,SPI_IOC_RD_MAX_SPEED_HZ,&speed);
	if(ret<0){
		printf("SPI_IOC_RD_MAX_SPEED_HZ");}

	write_data(fd); // send data

	ret = rtems_task_wake_after( 1 * rtems_clock_get_ticks_per_second() );
	close(fd);

	rtems_task_delete( RTEMS_SELF );    /* should not return */
}

rtems_task spi_register(rtems_task_argument unused){
	rtems_status_code status = bsp_register_spi(
    "/dev/spi-0",
    0x48030000,	//SPI0 register
    65
  );

	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error at registering SPI device /dev/spi-0");
	}

}

int rki_spi_register_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** SPI Test ***\n" );
	printf( "Register the SPI device\n\n" );

	task_name = rtems_build_name( 'S', 'P', 'I', '0' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating SPI Test\n");
	}

	status = rtems_task_start(task_id, spi_register, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting SPI Test\n");
	}

	return(0);
}

int rki_spi_test_command( int argc, char *argv[]){
	rtems_status_code status;
	rtems_id   task_id;         /* task id */
	rtems_name task_name;       /* task name */

	printf( "\n\n*** SPI Test ***\n" );
	printf( "Test made for the the SPI communication\n\n" );

	task_name = rtems_build_name( 'S', 'P', 'I', '1' );

	status = rtems_task_create(
	task_name, 1, RTEMS_MINIMUM_STACK_SIZE * 2, RTEMS_DEFAULT_MODES,
	TASK_ATTRIBUTES, &task_id
	);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error creating SPI Test\n");
	}

	status = rtems_task_start(task_id, spi_test, 0);
	if ( status != RTEMS_SUCCESSFUL )
	{
		printf("Error Starting SPI Test\n");
	}

	return(0);
}
