#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_I2C_ADDR         0x68
#define MPU6050_PWR_MGMT_1 		 0x6B
#define MPU6050_GYRO_CONFIG		 0x1B
#define MPU6050_ACCEL_CONFIG	 0x1C
#define GYRO_FS_250   			 (0x00 << 3)
#define GYRO_FS_500   		 	 (0x01 << 3)
#define GYRO_FS_1000  			 (0x02 << 3)
#define GYRO_FS_2000  			 (0x03 << 3)
#define MPU6050_CONFIG 			 0x1A
#define MPU6050_FIRST_ACC_BIT	 0x3B


extern I2C_HandleTypeDef hi2c1;

#define GYRO_CALIB_SAMPLES 500

static float gyro_offset_x = 0.f;
static float gyro_offset_y = 0.f;
static float gyro_offset_z = 0.f;


void mpu6050_init(void){

	//powers on gyro
	uint8_t data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, (MPU6050_I2C_ADDR<<1), MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	//sets default +- 2g for accelo
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, (MPU6050_I2C_ADDR<<1), MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	//sets +-500 deg/s for gyro
	data = GYRO_FS_500;
	HAL_I2C_Mem_Write(&hi2c1, (MPU6050_I2C_ADDR<<1), MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	//sets lowpass filter for gyro and accelo
	data = 3;
	HAL_I2C_Mem_Write(&hi2c1, (MPU6050_I2C_ADDR<<1), MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);


	 HAL_Delay(100);

	 //gyro calibration
	 MPU6050_Raw_t raw;
	 float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f;
	 int n = 0;

   	 for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
   		 if (mpu6050_read_all(&raw) == HAL_OK) {
   			 sum_x += raw.gyro_x / 65.5f;
   			 sum_y += raw.gyro_y / 65.5f;
   			 sum_z += raw.gyro_z / 65.5f;
   			 n++;
   		  }
   	  HAL_Delay(2);
   	  }

   	 if (n > 0) {
	 gyro_offset_x = sum_x / n;
	 gyro_offset_y = sum_y / n;
	 gyro_offset_z = sum_z / n;
	}

}

HAL_StatusTypeDef mpu6050_read_all(MPU6050_Raw_t *data){

	uint8_t buf[14];

	if(HAL_I2C_Mem_Read(&hi2c1, (MPU6050_I2C_ADDR<<1), MPU6050_FIRST_ACC_BIT, I2C_MEMADD_SIZE_8BIT, buf, 14, 100) != HAL_OK){
		return HAL_ERROR;
	}

	data->accel_x = (int16_t)((buf[0]<<8) | buf[1]);
	data->accel_y = (int16_t)((buf[2]<<8) | buf[3]);
	data->accel_z = (int16_t)((buf[4]<<8) | buf[5]);

	data->gyro_x = (int16_t)((buf[8]<<8) | buf[9]);
	data->gyro_y = (int16_t)((buf[10]<<8) | buf[11]);
	data->gyro_z = (int16_t)((buf[12]<<8) | buf[13]);


	return HAL_OK;
}


void mpu6050_convert(const MPU6050_Raw_t *raw_data, MPU6050_t *data){

	data->accel_x = (float)raw_data->accel_x / 16384.0f;
	data->accel_y = (float)raw_data->accel_y / 16384.0f;
	data->accel_z = (float)raw_data->accel_z / 16384.0f;


    data->gyro_x = (float)raw_data->gyro_x / 65.5f - gyro_offset_x;
    data->gyro_y = (float)raw_data->gyro_y / 65.5f - gyro_offset_y;
    data->gyro_z = (float)raw_data->gyro_z / 65.5f - gyro_offset_z;

}
