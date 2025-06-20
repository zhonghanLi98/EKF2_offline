#include "Myekf2.h"
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <string>
#include <iostream>

class Ekf2;
// 将文件流的定义移出全局范围，使其可以在函数内部根据路径参数创建
// std::ifstream read0("../../data/accel_data.txt");
// ...

bool bReadGPS, bReadMag, bReadBaro;


namespace ekf2
{
Ekf2 *instance = nullptr;
}

Ekf2::Ekf2():
	_ekf(),
	_params(_ekf.getParamHandle())
{

}

Ekf2::~Ekf2()
{

}


void Ekf2::print_status()
{
	printf("local position OK %s", (_ekf.local_position_is_valid()) ? "[YES]" : "[NO]");
	printf("global position OK %s", (_ekf.global_position_is_valid()) ? "[YES]" : "[NO]");
}

// 修改 task_main 以只接收一个统一的数据目录路径
void Ekf2::task_main(const std::string& data_dir)
{
	// 根据传入的路径创建文件流
	std::ifstream read0(data_dir + "/accel_data.txt");
	std::ifstream read1(data_dir + "/gyro_data.txt");
	std::ifstream read2(data_dir + "/gps_data.txt");
	std::ifstream read3(data_dir + "/mag_data.txt");
	std::ifstream read4(data_dir + "/baro_data.txt");

	std::ofstream euler_estimator(data_dir + "/euler_estimator.txt");
	std::ofstream position_estimator(data_dir + "/position_estimator.txt");
	std::ofstream estimated_state_log(data_dir + "/estimated_state.txt");
	std::ofstream vel_pos_innov_log(data_dir + "/vel_pos_innov.txt");
	std::ofstream vel_pos_innov_var_log(data_dir + "/vel_pos_innov_var.txt");
	std::ofstream mag_innov_log(data_dir + "/mag_innov.txt");
	std::ofstream mag_innov_var_log(data_dir + "/mag_innov_var.txt");

	// 检查文件是否成功打开
	if (!read0.is_open()) { printf("Error: Cannot open Accelerometer data file: %s\n", (data_dir + "/accel_data.txt").c_str()); return; }
	if (!read1.is_open()) { printf("Error: Cannot open Gyroscope data file: %s\n", (data_dir + "/gyro_data.txt").c_str()); return; }
	if (!read2.is_open()) { printf("Error: Cannot open GPS data file: %s\n", (data_dir + "/gps_data.txt").c_str()); return; }
	if (!read3.is_open()) { printf("Error: Cannot open MAG data file: %s\n", (data_dir + "/mag_data.txt").c_str()); return; }
	if (!read4.is_open()) { printf("Error: Cannot open BARO data file: %s\n", (data_dir + "/baro_data.txt").c_str()); return; }
	if (!euler_estimator.is_open() || !position_estimator.is_open() || !estimated_state_log.is_open() ||
		!vel_pos_innov_log.is_open() || !vel_pos_innov_var_log.is_open() || !mag_innov_log.is_open() || !mag_innov_var_log.is_open()) {
		printf("Error: Cannot open one or more output files in directory: %s\n", data_dir.c_str());
		return;
	}
	printf("All data files opened successfully from/to directory: %s\n", data_dir.c_str());

	bReadGPS = true;
	bReadMag = true;
	bReadBaro = true;


	// initialise parameter cache// TODO
	//updateParams();
//	std::ifstream read1("data/imu_data.txt");

	float gyro_integral_dt = 0;
	float accelerometer_integral_dt = 0;
	float last_IMUtime = 0;
    float now = 0;
    double gps_time_ms = 0, lat, lon, alt, vel_n, vel_e, vel_d, eph, epv, satellites;
    double mag_time_ms_read = 0, magx, magy, magz;
    double baro_time_ms_read = 0, baroHeight, baroHeight_origin = 0.0f;

	//while (!_task_should_exit && !read1.eof() && !read2.eof() && !read3.eof() && !read4.eof()) {
	while (!_task_should_exit && read0.good() && read1.good() && read2.good() && read3.good() && read4.good()) {

		bool isa = true;
		bool mag_updated = false;
		bool baro_updated = false;
		bool gps_updated = false;
		bool vehicle_status_updated = false;

		// IMU数据缓冲区结构
		struct IMUBuffer {
			bool has_data;
			double timestamp_ms;
			float data[3];
			float temperature;
			int device_id;
			int error_count;
			int clip_counter;
			float last_used_time;
		};
		
		// 移除 static，使其成为每次调用都重新初始化的局部变量
		IMUBuffer accel_buffer = {false, 0, {0}, 0, 0, 0, 0, 0};
		IMUBuffer gyro_buffer = {false, 0, {0}, 0, 0, 0, 0, 0};
		
		// 移除 static
		float last_processed_accel[3] = {0, 0, 0};
		
		// 尝试填充加速度计缓冲区
		if (!accel_buffer.has_data) {
			float temp_device_id, temp_error_count, temp_clip_counter;
			if (read0 >> accel_buffer.timestamp_ms >> accel_buffer.data[0] >> accel_buffer.data[1] >> accel_buffer.data[2] 
				>> accel_buffer.temperature >> temp_device_id >> temp_error_count >> temp_clip_counter) {
				accel_buffer.device_id = (int)temp_device_id;
				accel_buffer.error_count = (int)temp_error_count;
				accel_buffer.clip_counter = (int)temp_clip_counter;
				accel_buffer.has_data = true;
				printf("Accel buffered: time=%.3f, data=[%.6f,%.6f,%.6f]\n", 
					   accel_buffer.timestamp_ms, accel_buffer.data[0], accel_buffer.data[1], accel_buffer.data[2]);
			} else {
				printf("Accelerometer data finished, ending loop\n");
				break;
			}
		}
		
		// 尝试填充陀螺仪缓冲区
		if (!gyro_buffer.has_data) {
			float temp_device_id, temp_error_count, temp_clip_counter;
			if (read1 >> gyro_buffer.timestamp_ms >> gyro_buffer.data[0] >> gyro_buffer.data[1] >> gyro_buffer.data[2] 
				>> gyro_buffer.temperature >> temp_device_id >> temp_error_count >> temp_clip_counter) {
				gyro_buffer.device_id = (int)temp_device_id;
				gyro_buffer.error_count = (int)temp_error_count;
				gyro_buffer.clip_counter = (int)temp_clip_counter;
				gyro_buffer.has_data = true;
				printf("Gyro buffered: time=%.3f, data=[%.6f,%.6f,%.6f]\n", 
					   gyro_buffer.timestamp_ms, gyro_buffer.data[0], gyro_buffer.data[1], gyro_buffer.data[2]);
			} else {
				printf("Gyroscope data finished, ending loop\n");
				break;
			}
		}
		
		// 检查是否有完整的IMU数据对 - 这是主时钟驱动条件
		if (accel_buffer.has_data && gyro_buffer.has_data) {
			// 配对成功！使用后到达的时间戳作为主时间戳
			float accel_time_us = accel_buffer.timestamp_ms * 1.e3f;
			float gyro_time_us = gyro_buffer.timestamp_ms * 1.e3f;
			now = std::max(accel_time_us, gyro_time_us);  // 主时钟时间戳
			
			printf("IMU pair matched! accel_time=%.3f, gyro_time=%.3f, now=%.3f\n", 
				   accel_buffer.timestamp_ms, gyro_buffer.timestamp_ms, now/1000.0f);
			
			// 分别计算积分时间（第一次使用时设置合理的初始dt）
			if (accel_buffer.last_used_time == 0) {
				accelerometer_integral_dt = 0.01f;  // 10ms
				accel_buffer.last_used_time = accel_time_us;
			} else {
				accelerometer_integral_dt = (accel_time_us - accel_buffer.last_used_time) / 1.e6f;
				accel_buffer.last_used_time = accel_time_us;
			}
			
			if (gyro_buffer.last_used_time == 0) {
				gyro_integral_dt = 0.01f;  // 10ms
				gyro_buffer.last_used_time = gyro_time_us;
			} else {
				gyro_integral_dt = (gyro_time_us - gyro_buffer.last_used_time) / 1.e6f;
				gyro_buffer.last_used_time = gyro_time_us;
			}
			
			// 计算积分值
			float gyro_integral[3] = {
				gyro_buffer.data[0] * gyro_integral_dt,
				gyro_buffer.data[1] * gyro_integral_dt,
				gyro_buffer.data[2] * gyro_integral_dt
			};
			
			float accel_integral[3] = {
				accel_buffer.data[0] * accelerometer_integral_dt,
				accel_buffer.data[1] * accelerometer_integral_dt,
				accel_buffer.data[2] * accelerometer_integral_dt
			};
			
			// 发送配对的IMU数据到EKF
			_ekf.setIMUData(now, gyro_integral_dt * 1.e6f, accelerometer_integral_dt * 1.e6f,
					gyro_integral, accel_integral);
			last_IMUtime = now;
			
			printf("IMU pair sent to EKF: time=%.3f, gyro_dt=%.6f, accel_dt=%.6f\n",
				   now/1000.0f, gyro_integral_dt, accelerometer_integral_dt);
			
			// 保存最后处理的加速度计数据
			last_processed_accel[0] = accel_buffer.data[0];
			last_processed_accel[1] = accel_buffer.data[1];
			last_processed_accel[2] = accel_buffer.data[2];
			
			// 清空缓冲区，准备接收下一对数据
			accel_buffer.has_data = false;
			gyro_buffer.has_data = false;
		} else {
			// 缓冲区未满，跳过其他传感器处理，继续下一次循环获取IMU数据
			continue;
		}

		if(bReadMag)
		{
			// 读取磁力计数据: timestamp_ms, x, y, z, temperature, device_id, error_count
			float mag_temp, temp_device_id, temp_error_count;
			
			if (!(read3 >> mag_time_ms_read >> magx >> magy >> magz 
				  >> mag_temp >> temp_device_id >> temp_error_count)) {
				printf("Magnetometer data finished, no more mag updates available\n");
				bReadMag = false;
				// 磁力计数据读完，但继续处理其他传感器
			} else {
				printf("Read mag data: time=%.3f, mag=[%.6f,%.6f,%.6f], temp=%.1f\n", 
					   mag_time_ms_read, magx, magy, magz, mag_temp);
				bReadMag = false;
			}
		}
		
		// 检查磁力计时间同步条件
		if(mag_time_ms_read * 1.e3f <= now)
		{
			mag_updated = true;
			bReadMag = true;
			printf("Mag time check passed: mag_time=%.3f <= now=%.3f\n", 
				   mag_time_ms_read, now/1000.0f);
		}
		
		if(mag_updated)
		{
			_timestamp_mag_us = mag_time_ms_read * 1.e3f;

			// If the time last used by the EKF is less than specified, then accumulate the
			// data and push the average when the 50msec is reached.
			_mag_time_sum_ms += _timestamp_mag_us / 1000.0f;
			_mag_sample_count++;
			_mag_data_sum[0] += magx;
			_mag_data_sum[1] += magy;
			_mag_data_sum[2] += magz;
			uint32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;
			
			if (mag_time_ms - _mag_time_ms_last_used > _params->sensor_interval_min_ms) {
				float mag_sample_count_inv = 1.0f / (float)_mag_sample_count;
				float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv, 
											_mag_data_sum[1] *mag_sample_count_inv, 
											_mag_data_sum[2] *mag_sample_count_inv};
				_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);
				printf("Mag data sent to EKF: time=%d, data=[%.3f,%.3f,%.3f], count=%d\n",
						mag_time_ms, mag_data_avg_ga[0], mag_data_avg_ga[1], mag_data_avg_ga[2], _mag_sample_count);
				_mag_time_ms_last_used = mag_time_ms;
				_mag_time_sum_ms = 0;
				_mag_sample_count = 0;
				_mag_data_sum[0] = 0.0f;
				_mag_data_sum[1] = 0.0f;
				_mag_data_sum[2] = 0.0f;	
			}		
		}


		if(bReadBaro)
		{
			// 读取气压计数据: timestamp_ms, altitude_m, temperature, device_id, error_count
			// 注意：Python脚本已经将气压转换为高度，直接读取高度值
			float baro_temp, temp_device_id, temp_error_count;
			
			if (!(read4 >> baro_time_ms_read >> baroHeight >> baro_temp 
				  >> temp_device_id >> temp_error_count)) {
				printf("Barometer data finished, no more baro updates available\n");
				bReadBaro = false;
				// 气压计数据读完，但继续处理其他传感器
			} else {
				printf("Read baro data: time=%.3f, altitude=%.2f m, temp=%.1f\n", 
					   baro_time_ms_read, baroHeight, baro_temp);
				bReadBaro = false;
			}
		}
		if(baro_time_ms_read *1.e3f <= now)
		{
			baro_updated = true;
			bReadBaro = true;
		}
		
		if(baro_updated)
		{
			_timestamp_balt_us = baro_time_ms_read * 1.e3f;

				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the 50msec is reached.
				_balt_time_sum_ms += _timestamp_balt_us / 1000;
				_balt_sample_count++;
				_balt_data_sum += baroHeight;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

			if (balt_time_ms - _balt_time_ms_last_used > (uint32_t)_params->sensor_interval_min_ms) {
				float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;
				_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);
				printf("Baro data sent to EKF: time=%d, altitude=%.2f m, count=%d\n",
						balt_time_ms, balt_data_avg, _balt_sample_count);
				_balt_time_ms_last_used = balt_time_ms;
				_balt_time_sum_ms = 0;
				_balt_sample_count = 0;
				_balt_data_sum = 0.0f;
			}			
		}

		if(bReadGPS)
		{
			// 读取GPS数据: timestamp_ms, lat, lon, alt, vel_n, vel_e, vel_d, eph, epv, satellites, fix_type
			// 使用浮点数临时变量处理浮点数格式的satellites和fix_type
			float temp_satellites, temp_fix_type;
			
			if (!(read2 >> gps_time_ms >> lat >> lon >> alt 
				  >> vel_n >> vel_e >> vel_d >> eph >> epv >> temp_satellites >> temp_fix_type)) {
				printf("GPS data finished, no more GPS updates available\n");
				bReadGPS = false;
				// GPS数据读完，但继续处理其他传感器
			} else {
				// 转换为整数
				satellites = temp_satellites;
				int fix_type = (int)temp_fix_type;
				
				printf("Read GPS data: time=%.3f, lat=%.7f, lon=%.7f, alt=%.2f, sats=%.0f, fix=%d\n", 
					   gps_time_ms, lat/1e7, lon/1e7, alt/1000.0, satellites, fix_type);
				bReadGPS = false;
			}
		}
		
		// 检查GPS时间同步条件（原作者方式）
		if(gps_time_ms * 1.e3f <= now)
		{
			gps_updated = true;
			bReadGPS = true;
		}
		
		if(gps_updated)
		{
			struct gps_message gps_msg = {};
			gps_msg.time_usec = (uint64_t)(gps_time_ms * 1.e3f);
			// GPS数据已经是PX4原始格式，直接使用
			gps_msg.lat = (int32_t)lat;        // 已经是1e7格式
			gps_msg.lon = (int32_t)lon;        // 已经是1e7格式  
			gps_msg.alt = (int32_t)alt;        // 已经是mm格式
			
			gps_msg.fix_type = 3;
			gps_msg.eph = (float)eph;
			gps_msg.epv = (float)epv;
			gps_msg.sacc = (eph < 1.0f) ? 0.2f : 0.5f;  // GPS速度精度，根据位置精度动态设置
			gps_msg.vel_m_s = sqrt(vel_n*vel_n + vel_e*vel_e);  // 水平速度模长
			gps_msg.vel_ned[0] = (float)vel_n;	// 北向速度 (m/s)
			gps_msg.vel_ned[1] = (float)vel_e;	// 东向速度 (m/s)
			gps_msg.vel_ned[2] = (float)vel_d;	// 下向速度 (m/s)
			gps_msg.vel_ned_valid = 1;
			gps_msg.nsats = (uint8_t)satellites;
			gps_msg.gdop = 0.0f;

			printf("GPS message sent to EKF: lat=%d, lon=%d, alt=%d, vel=[%.3f,%.3f,%.3f]\n",
				   gps_msg.lat, gps_msg.lon, gps_msg.alt, 
				   gps_msg.vel_ned[0], gps_msg.vel_ned[1], gps_msg.vel_ned[2]);

			_ekf.setGpsData(gps_msg.time_usec, &gps_msg);
		}

		//run the EKF update and output
		if (_ekf.update()) {
			// 获取EKF内部的时间戳，确保时间一致性
			uint64_t ekf_timestamp;
			_ekf.copy_timestamp(&ekf_timestamp);
			float ekf_time_sec = ekf_timestamp / 1.e6f;

			matrix::Quaternion<float> q;
			_ekf.copy_quaternion(q.data());

			float velocity[3];
			_ekf.get_velocity(velocity);
			//printf("velocity: %lf,%lf,%lf\n", velocity[0], velocity[1], velocity[2]);

			float position[3];
			_ekf.get_position(position);

			// Write to estimated_state.txt
			estimated_state_log << ekf_time_sec << " " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " "
					    << velocity[0] << " " << velocity[1] << " " << velocity[2] << " "
					    << position[0] << " " << position[1] << " " << position[2] << std::endl;

			// Get and write innovation data
			float vel_pos_innov[6];
			_ekf.get_vel_pos_innov(vel_pos_innov);
			vel_pos_innov_log << ekf_time_sec << " " << vel_pos_innov[0] << " " << vel_pos_innov[1] << " "
					  << vel_pos_innov[2] << " " << vel_pos_innov[3] << " " << vel_pos_innov[4] << " "
					  << vel_pos_innov[5] << std::endl;

			float vel_pos_innov_var[6];
			_ekf.get_vel_pos_innov_var(vel_pos_innov_var);
			vel_pos_innov_var_log << ekf_time_sec << " " << vel_pos_innov_var[0] << " " << vel_pos_innov_var[1] << " "
						<< vel_pos_innov_var[2] << " " << vel_pos_innov_var[3] << " " << vel_pos_innov_var[4] << " "
						<< vel_pos_innov_var[5] << std::endl;

			float mag_innov[3];
			_ekf.get_mag_innov(mag_innov);
			mag_innov_log << ekf_time_sec << " " << mag_innov[0] << " " << mag_innov[1] << " "
				      << mag_innov[2] << std::endl;

			float mag_innov_var[3];
			_ekf.get_mag_innov_var(mag_innov_var);
			mag_innov_var_log << ekf_time_sec << " " << mag_innov_var[0] << " " << mag_innov_var[1] << " "
					  << mag_innov_var[2] << std::endl;

			float gyro_rad[3];

			{
				// generate control state data
				float gyro_bias[3] = {};
				_ekf.get_gyro_bias(gyro_bias);
				gyro_rad[0] = gyro_buffer.data[0] - gyro_bias[0];  // 使用配对的陀螺仪数据
				gyro_rad[1] = gyro_buffer.data[1] - gyro_bias[1];
				gyro_rad[2] = gyro_buffer.data[2] - gyro_bias[2];

				// Velocity in body frame
				Vector3f v_n(velocity);
				matrix::Dcm<float> R_to_body(q.inversed());
				Vector3f v_b = R_to_body * v_n;

				// Local Position NED
				// float position[3];
				// _ekf.get_position(position);
				//printf("position: %lf,%lf,%lf\n", position[0], position[1], position[2]);
				position_estimator<< ekf_time_sec <<" "<<position[0] <<" "<<position[1] <<" "
				<<-position[2] <<" "<<std::endl;
				//position_estimator<< now/1.e6f <<" "<<position[0] <<" "<<position[1] - 0.278398 <<" "
				//<<-position[2] + 0.0849676 <<" "<<std::endl;
				// Attitude quaternion
				//q.copyTo(ctrl_state.q);

				// Acceleration data - 使用配对的加速度计数据
				matrix::Vector<float, 3> acceleration(last_processed_accel);

				float accel_bias[3];
				_ekf.get_accel_bias(accel_bias);
				// ctrl_state.x_acc = acceleration(0) - accel_bias[0];
				// ctrl_state.y_acc = acceleration(1) - accel_bias[1];
				// ctrl_state.z_acc = acceleration(2) - accel_bias[2];

				// compute lowpass filtered horizontal acceleration
				acceleration = R_to_body.transpose() * acceleration;
				// _acc_hor_filt = 0.95f * _acc_hor_filt + 0.05f * sqrtf(acceleration(0) * acceleration(0) +
				// 		acceleration(1) * acceleration(1));
				// ctrl_state.horz_acc_mag = _acc_hor_filt;

				// ctrl_state.airspeed_valid = false;

			}
			
			// generate vehicle local position data
			float pos[3] = {};
			// Position of body origin in local NED frame
			_ekf.get_position(pos);
			//printf("%f  %f  %f\n", pos[0],pos[1],pos[2]);

			// Velocity of body origin in local NED frame (m/s)

			// TODO: better status reporting
	

			// Position of local NED origin in GPS / WGS84 frame
			
			// true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon)
			//_ekf.get_ekf_origin(&lpos.ref_timestamp, &ekf_origin, &lpos.ref_alt);
		
			// The rotation of the tangent plane vs. geographical north
			matrix::Eulerf euler(q);
			euler_estimator<< ekf_time_sec <<" "<<euler.phi() <<" "<<euler.theta() <<" "
			<<euler.psi() <<" "<<std::endl;			
			
			// TODO: uORB definition does not define what these variables are. We have assumed them to be horizontal and vertical 1-std dev accuracy in metres
			Vector3f pos_var, vel_var;
			_ekf.get_pos_var(pos_var);
			_ekf.get_vel_var(vel_var);
			//printf("pos_var: %lf,%lf,%lf\n", pos_var(0), pos_var(1), pos_var(2) );
			//printf("vel_var: %lf,%lf,%lf\n", vel_var(0), vel_var(1), vel_var(2) );

		} 

	}
	printf("end\n");


}

int main(int argc, char *argv[])
{
	printf("begin\n");

	// 默认的数据目录
	std::string data_dir = "../../data";

	// 解析命令行参数，只寻找 --input_dir
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if ((arg == "--input_dir") && i + 1 < argc) {
			data_dir = argv[++i];
		} else {
			std::cerr << "Usage: " << argv[0] << " --input_dir <path>" << std::endl;
			return 1;
		}
	}

	std::cout << "Using data directory for input and output: " << data_dir << std::endl;
	
	Ekf2* _ekf2 = new Ekf2();
	// 将统一的路径传递给 task_main
	_ekf2->task_main(data_dir);


	delete _ekf2; // 释放内存
	return 0; // 成功时返回0
}
