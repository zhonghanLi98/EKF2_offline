#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// POSIX头文件只在非Windows系统中包含
#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#endif
#include <errno.h>
#include <math.h>
#include <time.h>
#include <float.h>

#include <ekf.h>

class Ekf2
{
public:
	/**
	 * Constructor
	 */
	Ekf2();

	/**
	 * Destructor, also kills task.
	 */
	~Ekf2();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	//int	start();

	void task_main(const std::string& data_dir);

	static void task_main_trampoline(int argc, char *argv[]);

	void print_status();

	void exit() { _task_should_exit = true; }

private:
	static constexpr float _dt_max = 0.02;
	bool	_task_should_exit = false;
	int	_control_task = -1;		// task handle for task
	bool 	_replay_mode;			// should we use replay data from a log
	int 	_publish_replay_mode;		// defines if we should publish replay messages
	float	_default_ev_pos_noise = 0.05f;	// external vision position noise used when an invalid value is supplied
	float	_default_ev_ang_noise = 0.05f;	// external vision angle noise used when an invalid value is supplied

	// Initialise time stamps used to send sensor data to the EKF and for logging
	uint64_t _timestamp_mag_us = 0;
	uint64_t _timestamp_balt_us = 0;

	// Used to down sample magnetometer data
	matrix::Vector <float, 4> mag_from_API;
	float _mag_data_sum[3] = {0};			// summed magnetometer readings (Ga)
	uint64_t _mag_time_sum_ms = 0;		// summed magnetoemter time stamps (msec)
	// uint64_t _mag_time_sum_ms;		// summed magnetoemter time stamps (msec)
	uint8_t _mag_sample_count = 0;		// number of magnetometer measurements summed
	uint32_t _mag_time_ms_last_used = 0;	// time stamp in msec of the last averaged magnetometer measurement used by the EKF

	// Used to down sample barometer data
	matrix::Vector2f baro_from_API;
	float _balt_data_sum;			// summed barometric altitude readings (m)
	uint64_t _balt_time_sum_ms;		// summed barometric altitude time stamps (msec)
	uint8_t _balt_sample_count = 0;		// number of barometric altitude measurements summed
	uint32_t _balt_time_ms_last_used = 0;	// time stamp in msec of the last averaged barometric altitude measurement used by the EKF

	bool	_prev_landed = true;	// landed status from the previous frame

	float _acc_hor_filt = 0.0f; 	// low-pass filtered horizontal acceleration

	matrix::Vector <float, 4> gps_from_API;
	Ekf _ekf;

	parameters *_params;	// pointer to ekf parameter struct (located in _ekf class instance)

};
