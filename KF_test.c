/*
 * kf_test.c
 *
 *  Created on: 2017.03.14
 *      Author: yueyu
 */
#include <cr_section_macros.h>

#if defined (__MULTICORE_MASTER_SLAVE_M0SLAVE) || \
    defined (__MULTICORE_MASTER_SLAVE_M4SLAVE)
#include "boot_multicore_slave.h"
#endif

#include <stdio.h>

#include "misc.h"
#include "kalmanfilter.h"
#include "mat.h"

#include "kf_template_poly_line_ans.h"
#include "kf_template_poly_line.h"

#define TESTDELTA 1e-6

void kf_test(void) {

#if defined (__MULTICORE_MASTER_SLAVE_M0SLAVE) || \
    defined (__MULTICORE_MASTER_SLAVE_M4SLAVE)
    boot_multicore_slave();
#endif

int input_count = 0; // size count starts from 1 and ends at size of input_data;
        			 // input round count, ++ for every group of input
        			 // when count == 1, starts initialization.

        // input_data      <-- position raw data input
        //                        every line: (time_stamp), (position_x), (position_y),
        // input_data_size <-- size of input_data

        // kf_answer      <-- kalman filter answer for comparison
        //                        every line: (time_stamp), (position_x), (position_y),
        // kf_answer_size <-- size of answer

double input_x, input_y;
double time_stamp, prev_time_stamp;
double ans_time_stamp, ans_x, ans_y;

// -----------------------------------------------
// variables used for processing
double *meas_cov, *sys_cov, *state, *err_cov,
	   *transf_mat, *meas_mat, *input;

prev_time_stamp = 0;

//first input, start initialization.
input_count++;
// input re-formatting
time_stamp = input_data[(input_count - 1) * 3];
input_x    = input_data[(input_count - 1) * 3 + 1];
input_y    = input_data[(input_count - 1) * 3 + 2];

// answer template re-formatting
ans_time_stamp = kf_answer[(input_count - 1) * 3];
ans_x          = kf_answer[(input_count - 1) * 3 + 1];
ans_y          = kf_answer[(input_count - 1) * 3 + 2];

//initialize matrix
//meas_cov : R,2*2
meas_cov = zeros(2, 2);
//state vector : X,4*1,(x),(y),(vx),(vy)
state = zeros(4, 1);
//sys_cov : Q,4*4
sys_cov = zeros(4, 4);
//err_cov : P,4*4
err_cov = zeros(4, 4);
//meas_mat : H,2*4
meas_mat = zeros(2, 4);
//transf_mat : A,4*4
transf_mat = eye(4);
//input : (delta_t), (position_x), (position_y),
input = zeros(3, 1);

meas_cov[0] = 1e-1; // var_x
meas_cov[3] = 1e-1; // var_y

sys_cov[10] = 0.5e-5; // var_vx
sys_cov[15] = 0.5e-5; // var_vy

state[0] = input_x; // first value of x
state[1] = input_y; // first value of y

meas_mat[0] = 1;
meas_mat[5] = 1;

prev_time_stamp = time_stamp;

while (1) {
	input_count++;

	// if input_count is larger than the size of input_data, break
	if (input_count > input_data_size) {
		input_count = 0;
		break;
	}

	// input re-formatting
	time_stamp = input_data[(input_count - 1) * 3];
	input_x    = input_data[(input_count - 1) * 3 + 1];
	input_y    = input_data[(input_count - 1) * 3 + 2];

	// answer template re-formatting
	ans_time_stamp = kf_answer[(input_count - 1) * 3];
	ans_x          = kf_answer[(input_count - 1) * 3 + 1];
	ans_y          = kf_answer[(input_count - 1) * 3 + 2];

	//input[0] : delta_t
	input[0] = time_stamp - prev_time_stamp;
	prev_time_stamp = time_stamp;
    //input_x,input_y
	input[1] = input_x;
	input[2] = input_y;

	//update the transf_mat
	//x_predict = x_last + delta_t * vx
	transf_mat[2] = input[0];
	//y_predict = y_last + delta_t * vy
	transf_mat[7] = input[0];

	//Kalman Filter
	KalmanFilterV1(state, meas_cov, sys_cov, err_cov, transf_mat, meas_mat, input);

	printf("RN: %d\n X: %lf, Y: %lf, time: %lf\n", input_count, input_x,
			  input_y, time_stamp);
	printf("FTRD X: %lf, Y: %lf, VX: %lf, VY: %lf\n", state[0], state[1],
			  state[2], state[3]);
	printf("ANSW X: %lf, Y: %lf\n\n", ans_x, ans_y);
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TESTDELTA, ans_x, state[0], "[ERR] dot(): wrong position_x");
    TEST_ASSERT_DOUBLE_WITHIN_MESSAGE(TESTDELTA, ans_y, state[1], "[ERR] dot(): wrong position_y");
}
}
