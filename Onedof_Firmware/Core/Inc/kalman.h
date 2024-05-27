/*
 * kalman.h
 *
 *  Created on: May 21, 2024
 *      Author: naker
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"
#include "arm_math.h"

typedef struct{

	// R & Q constance
	float32_t R,Q;

	// Identity matrix
	arm_matrix_instance_f32 I;

	// Process model matrix
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 C;
	arm_matrix_instance_f32 D;
	arm_matrix_instance_f32 G;

	// Transpose matrix
	arm_matrix_instance_f32 A_T;
	arm_matrix_instance_f32 C_T;
	arm_matrix_instance_f32 G_T;

	// Computation matrix

	// Predict matrix
	arm_matrix_instance_f32 x_k;
	arm_matrix_instance_f32 x_p_k;
	arm_matrix_instance_f32 x_k_1;
	arm_matrix_instance_f32 Ax_k_1;
	arm_matrix_instance_f32 u_k_1;
	arm_matrix_instance_f32 Bu_k_1;
	arm_matrix_instance_f32 w;
	arm_matrix_instance_f32 Gw;
	arm_matrix_instance_f32 P_p_k;
	arm_matrix_instance_f32 P_k;
	arm_matrix_instance_f32 P_k_1;
	arm_matrix_instance_f32 AP_k_1A_T;
	arm_matrix_instance_f32 GG_T;
	arm_matrix_instance_f32 GQG_T;

	// Update matrix
	arm_matrix_instance_f32 y;
	arm_matrix_instance_f32 y_p;
	arm_matrix_instance_f32 Cx_p_k;
	arm_matrix_instance_f32 S;
	arm_matrix_instance_f32 S_inv;
	arm_matrix_instance_f32 CP_kC_T;
	arm_matrix_instance_f32 K;
	arm_matrix_instance_f32 Ky_p;
	arm_matrix_instance_f32 KC;

} Kalman;

void kalman_init(Kalman* kalman);
void compute_kalman(Kalman* kalman, float32_t input, float32_t measure);

#endif /* INC_KALMAN_H_ */
