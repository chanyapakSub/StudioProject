/*
 * kalman.c
 *
 *  Created on: May 21, 2024
 *      Author: naker
 */

#include "kalman.h"

void kalman_init(Kalman* kalman){

	// Adjust R&Q here
	kalman -> Q = 1.0;
	kalman -> R = 1.0;

	// Initialize matrix
	float32_t A[9] = {	1.0f, 0.001f, 0.001f*0.001f*0.5f,
						0.0f, 1.0f, 0.001f,
						0.0f, 0.0f, 1.0f };

	float32_t B[3] = {	1.0f,
						0.0f,
						0.0f };

	float32_t C[3] = {	1.0f, 0.0f, 0.0f };

	float32_t G[3] = {	0.001f*0.001f*0.5f,
						0.001f,
						1.0f };

	float32_t w[1] = { 0.000001f };

	float32_t one[1] = { 0.0f };

	float32_t oxt[3] = { 0.0f , 0.0f, 0.0f};

	float32_t txo[3] = {	0.0f,
							0.0f,
							0.0f };

	float32_t txt[9] = { 0.0f, 0.0f, 0.0f,
						 0.0f, 0.0f, 0.0f,
						 0.0f, 0.0f, 0.0f };

	float32_t i[9] = {	1.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f,
						0.0f, 0.0f, 1.0f };


	arm_mat_init_f32(&kalman -> I, 3, 3, i);

	// Process model initialize
	arm_mat_init_f32(&kalman -> A, 3, 3, A);
	arm_mat_init_f32(&kalman -> B, 3, 1, B);
	arm_mat_init_f32(&kalman -> C, 1, 3, C);
	arm_mat_init_f32(&kalman -> G, 3, 1, G);

	// Transpose initialize
	arm_mat_init_f32(&kalman -> A_T, 3, 3, A);
	arm_mat_trans_f32(&kalman -> A, &kalman -> A_T);
	arm_mat_init_f32(&kalman -> C_T , 3, 1, txo);
	arm_mat_trans_f32(&kalman -> C, &kalman -> C_T);
	arm_mat_init_f32(&kalman -> G_T , 1, 3, oxt);
	arm_mat_trans_f32(&kalman -> G, &kalman -> G_T);

	// Predict equation
	// x_p_k = Ax_k_1 + Bu_k_1 + Gw
	arm_mat_init_f32(&kalman -> x_k, 3, 1, txo);
	arm_mat_init_f32(&kalman -> x_p_k, 3, 1, txo);
	arm_mat_init_f32(&kalman -> x_k_1, 3, 1, txo);
	arm_mat_init_f32(&kalman -> Ax_k_1, 3, 1, txo);
	arm_mat_init_f32(&kalman -> u_k_1, 1, 1, one);
	arm_mat_init_f32(&kalman -> Bu_k_1, 3, 1, txo);
	arm_mat_init_f32(&kalman -> w, 1, 1, w);
	arm_mat_init_f32(&kalman -> Gw, 3, 1, txo);
	// P_k = AP_k_1A_T + GQG_T
	arm_mat_init_f32(&kalman -> P_k, 3, 3, txt);
	arm_mat_init_f32(&kalman -> P_p_k, 3, 3, txt);
	arm_mat_init_f32(&kalman -> P_k_1, 3, 3, txt);
	arm_mat_init_f32(&kalman -> AP_k_1A_T, 3, 3, txt); // Apply identity at p_k_1
	arm_mat_init_f32(&kalman -> GG_T, 3, 3, txt);
	arm_mat_init_f32(&kalman -> GQG_T, 3, 3, txt);
//	arm_mat_mult_f32(&kalman -> A, &kalman -> x_k_1, &kalman -> Ax_k_1);

	// Update equation
	// y_p = y - Cx_p_k
	arm_mat_init_f32(&kalman -> y_p, 1, 1, one);
	arm_mat_init_f32(&kalman -> y, 1, 1, one);
	arm_mat_init_f32(&kalman -> Cx_p_k, 1, 1, one);
//	// S = CP_kC_T + R
	arm_mat_init_f32(&kalman -> S, 1, 1, one);
	arm_mat_init_f32(&kalman -> S_inv, 1, 1, one);
	arm_mat_init_f32(&kalman -> CP_kC_T, 1, 1, one); // Apply identity at p_k_1
//	// K = P_kC_TS_inv
	arm_mat_init_f32(&kalman -> K, 3, 1, txt);
	arm_mat_init_f32(&kalman -> KC, 3, 3, txt);
}

//void compute_kalman(Kalman* kalman, float32_t input, float32_t measure){
//
//	arm_matrix_instance_f32 Q;
//	arm_matrix_instance_f32 R;
//	arm_mat_init_f32(&Q, 1, 1, &kalman -> Q);
//	arm_mat_init_f32(&R, 1, 1, &kalman -> R);
//	arm_mat_init_f32(&kalman -> u_k_1, 1, 1, &input);
//
//	// Predict
//	// x_p_k = Ax_k_1 + Bu_k_1 + Gw
//	// Ax_k_1
//	arm_mat_mult_f32(&kalman -> A, &kalman -> x_k_1, &kalman -> Ax_k_1);
//	// Bu_k_1
//	arm_mat_mult_f32(&kalman -> B, &kalman -> u_k_1, &kalman -> Bu_k_1);
//	// Gw
//	arm_mat_mult_f32(&kalman -> G, &kalman -> w, &kalman -> Gw);
//	// x_p_k
//	arm_mat_add_f32(&kalman -> Ax_k_1, &kalman -> Bu_k_1, &kalman -> x_p_k);
//	arm_mat_add_f32(&kalman -> x_p_k, &kalman -> Gw, &kalman -> x_p_k);
//	// P_p_k = AP_k_1A_T + GQG_T
//	// AP_k_1A_T
//	arm_mat_mult_f32(&kalman -> A, &kalman -> P_k_1, &kalman -> AP_k_1A_T);
//	arm_mat_mult_f32(&kalman -> AP_k_1A_T, &kalman -> A_T, &kalman -> AP_k_1A_T);
//	// GQG_T
//	arm_mat_mult_f32(&kalman -> G, &kalman -> G_T, &kalman -> GG_T);
//	arm_mat_mult_f32(&kalman -> GG_T, &Q, &kalman -> GQG_T);
//	// P_p_k
//	arm_mat_add_f32(&kalman -> AP_k_1A_T, &kalman -> GQG_T, &kalman -> P_p_k);
//
//	// Update
//	// Y_p = y - Cx_p_k
//	// Cx_p_k
//	arm_mat_mult_f32(&kalman -> C, &kalman -> x_p_k, &kalman -> Cx_p_k);
//	// y - Cx_p_k
//	arm_mat_init_f32(&kalman -> y, 1, 1, &measure);
//	arm_mat_sub_f32(&kalman -> y, &kalman -> Cx_p_k, &kalman -> y_p);
//	//CP_kC_T
//	arm_mat_mult_f32(&kalman -> C, &kalman -> P_k, &kalman -> CP_kC_T);
//	arm_mat_mult_f32(&kalman -> CP_kC_T, &kalman -> C_T, &kalman -> CP_kC_T);
//	// S = CP_k_C_T - R
//	arm_mat_sub_f32(&kalman -> CP_kC_T, &R, &kalman -> S);
//	// K = P_kC_TS_inv
//	arm_mat_mult_f32(&kalman -> P_k, &kalman -> C_T, &kalman -> K);
//	arm_mat_inverse_f32(&kalman -> S, &kalman -> S_inv);
//	arm_mat_mult_f32(&kalman -> K, &kalman -> S_inv, &kalman -> K);
//	// x_k = x_p_k + Ky_p
//	arm_mat_mult_f32(&kalman -> K, &kalman -> y_p, &kalman -> Ky_p);
//	arm_mat_add_f32(&kalman -> x_p_k, &kalman -> Ky_p, &kalman -> x_k);
//	// P_k = (I - KC)P_p_k
//	arm_mat_mult_f32(&kalman -> K, &kalman -> C, &kalman -> KC);
//	arm_mat_sub_f32(&kalman -> I, &kalman -> KC, &kalman -> P_k);
//	arm_mat_mult_f32(&kalman -> P_k, &kalman -> P_p_k, &kalman -> P_k);
//}
void compute_kalman(Kalman* kalman, float32_t input, float32_t measure) {
    arm_matrix_instance_f32 Q, R, temp1, temp2;
    arm_mat_init_f32(&Q, 1, 1, &kalman->Q);
    arm_mat_init_f32(&R, 1, 1, &kalman->R);
    arm_mat_init_f32(&kalman->u_k_1, 1, 1, &input);

    // Predict
    // x_p_k = A * x_k_1 + B * u_k_1 + G * w
    arm_mat_mult_f32(&kalman->A, &kalman->x_k_1, &kalman->Ax_k_1);
    arm_mat_mult_f32(&kalman->B, &kalman->u_k_1, &kalman->Bu_k_1);
    arm_mat_mult_f32(&kalman->G, &kalman->w, &kalman->Gw);
    arm_mat_add_f32(&kalman->Ax_k_1, &kalman->Bu_k_1, &kalman->x_p_k);
    arm_mat_add_f32(&kalman->x_p_k, &kalman->Gw, &kalman->x_p_k);

    // P_p_k = A * P_k_1 * A_T + G * Q * G_T
    arm_mat_mult_f32(&kalman->A, &kalman->P_k_1, &temp1);
    arm_mat_mult_f32(&temp1, &kalman->A_T, &kalman->AP_k_1A_T);
    arm_mat_mult_f32(&kalman->G, &Q, &temp1);
    arm_mat_mult_f32(&temp1, &kalman->G_T, &kalman->GQG_T);
    arm_mat_add_f32(&kalman->AP_k_1A_T, &kalman->GQG_T, &kalman->P_p_k);

    // Update
    // y = measure - C * x_p_k
    arm_mat_init_f32(&kalman->y, 1, 1, &measure);
    arm_mat_mult_f32(&kalman->C, &kalman->x_p_k, &kalman->Cx_p_k);
    arm_mat_sub_f32(&kalman->y, &kalman->Cx_p_k, &kalman->y_p);

    // S = C * P_p_k * C_T + R
    arm_mat_mult_f32(&kalman->C, &kalman->P_p_k, &temp1);
    arm_mat_mult_f32(&temp1, &kalman->C_T, &kalman->CP_kC_T);
    arm_mat_add_f32(&kalman->CP_kC_T, &R, &kalman->S);

    // K = P_p_k * C_T * inv(S)
    arm_mat_mult_f32(&kalman->P_p_k, &kalman->C_T, &kalman->K);
    arm_mat_inverse_f32(&kalman->S, &kalman->S_inv);
    arm_mat_mult_f32(&kalman->K, &kalman->S_inv, &kalman->K);

    // x_k = x_p_k + K * y_p
    arm_mat_mult_f32(&kalman->K, &kalman->y_p, &kalman->Ky_p);
    arm_mat_add_f32(&kalman->x_p_k, &kalman->Ky_p, &kalman->x_k);

    // P_k = (I - K * C) * P_p_k
    arm_mat_mult_f32(&kalman->K, &kalman->C, &kalman->KC);
    arm_mat_sub_f32(&kalman->I, &kalman->KC, &temp1);
    arm_mat_mult_f32(&temp1, &kalman->P_p_k, &kalman->P_k);
}

