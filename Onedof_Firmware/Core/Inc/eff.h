/*
 * eff.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef INC_EFF_H_
#define INC_EFF_H_
<<<<<<< Updated upstream
typedef struct {
	uint8_t cylinder_status;
	uint8_t cylinder_position;
	uint8_t vacuum_status;
} EFF;

void eff_init(EFF* eff);
void Update_cylinder(EFF* eff);
void Update_vacuum(EFF* eff);

=======
#include "main.h"

typedef struct{
	uint16_t gripper_actual_status; // Reed switch detection channel 0 is pull activate 1 is push activate
	uint8_t gripper_status; // Solenoid valve command if gripper_status == 0 pull , == 1 push
	uint8_t vacuum_status; // Solenoid valve command
} EFF;

void Update_endeffector_status(EFF* eff, GPIO_TypeDef* GPIO_Pull,uint16_t GPIO_Pin_Pull,/* Reed switch pull */
		GPIO_TypeDef* GPIO_Push,uint16_t GPIO_Pin_Push /* Reed switch push*/);
void Update_endeffector_command(EFF* eff, GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull,/* Solenoid valve pull */
		GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push, /* Solenoid valve push*/
		GPIO_TypeDef* GPIO_Vacuum, uint16_t GPIO_Pin_Vacuum /* Solenoid valve vacuum*/);
>>>>>>> Stashed changes
#endif /* INC_EFF_H_ */
