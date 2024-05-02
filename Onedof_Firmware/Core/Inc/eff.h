/*
 * eff.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef INC_EFF_H_
#define INC_EFF_H_
typedef struct {
	uint8_t cylinder_status;
	uint8_t cylinder_position;
	uint8_t vacuum_status;
} EFF;

void eff_init(EFF* eff);
void Update_cylinder(EFF* eff);
void Update_vacuum(EFF* eff);

#endif /* INC_EFF_H_ */
