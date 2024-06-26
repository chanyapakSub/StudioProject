#ifndef INC_BASESYSTEM_H_
#define INC_BASESYSTEM_H_

#include "main.h"
#include "ModBusRTU.h"
#include "eff.h"

extern ModbusHandleTypedef hmodbus;
extern u16u8_t registerFrame[200];
extern char Vacuum[10];
extern char Gripper[20];
extern char Shelves[20];
extern char Order[10];
extern char Jogmode[20];
extern char Home[20];
extern uint16_t Pick[5];
extern uint16_t Place[5];
extern uint16_t Jogging;
extern uint16_t Jogginghome;
extern uint16_t countPick;
extern uint16_t countPlace;
extern uint16_t state;
extern uint8_t set_shelves_state;

void Vacuum_Status(EFF* eff);
void Gripper_Movement_Status(EFF* eff);
uint16_t Set_Shelves();
float Set_Goal_Point();
uint16_t Run_Point_Mode();
void Set_Home();
void SetPick_PlaceOrder();
uint16_t Run_Jog_Mode();

#endif /* INC_BASESYSTEM_H_ */
