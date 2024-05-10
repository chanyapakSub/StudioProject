/*
 * Basesystem.c
 *
 *  Created on: Apr 30, 2024
 *      Author: chanyapak
 */

#ifndef SRC_BASESYSTEM_C_
#define SRC_BASESYSTEM_C_
#include "Basesystem.h"

char Vacuum[10] = "Off";
char Gripper[20] = "None";
char Shelves[20] = "None";
char Order[10] = "None";
char Jogmode[20] = "None";
char Home[20] = "None";
uint16_t Pick[5];
uint16_t Place[5];
uint16_t Jogging = 0;
uint16_t Jogginghome = 0;
uint16_t countPick = 0;
uint16_t countPlace = 0;
uint16_t state = 0;
uint8_t set_shelves_state = 0;
ModbusHandleTypedef hmodbus;

void Vacuum_Status(EFF* eff){
    //Vacuum On
    if (registerFrame[0x02].U16 == 1){
        strcpy(Vacuum, "On");
        eff -> solenoid_command[0] = 1;
    }
    //Vacuum Off
    else if (registerFrame[0x02].U16 == 0){
        strcpy(Vacuum, "Off");
        eff -> solenoid_command[0] = 0;
    }
}

void Gripper_Movement_Status(EFF* eff){
    //Movement Forward (push)
    if (registerFrame[0x03].U16 == 1){
        strcpy(Gripper, "Forward");
        // if pull reed switch is activate
        if(eff -> actual_status[0] == 1 && eff -> actual_status[1] == 0){
			eff -> solenoid_command[1] = 1;
			eff -> solenoid_command[2] = 0;
        }
        // if push reed switch is activate
        else if(eff -> actual_status[0] == 0 && eff -> actual_status[1] == 1){
        	eff -> solenoid_command[1] = 0;
			eff -> solenoid_command[2] = 0;
        }
    }
    //Movement Backward (pull)
    else if (registerFrame[0x03].U16 == 0){
        strcpy(Gripper, "Backward");
        // if pull reed switch is activate
        if(eff -> actual_status[0] == 1 && eff -> actual_status[1] == 0){
			eff -> solenoid_command[1] = 0;
			eff -> solenoid_command[2] = 0;
        }
        // if push reed switch is activate
        else if(eff -> actual_status[0] == 0 && eff -> actual_status[1] == 1){
        	eff -> solenoid_command[1] = 0;
			eff -> solenoid_command[2] = 1;
        }
    }
}

uint16_t Set_Shelves(){
    //Set shelve
    if (registerFrame[0x01].U16 == 1){
    	state = 1;
        strcpy(Shelves, "SET");
        registerFrame[0x01].U16 = 0;
        registerFrame[0x10].U16 = 1;
        set_shelves_state = 1;
        return 1;
    }
    else{return 0;}
}

// wait for Data type check
uint16_t Set_Goal_Point(){
	return registerFrame[0x30].U16;
}

uint16_t Run_Point_Mode(){
	if (registerFrame[0x01].U16 == 8){
		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = 16;
		state = registerFrame[0x10].U16;
		return 1;
	}else{return 0;}
}

void Set_Home(){
	if(registerFrame[0x10].U16 == 2){
		state = 2;
		strcpy(Home, "Homing...");
	}
	else{
		return;
	}
//	if (Jogginghome == 1){
//		registerFrame[0x01].U16 = 0;
//	}


}
void SetPick_PlaceOrder() {
    if (registerFrame[0x21].U16 != 00000 && registerFrame[0x22].U16 != 00000 ) {
    	strcpy(Order, "Pick");
    	Pick[0] = registerFrame[0x21].U16/10000;
    	countPick += Pick[0]*10000;
    	strcpy(Order, "Place");
    	Place[0] = registerFrame[0x22].U16/10000;
    	countPlace +=Place[0]*10000;


    	strcpy(Order, "Pick");
    	Pick[1] = (registerFrame[0x21].U16- countPick)/1000;
    	countPick += Pick[1]*1000;
    	strcpy(Order, "Place");
    	Place[1] = (registerFrame[0x22].U16- countPlace)/1000;
    	countPlace +=Place[1]*1000;


    	strcpy(Order, "Pick");
		Pick[2] = (registerFrame[0x21].U16 - countPick) / 100;
		countPick += Pick[2] * 100;
		strcpy(Order, "Place");
		Place[2] = (registerFrame[0x22].U16 - countPlace) / 100;
		countPlace += Place[2] * 100;


		strcpy(Order, "Pick");
		Pick[3] = (registerFrame[0x21].U16 - countPick) / 10;
		countPick += Pick[3] * 10;
		strcpy(Order, "Place");
		Place[3] = (registerFrame[0x22].U16 - countPlace) / 10;
		countPlace += Place[3] * 10;


		strcpy(Order, "Pick");
		Pick[4] = (registerFrame[0x21].U16 - countPick);
		countPick = 0;
		strcpy(Order, "Place");
		Place[4] = (registerFrame[0x22].U16 - countPlace);
		countPlace = 0;

    }
}



uint16_t Run_Jog_Mode() {
	if (registerFrame[0x01].U16 == 1) {
		strcpy(Jogmode, "Run Jog Mode");
		registerFrame[0x01].U16 = 0;
		return 1;
//		for (int i = 0; i < 5; i++) {
//			state = 4;
//			strcpy(Jogmode, "Go to Pick...");
//			registerFrame[0x10].U16 = 4;
//			SetPick_PlaceOrder(); //แก้ให้เข้ากับซัน
//
//			state = 8;
//			strcpy(Jogmode, "Go to Place...");
//			registerFrame[0x10].U16 = 8;
//			SetPick_PlaceOrder(); //แก้ให้เข้ากับซัน
//		}
	}
	else{return 0;}
//	registerFrame[0x10].U16 = 0;
}

#endif /* SRC_BASESYSTEM_C_ */
