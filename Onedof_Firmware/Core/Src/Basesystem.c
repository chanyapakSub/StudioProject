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

ModbusHandleTypedef hmodbus;
//1.Heart Beat
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    if (htim == &htim3 )
//    {
//       registerFrame[0x00].U16 = 22881;
//    }
//}


void Vacuum_Status(){
    //Vacuum On
    if (registerFrame[0x02].U16 == 1){ // ใช้ == แทน =
        strcpy(Vacuum, "On");
    }
    //Vacuum Off
    else if (registerFrame[0x02].U16 == 0){ // ใช้ == แทน =
        strcpy(Vacuum, "Off");
    }
}

void Gripper_Movement_Status(){
    //Movement Forward
    if (registerFrame[0x03].U16 == 1){ // ใช้ == แทน =
        strcpy(Gripper, "Forward");
    }
    //Movement Backward
    else if (registerFrame[0x03].U16 == 0){ // ใช้ == แทน =
        strcpy(Gripper, "Backward");
    }
}

void Set_Shelves(){
    //Set
    if (registerFrame[0x01].U16 == 1){ // ใช้ == แทน =
        strcpy(Shelves, "SET");
        registerFrame[0x01].U16 = 0;
        registerFrame[0x10].U16 = 1;

        if(Jogging == 1){
            registerFrame[0x10].U16 = 0;
        }
    }

    registerFrame[0x23].U16 = 8;  //1st Shelve Position
    registerFrame[0x24].U16 = 8;  //2nd Shelve Position
    registerFrame[0x25].U16 = 8;  //3rd Shelve Position
    registerFrame[0x26].U16 = 8;  //4th Shelve Position
    registerFrame[0x27].U16 = 8;  //5th Shelve Position
}


void Set_Goal_Point(){
	if (registerFrame[0x30].U16 != 0){
		Run_Point_Mode();
	 }
}

void Run_Point_Mode(){

	if (registerFrame[0x01].U16 = 8){
		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = 1 ;
	}
}

void Set_Home(){
	registerFrame[0x01].U16 = 2;
	registerFrame[0x01].U16 = 0;
	registerFrame[0x01].U16 = 2;
	strcpy(Home, "Homing...");
	if (Jogginghome == 1){
		registerFrame[0x01].U16 = 0;
	}


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



void Run_Jog_Mode() {
	if (registerFrame[0x01].U16 = 4) {
		strcpy(Jogmode, "Run Jog Mode");
		registerFrame[0x01].U16 = 0;
		for (int i = 0; i < 5; i++) {
			strcpy(Jogmode, "Go to Pick...");
			registerFrame[0x10].U16 = 4;
			SetPick_PlaceOrder(); //แก้ให้เข้ากับซัน

			strcpy(Jogmode, "Go to Place...");
			registerFrame[0x10].U16 = 8;
			SetPick_PlaceOrder(); //แก้ให้เข้ากับซัน
		}
	}
	registerFrame[0x10].U16 = 0;
}

#endif /* SRC_BASESYSTEM_C_ */
