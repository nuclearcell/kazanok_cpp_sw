//#include <TinyGPS++.h>
#pragma once
#pragma region ConversationDefs
#define SYNC_PACKET 0x53 //(S) Sync
#define DATA_PACKET 0x44 //(D) Data
#define COMMAND_PACKET 0x43 //(C) Command
#define IMG_PACKET 0x49 //(I) Image
#define F_IMG_PACKET 0x46 //(F) Final Image packet

//#define IMG_BUFFER_SIZE 4096

const char* SECRET_CODE = "\x1F\xF5\xFF";//???
#define IMG_PACKET_MAX_SIZE 250



#pragma endregion

#pragma region ConversationDataStructs

//https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.h#L461
struct mpu6050_data
{
    /* data */
};

//https://github.com/LowPowerLab/SFE_BMP180/blob/master/SFE_BMP180.h
//
struct bmp180_data
{
    /* data */
};


struct GPS_data
{
    /* data */
};




struct data_packet
{
    
};




#pragma endregion

#pragma region IntegrationMethods
//https://medium.com/@szewczyk.franciszek02/rope-simulator-in-c-a595a3ef956c
void Verlet(float& x0, float& x, float a, float dt) {              
        float xCopy = x;                        
        x = x + x - x0 + a * dt * dt;           
        x0 = xCopy;                             
    }
//https://en.wikipedia.org/wiki/Verlet_integration

#pragma endregion

//https://perso.liris.cnrs.fr/nicolas.pronost/UUCourses/GamePhysics/lectures/lecture%205%20Numerical%20Integration.pdf