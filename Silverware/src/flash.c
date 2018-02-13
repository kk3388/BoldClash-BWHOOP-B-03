#include "project.h"
#include "drv_fmc.h"

extern int fmc_erase( void );
extern void fmc_unlock(void);
extern void fmc_lock(void);

extern float accelcal[];
extern float * pids_array[3];
extern float * pids_array2[3]; // dual PIDs code

extern float hardcoded_pid_identifier;


#define FMC_HEADER 0x12AA0001

float initial_pid_identifier = -10;
float initial_pid_identifier2 = -10; // dual PIDs code
float saved_pid_identifier;
float saved_pid_identifier2; // dual PIDs code


float flash_get_hard_coded_pid_identifier( void) {
	float result = 0;

	for (int i=0;  i<3 ; i++) {
		for (int j=0; j<3 ; j++) {
			result += pids_array[i][j] * (i+1) * (j+1) * 0.932f;
		}
	}
	return result;
}

// ----- DUAL PIDS CODE ---------
float flash_get_hard_coded_pid_identifier2( void) {
	float result = 0;

	for (int i=0;  i<3 ; i++) {
		for (int j=0; j<3 ; j++) {
			result += pids_array2[i][j] * (i+1) * (j+1) * 0.932f;
		}
	}
	return result;
}
// ------- END OF DUAL PIDS CODE ---------

void flash_hard_coded_pid_identifier( void)
{
 initial_pid_identifier = flash_get_hard_coded_pid_identifier();
}

// ----- DUAL PIDS CODE ---------
void flash_hard_coded_pid_identifier2( void)
{
 initial_pid_identifier2 = flash_get_hard_coded_pid_identifier2();
}
// ------- END OF DUAL PIDS CODE ---------



void flash_save( void) {

    fmc_unlock();
	fmc_erase();
	
	unsigned long addresscount = 0;

    writeword(addresscount++, FMC_HEADER);
   
	fmc_write_float(addresscount++, initial_pid_identifier );
	
	for (int i=0;  i<3 ; i++) {
		for (int j=0; j<3 ; j++) {
            fmc_write_float(addresscount++, pids_array[i][j]);
		}
	}

    fmc_write_float(addresscount++, accelcal[0]);
    fmc_write_float(addresscount++, accelcal[1]);
    fmc_write_float(addresscount++, accelcal[2]);

// ------- DUAL PIDS CODE ---------
 	fmc_write_float(addresscount++, initial_pid_identifier2 );
	
	for (int i=0;  i<3 ; i++) {
		for (int j=0; j<3 ; j++) {
            fmc_write_float(addresscount++, pids_array2[i][j]);
		}
	}
// ------- END OF DUAL PIDS CODE ------------
 
	
    writeword(255, FMC_HEADER);
    
	fmc_lock();
}



void flash_load( void) {

	unsigned long addresscount = 0;
// check if saved data is present
    if (FMC_HEADER == fmc_read(addresscount++)&& FMC_HEADER == fmc_read(255))
    {

     saved_pid_identifier = fmc_read_float(addresscount++);
// load pids from flash if pid.c values are still the same       
     if (  saved_pid_identifier == initial_pid_identifier )
     {
         for (int i=0;  i<3 ; i++) {
            for (int j=0; j<3 ; j++) {
                pids_array[i][j] = fmc_read_float(addresscount++);
            }
        }
     }
     else{
         addresscount+=9; 
     }


    accelcal[0] = fmc_read_float(addresscount++ );
    accelcal[1] = fmc_read_float(addresscount++ );
    accelcal[2] = fmc_read_float(addresscount++ );  

// ------- DUAL PIDS CODE ----------
	saved_pid_identifier2 = fmc_read_float(addresscount++);
     if (  saved_pid_identifier2 == initial_pid_identifier2 )
     {
         for (int i=0;  i<3 ; i++) {
            for (int j=0; j<3 ; j++) {
                pids_array2[i][j] = fmc_read_float(addresscount++);
            }
        }
     }
/*     else{
         addresscount+=9;  // not needed because there is no data after this, but in case something is added later, need to be used
     }    */
// ------- END OF DUAL PIDS CODE -------
	
	
	
    }
    else
    {
        
    }
    
}
