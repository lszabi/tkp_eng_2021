#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#define FOCUS_SENSORS_NUM 5
#define TRACK_SENSORS_NUM 19
#define OPPONENTS_SENSORS_NUM 36

typedef enum{WARMUP,QUALIFYING,RACE,UNKNOWN,INIT} tstage;

typedef struct {
        float angle;
        float curLapTime;
        float damage;
        float distFromStart;
        float distRaced;
        float focus[FOCUS_SENSORS_NUM];
        float fuel;
        int   gear;
        float lastLapTime;
        float opponents[OPPONENTS_SENSORS_NUM];
        int   racePos;
        int   rpm;
        float speedX;
        float speedY;
        float speedZ;
        float track[TRACK_SENSORS_NUM];
        float trackPos;
        float wheelSpinVel[4];
        float z;
        tstage stage;
        tstage prevStage;
} structCarState;

typedef struct {
    
        // Accelerate command [0,1]
        float accel;

        // Brake command [
        float brake;

        // Gear command
        int gear;
        
        // Steering command [-1,1]
        float steer;
        
        // Clutch command [0,1]
        float clutch;

        // meta-command
        int meta;

  	// focus command [-90,90], i.e. angle of track sensor focus desired by client
	int focus;

} structCarControl;

#endif /* TYPEDEFS_H_ */