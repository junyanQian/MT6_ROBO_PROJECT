#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

#define SPEED_SEARCH		150
#define SPEED_FETCH			80
#define SPEED_ROTATION		50
#define STEP_FETCH			200
#define LIMIT_TOF			60

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
