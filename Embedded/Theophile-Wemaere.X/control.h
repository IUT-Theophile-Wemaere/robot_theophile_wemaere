#ifndef CONTROL_H
#define	CONTROL_H

void SetRobotState(unsigned char RobotState);
void SetRobotAutoControlState(unsigned char control);
void OperatingSystemLoop(void);
void SetNextRobotStateInAutomaticMode(void);

#endif /*CONTROL_H*/