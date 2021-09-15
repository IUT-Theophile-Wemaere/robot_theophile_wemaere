#ifndef QEI_H
#define QEI_H

#define DISTROUES 0.218
#define WHEEL_DIAMETER 0.0426
#define POINT_TO_METER 0.000016336  //(WHEEL_DIAMETER * PI / 8192.0)
#define FREQ_ECH_QEI 250
#define POSITION_DATA 0x0061
#define ASSERVISSEMENT 0x0062

void InitQEI1(void);
void InitQEI2(void);
void QEIUpdateData(void);
void SendPositionData(void);

#endif /* QEI_H */
