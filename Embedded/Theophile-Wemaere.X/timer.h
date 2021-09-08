#ifndef TIMER_H
#define TIMER_H

void InitTimer23(void) ;
void InitTimer1(void) ;
void SetFreqTimer1(float freq);
void InitTimer4 (void);
void SetFreqTimer4(float freq);
extern unsigned long timestamp;

#define FREQ_ECH_QEI 250

#endif /* TIMER_H */
