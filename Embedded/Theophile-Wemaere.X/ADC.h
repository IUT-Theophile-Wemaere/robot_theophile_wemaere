#ifndef ADC_H
#define	ADC_H

void InitADC1(void);
void ADC1StartConversionSequence(void);
void ADCClearConversionFinishedFlag(void);
unsigned char ADCIsConversionFinished(void);
unsigned int * ADCGetResult(void);

#endif /*ADC_H*/