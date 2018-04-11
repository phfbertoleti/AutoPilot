/*
Header do modulo de controle PID
Data: Abril/2018
Autor: Pedro Bertoleti
*/

#ifndef CONTROLEPID_H_
#define CONTROLEPID_H_

//prototypes
void configura_setpoint(float valor_setpoint);
float le_setpoint(void);
float computa_PID(float variavel_processo, float step_tempo);

#endif
