/*
Header do modulo de obtenção de velocidade mediante leituras do acelerometro
Data: Abril/2018
Autor: Pedro Bertoleti
*/

#ifndef VELACC_H_
#define VELACC_H_

//defines
#define M_POR_S     0
#define KM_POR_HORA 1

//typedefs
typedef enum {
	eStatusExecOK = 0,
	eStatusExecFailRootPrivillege = -1,
	eStatusExecFailInitIMU = -2,
}TStatusExec;

//prototypes
TStatusExec init_velocidade_por_acc(void);
void finaliza_velocidade_por_acc(void);
float get_vel_atual(char tipo_vel);

#endif
