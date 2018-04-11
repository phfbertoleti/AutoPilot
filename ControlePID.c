/*
Modulo: controle PID
Autor: Pedro Bertoleti
Data: Abril/2018
*/

//includes
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "ControlePID.h"

//defines
#define CONSTANTE_KP    1.0  //obtido empiricamente
#define CONSTANTE_KI    0.5  //obtido empiricamente
#define CONSTANTE_KD    0.0  //obtido empiricamente
#define LIMITE_MAX_PID  1.0  //100%
#define LIMITE_MIN_PID  0.0  //0%

//variaveis globais
static float setpoint_pid = 0.0;
static float ultimo_erro = 0.0;
static float acum_integral = 0.0;

//Funcao: configura um setpoint
//Parametros: setpoint desejado
//Retorno: nenhum
void configura_setpoint(float valor_setpoint)
{
   //na configuração de um novo setpoint, faz sentido inicializar as variaveis de controle PID
   setpoint_pid = 0.0;
   ultimo_erro = 0.0;
   acum_integral = 0.0;
   setpoint_pid = valor_setpoint;
}

//Funcao: le o setpoint configurado
//Parametros: nenhum
//Retorno: setpoint configurado
float le_setpoint(void)
{
   return setpoint_pid;
}

//Funcao: computa o controlador PID
//Parametros: variavel de processo e step de tempo entre medidas (em segundos)
//Returno: valor do PID computado
float computa_PID(float variavel_processo, float step_tempo)
{
    float erro_atual;
    float fator_p;
    float fator_i;
    float fator_d;
    float res_pid;

    //calcula erro
    erro_atual = setpoint_pid - variavel_processo;

    //faz os calculos dos fatores P, I e D
    fator_p = CONSTANTE_KP * erro_atual;

    fator_i = acum_integral + (CONSTANTE_KI * erro_atual * step_tempo);
    acum_integral = fator_i;

    fator_d = CONSTANTE_KD * ( (ultimo_erro - erro_atual) / step_tempo );
    ultimo_erro = erro_atual;

    //calcula valor final do PID
    res_pid = fator_p + fator_i + fator_d;

    //garante que o valor computado está dentro dos limites
    if (res_pid > LIMITE_MAX_PID)
      res_pid = LIMITE_MAX_PID;

    if (res_pid < LIMITE_MIN_PID)
      res_pid = LIMITE_MIN_PID;

    //rtetorna o valor computado
    return res_pid;
}
