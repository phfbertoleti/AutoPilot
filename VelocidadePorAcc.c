/*
Modulo: obtenção de velocidade mediante leituras do acelerometro
Data: Abril/2018
Autor: Pedro Bertoleti

IMPORTANTE: 
1) Os offsets de aceleração nos eixos X e Y (calibração do acelerômetro) são calculados
   automaticamente na função de inicialização (init_velocidade_por_acc() ).
2) Este módulo foi baseado nos conceitos apresentados no aplication note AN3397 da 
   Freescale (atual NXP)
*/

//includes
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <string.h>
#include <limits.h>
#include "VelocidadePorAcc.h"

//defines
#define NUMERO_LEITURAS_PARA_FILTRAGEM        100  //OBS: para filtro de mediana, este numero deve ser impar e maior que 3
#define NUMERO_LEITURAS_CALIBRACAO_ACC        1000 //numero de leituras de acelerometro (para cada eixo, no caso x e y) para determinar offset 

//defines do IMU
#define FREQ_AMOSTRAGEM_ACC_HZ                200            //frequencia de amostragem do acelerômetro. Max: 200Hz
#define SENSIBILIDADE_ACC                     A_FSR_2G       //sensibilidade do acelerômetro (maior sensibilidade em A_FSR_2G e menor
                                                             //sensibilidade em A_FSR_16G  
#define NIVEL_FILTRO_DLFP                     ACCEL_DLPF_41  //frequência de corte de filtro passa-baixa do acelerômetro. Este filtro é nativo
                                                             //do acelerômetro. Seu valor deve ser escolhido empiricamente.

//defines de filtragem / adequacao das amostras do acelerometro
#define JANELA_ZERO_ACC                       0.3  //Limite (entre -JANELA_ZERO_ACC e +JANELA_ZERO_ACC) para considerar aceleração
                                                   //zero (eixos x e y) 
#define LIMIAR_MEDICOES_ACC_VEL_NULA          25   //número de medições de aceleração em zero (eixo x e/ou y) para zerar velocidade
#define NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL    20   //número de amostras do filtro de média móvel

//descomente a linha abaixo para habilitar o debug
#define HABILITA_DEBUG_V

#ifdef HABILITA_DEBUG_V
   #define DEBUG_VEL(...) printf(__VA_ARGS__)
#endif

//typedefs

//prototypes locais
void set_vel_atual(float vel);
float calcula_delta_velocidade(float accT0, float accT1, float freq_amostragem);
void chegou_leitura_imu(void);
void calcula_vel_atual(float accX_t0, float accY_t0, float accX_t1, float accY_t1);
float verifica_janela_acc(float amostra);
void atualiza_contador_acc_nula(float acc_t0, float acc_t1, unsigned int * ptrContador);

//variáveis globais
static rc_imu_data_t data; //dados de leituras do IMU
static float vel_atual = 0.0;
static float vel_x_atual = 0.0;
static float vel_y_atual = 0.0;
static float accX_t0 = 0.0;
static float accY_t0 = 0.0;
static float accX_t1 = 0.0;
static float accY_t1 = 0.0;
static float offset_acc_x=0.0;    //offset a ser aplicado na medicao de aceleracao do eixo x (calibracao de acelerometro)
static float offset_acc_y=0.0;    //offset a ser aplicado na medicao de aceleracao do eixo y (calibracao de acelerometro)
static int contador_interrupcoes_imu;
static int contador_preenchimento_buffer_filtro = 0;
static unsigned int contador_accx_zero = 0;
static unsigned int contador_accy_zero = 0;
static float amostras_filtro_media_movel_accX[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL];
static float amostras_filtro_media_movel_accY[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL];

//implementações

//Funcao: callback de leitura do IMU (em modo DMP)
//Parametros: nenhum
//Retorno: nenhum
void chegou_leitura_imu(void)
{
        float accX_filtrada;
	float accY_filtrada;
	float somaX;
	float somaY;
	int i;

	//verifica se está na fase de preenchimento do buffer do filtro de média móvel.
	//Em caso positivo, adiciona a medição no buffer e sai da função
	//Em caso negativo, a medição mais atual é adicionada e o filtro é aplicado.
	if (contador_preenchimento_buffer_filtro < NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL)
	{
		//preenchimento do buffer do filtro de média móvel
		amostras_filtro_media_movel_accX[contador_preenchimento_buffer_filtro] = verifica_janela_acc(data.accel[0] - offset_acc_x);
		amostras_filtro_media_movel_accX[contador_preenchimento_buffer_filtro] = verifica_janela_acc(data.accel[1] - offset_acc_y);
		contador_preenchimento_buffer_filtro++;
		return;
	}
	else
	{
		//a medição mais atual é adicionada e o filtro é aplicado
		somaX = 0.0;
		somaY = 0.0;
		for(i=1; i<NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL; i++)
		{
			amostras_filtro_media_movel_accX[i-1] = amostras_filtro_media_movel_accX[i];
			amostras_filtro_media_movel_accY[i-1] = amostras_filtro_media_movel_accY[i];
			somaX = somaX + amostras_filtro_media_movel_accX[i];
			somaY = somaY + amostras_filtro_media_movel_accY[i];
		}

		amostras_filtro_media_movel_accX[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL-1] = verifica_janela_acc(data.accel[0] - offset_acc_x);
		amostras_filtro_media_movel_accY[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL-1] = verifica_janela_acc(data.accel[1] - offset_acc_y);

		somaX = somaX + amostras_filtro_media_movel_accX[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL-1];
		somaY = somaY + amostras_filtro_media_movel_accY[NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL-1];

		accX_filtrada = somaX / NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL;
		accY_filtrada = somaY / NUMERO_AMOSTRAS_FILTRO_MEDIA_MOVEL;
	}

	contador_interrupcoes_imu++;

    if (contador_interrupcoes_imu == 2)
    {
        accX_t1 = accX_filtrada;
        accY_t1 = accY_filtrada;
        calcula_vel_atual(accX_t0, accY_t0, accX_t1, accY_t1);
        contador_interrupcoes_imu=0;
    }
    else
    {
	accX_t0 = accX_filtrada;
        accY_t0 = accY_filtrada;
    }
}

//Função: Verifica se a amostra passada está dentro da janela de aceleração
//Parâmetros: amostra
//Retorno: 0.0: amostra dentro da janela de aceleração
//         próprio valor de amostra: amostra fora da janela de aceleração
float verifica_janela_acc(float amostra)
{
	if ( abs(amostra) <= JANELA_ZERO_ACC )
		return 0.0;
	else
		return amostra;
}

//Função: atualiza contador de aceleração nula
//Parâmetros: acelerações no eixo referido e ponteiro para contador de acelerações nulas no eixo referido
//Retorno: nenhum
void atualiza_contador_acc_nula(float acc_t0, float acc_t1, unsigned int * ptrContador)
{
	if ( (acc_t0 == 0.0) && (acc_t1 == 0.0) )
        {
		*ptrContador = (*ptrContador) + 1;

                //Garante que o contador chegue ate UINT_MAX.
                //se isso ocorrer, restaura contador para LIMIAR_MEDICOES_ACC_VEL_NULA
                if (*ptrContador == UINT_MAX)
                    *ptrContador = LIMIAR_MEDICOES_ACC_VEL_NULA;
        }
	else
		*ptrContador = 0;
}

//Função: calcula velocidade, com base na leitura do acelerometro
//Parâmetros: nenhum
//Retorno: acelerações x e y em t0 e t1
void calcula_vel_atual(float accX_t0, float accY_t0, float accX_t1, float accY_t1)
{
	float velX, velY, velResultante;

	//verifica se a aceleração em algum dos eixos é nula
	atualiza_contador_acc_nula(accX_t0, accX_t1, &contador_accx_zero);
	atualiza_contador_acc_nula(accY_t0, accY_t1, &contador_accy_zero);

	//calcula a velocidade de acordo com a aceleração em algum dos eixos ser nula por LIMIAR_MEDICOES_ACC_VEL_NULA amostras seguidas
        velX = 0.0;
        if (contador_accx_zero < LIMIAR_MEDICOES_ACC_VEL_NULA)
		velX = vel_x_atual + calcula_delta_velocidade(accX_t0, accX_t1, (float)FREQ_AMOSTRAGEM_ACC_HZ);

        velY = 0.0;
	if (contador_accy_zero < LIMIAR_MEDICOES_ACC_VEL_NULA)
		velY = vel_y_atual + calcula_delta_velocidade(accY_t0, accY_t1, (float)FREQ_AMOSTRAGEM_ACC_HZ);	

	//atualiza variáveis (usadas nas integrações sucessivas)
	vel_x_atual = velX;
	vel_y_atual = velY;

	//calcula a velocidade resultante / linear / tangencial do veiculo
	velResultante = (float)sqrt( (velX*velX) + (velY*velY) );
	set_vel_atual(velResultante);
}

//Função: inicializa obtenção de velocidade por acelerometro.
//        IMPORTANTE: assume-se aqui que a velocidade inicial (em x e y) é zero.
//Parametros: nenhum
//Retorno: status de inicialização
TStatusExec init_velocidade_por_acc(void)
{
    rc_imu_config_t conf;
    vel_atual = 0.0;
    vel_x_atual = 0.0;
    vel_y_atual = 0.0;
    float soma_acc_x, soma_acc_y;
    int i;

    //inicializa IMU
    if(rc_initialize())
	return eStatusExecFailRootPrivillege;

    //inicializa variável que controlará a aquisição das acelerações (entro da função de 
    //interrupção, a chegou_leitura_imu() ) 
    contador_interrupcoes_imu = 0;

    //faz as configuracoes do IMU
    conf = rc_default_imu_config();
    conf.enable_magnetometer = 0;
    conf.accel_fsr = SENSIBILIDADE_ACC;
    conf.accel_dlpf = NIVEL_FILTRO_DLFP;
    conf.dmp_sample_rate = FREQ_AMOSTRAGEM_ACC_HZ;

    //inicializa IMU (em modo "normal" / polling), para a finalidade de calibração do acelerômetro
    rc_initialize_imu(&data, conf);

    //faz a calibracao do acelerometro (determinacao do offset nos eixos x e y)
    i=0;
    soma_acc_x = 0.0;
    soma_acc_y = 0.0;

    while(i < NUMERO_LEITURAS_CALIBRACAO_ACC)
    {
        if (rc_read_accel_data(&data) < 0)
	{
	    //caso uma das leituras falhe, recomeça o processo de calibração
	    DEBUG_VEL("\r\n[ERRO] Falha da leitura da IMU (leitura %d) durante a calibracao.\n", i);
    	    soma_acc_x = 0.0;
            soma_acc_y = 0.0;
	    i=0;
	    continue;
	}

        soma_acc_x = soma_acc_x + data.accel[0];
        soma_acc_y = soma_acc_y + data.accel[1];
        i++;
    }

    offset_acc_x = soma_acc_x / NUMERO_LEITURAS_CALIBRACAO_ACC;
    offset_acc_y = soma_acc_y / NUMERO_LEITURAS_CALIBRACAO_ACC;
    offset_acc_x = offset_acc_x;
    offset_acc_y = offset_acc_y;

    DEBUG_VEL("\r\n[OFFSET X] %f\r\n[OFFSET Y] %f\n", offset_acc_x, offset_acc_y);

    //Uma vez feita a calibração, inicializa o IMU em modo DMP (por interrupçao, de modo 
    //a garantir aquisições à tempo fixo)
    if (rc_initialize_imu_dmp(&data, conf))
	return eStatusExecFailInitIMU;

    //confiogura o callback de leitura do IMU
    rc_set_imu_interrupt_func(chegou_leitura_imu);
    return eStatusExecOK;
}

//Função: finaliza cálculo da velocidade baseado no acelerometro
//Parâmetros: nenhum
//Retorno: nenhum
void finaliza_velocidade_por_acc(void)
{
    //para a leitura do IMU por interrupção
    rc_stop_imu_interrupt_func();

    //finaliza coisas da robotics cape lib
    rc_power_off_imu();
    rc_cleanup();
}


//Função: seta velocidade atual
//Parâmetros: velocidade a ser setada
//Retorno: nenhum
void set_vel_atual(float vel)
{
    vel_atual = vel;
}


//Função: obtem velocidade atual
//Parâmetros: KM_POR_HORA: indica que a velocidade sera em km/h
//            M_POR_S: indica que a velocidade sera em m/s
//Retorno: velocidade atual obtida
float get_vel_atual(char tipo_vel)
{
    float ret_vel;

    switch(tipo_vel)
    {
        case M_POR_S:
            ret_vel = vel_atual;
            break;

        case KM_POR_HORA:
            ret_vel = vel_atual*3.6;
            break;

        default:
            ret_vel=0.0;
            break;
    }

    return ret_vel; 
}

//Função: calcula delta de velocidade, baseado nas acelerações e no periodo de amostragem
//Parâmetros: acelerações (m/s^2) e periodo de amostragem (ms)
//Retorno: delta de velocidade. 
float calcula_delta_velocidade(float accT0, float accT1, float freq_amostragem)
{
	float delta = 0.0;
	float periodo_amostragem_segundos = 1/freq_amostragem;

    /*
	Situação 1: as acelerações em t0 e t1 são iguais. 
	Aqui, a inclinação da curva de aceleração em relação ao eixo X é 0º.
	Neste caso: 
	
	 accT0     accT1  
	   *-------*
	   |       |                     deltaV = Area sob a curva = T * accT0 ou     
	   |       |                                                 T * accT1
	   |       |
	   |       |
	   |       |
	---------------> t(ms)
	   |-  T  -|
	
	 T = período de amostragem
	*/
	if (accT0 == accT1)
	{
		delta = periodo_amostragem_segundos * accT0;
		return delta;
	}

	/*
	Situação 2: a aceleração em t0 é maior que em t1.
	Aqui, a inclinação da curva de aceleração em relação ao eixo X é negativa, logo o delta velocidade é negativo.
        Neste caso: 
	
	  accT0  
	   ^   
	   | \                          deltaV = Area sob a curva = (-1)*(accT0 + accT1)*T*0.5     
	   |  \
  	   |   \  accT1
	   |    \ ^  
	   |       |
	---------------> t(ms)
	   |-  T  -|
	
	 T = período de amostragem
         */
	if (accT0 > accT1)
	{
		delta = (-1)*(accT0 + accT1)*periodo_amostragem_segundos*0.5;
		return delta;
	}
	
	/*
	Situação 3: a aceleração em t0 é menor que em t1.
	Aqui, a inclinação da curva de aceleração em relação ao eixo X é positiva, logo o delta velocidade é positivo.
	Neste caso:

	         accT1
	           ^
	         / |                     deltaV = Area sob a curva = (accT0 + accT1)*T*0.5     
	       /   |
	accT0 /    |
	    ^/     |
	   |       |
	---------------> t(ms)
	   |-  T  -|

	 T = período de amostragem
         */
	//if (accT0 < accT1)
	delta = (accT0 + accT1)*periodo_amostragem_segundos*0.5;
        return delta;
}
