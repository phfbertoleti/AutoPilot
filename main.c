#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <rc_usefulincludes.h>
#include <roboticscape.h>
#include <time.h>
#include <MQTTClient.h>
#include "VelocidadePorAcc.h"
#include "ControlePID.h"
#include "ParametrosDeProjeto.h"

//defines
#define ADDRESS                          "tcp://iot.eclipse.org"
#define CLIENTID                         "MQTTAutoPilotBBBlue"
#define TOPICO_PUBLISH                   "MQTTVelAutoPilot"
#define TOPICO_SUBSCRIBE_AUTOPILOT       "MQTTVelAutoPilotCmd"
#define TOPICO_SUBSCRIBE_CONTROLE_MANUAL "MQTTVelManualCmd"

#define UNIDADE_VEL   KM_POR_HORA

#define STEP_TEMPO_LEITURAS  0.050  //s

//descomente a linha abaixo para habilitar o debug
#define HABILITA_DEBUG_MAIN

//descomente a linha abaixo para habilitar a geração de tabela csv do PID
#define HABILITA_TABELA_CSV_PID

#ifdef HABILITA_DEBUG_MAIN
   #define DEBUG_MAIN(...) printf(__VA_ARGS__)
#endif

//prototypes
void publish(MQTTClient client, char* topic, char* payload);
int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void executa_piloto_automatico(float vel_atual);

//variaveis globais
#ifdef HABILITA_TABELA_CSV_PID
  unsigned long int contador_pontos_pid=0;
#endif

//implementações

//Função: publicação de mensagem via MQTT
//Parametros: client MQTT, tópico e payload.
//Retorno: nenhum
void publish(MQTTClient client, char* topic, char* payload) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = payload;
    pubmsg.payloadlen = strlen(pubmsg.payload);
    pubmsg.qos = 2;
    pubmsg.retained = 0;
    MQTTClient_deliveryToken token;
    MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    MQTTClient_waitForCompletion(client, token, 1000L);
}

//Função: callback de mensagem recebida via MQTT
//Parametros: contexto, ponteiro para array com nome do topico, tamanho do topico e mensagem recebida.
//Retorno: 1: sucesso
int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    char* payload = message->payload;
    DEBUG_MAIN("Topico: %s Msg recebida: %s\n", topicName, payload);

    //Faz a etapa de parse da informacao recebida
    if ( strncmp(topicName, TOPICO_SUBSCRIBE_AUTOPILOT, topicLen) == 0)
        configura_setpoint(atof(payload));

    if ( strncmp(topicName, TOPICO_SUBSCRIBE_CONTROLE_MANUAL, topicLen) == 0)
    {

    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

//Função: executa o piloto automatico (com controle PID de velocidade)
//Parametro: velocidade atual
//Retorno: nenhum
void executa_piloto_automatico(float vel_atual)
{
   float vel_normalizada;
   float duty_cycle_alvo;
   char linha_cmd[100];

   vel_normalizada = vel_atual / VELOCIDADE_FULL_DUTY_CYCLE;
   duty_cycle_alvo = computa_PID(vel_normalizada, STEP_TEMPO_LEITURAS);
   rc_set_motor_all(duty_cycle_alvo);

   //faz log da ação PID (para futura conferencia)
   #ifdef HABILITA_TABELA_CSV_PID
      sprintf(linha_cmd,"echo '%f,%f,%ld' >> /tmp/tabela_pid.csv",le_setpoint(),duty_cycle_alvo,contador_pontos_pid);
      system(linha_cmd);
      contador_pontos_pid++;
   #endif
}

int main(int argc, char *argv[])
{
	TStatusExec StatusExec;
        float maior_vel=0.0;
        float menor_vel=0.0;
        float ultima_vel=0.0;
        float vel;
        const char unidade_velocidade[2][5]={"m/s\0", "km/h\0"};
        time_t ts_print;
        int rc;
        char string_velocidade[10];
        MQTTClient client;
        MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

        //Inicializações do MQTT
        MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
        MQTTClient_setCallbacks(client, NULL, NULL, on_message, NULL);

        if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) 
        {
            DEBUG_MAIN("Falha de conexao MQTT. Codigo de erro: %d\n", rc);
            exit(-1);
        }

        //Faz subscribe nos topicos desejados
        MQTTClient_subscribe(client, TOPICO_SUBSCRIBE_AUTOPILOT, 0);
        MQTTClient_subscribe(client, TOPICO_SUBSCRIBE_CONTROLE_MANUAL, 0);

        //Inicializações de medição e loop principal

	StatusExec = init_velocidade_por_acc();

        if ( StatusExec != eStatusExecOK)
	{
	   DEBUG_MAIN("\r\n[ERRO] Erro ao inicializar calculo de velocidade pelo acelerometro.\n\rCodigo de erro: %d\n",StatusExec);
	   return 0;
	}

        maior_vel = get_vel_atual(UNIDADE_VEL);
        menor_vel = maior_vel;

        //Inicialização dos motores
        rc_enable_motors();
        rc_set_motor_all(0.0);

        DEBUG_MAIN("\r\n[FAZENDO LEITURAS]\n");

        ts_print = time(NULL);

	//configura_setpoint(0.3);

	while (rc_get_state() != EXITING)
	{
             vel = get_vel_atual(UNIDADE_VEL);
             ultima_vel = vel;

             if (vel > maior_vel)
                 maior_vel = vel;

             if (vel < menor_vel)
                  menor_vel = vel;

             if ( (time(NULL) - ts_print) >=1)
             {
                 DEBUG_MAIN("\r[VELOCIDADE] Velocidade atual: %f %s\n", ultima_vel, unidade_velocidade[UNIDADE_VEL]);
                 sprintf(string_velocidade,"%.2f %s", ultima_vel, unidade_velocidade[UNIDADE_VEL]);
                 publish(client, TOPICO_PUBLISH, string_velocidade);
                 ts_print = time(NULL);
             }

             //executa_piloto_automatico(vel);
             rc_usleep(STEP_TEMPO_LEITURAS*1000000);
	}

        //desabilita motores
        rc_disable_motors();

        //desabilita IMU
	finaliza_velocidade_por_acc();

        DEBUG_MAIN("\r\n[LEITURAS FINALIZADAS] Maior: %f km/h    Menor: %f km/h   Ultima: %f km/h\n",maior_vel,menor_vel,ultima_vel);

        //finaliza MQTT
        MQTTClient_disconnect(client, 1000);
        MQTTClient_destroy(&client);
	return 0;
}
