/*
 * File:   global.h
 * Author: pehladik
 *
 * Created on 21 avril 2011, 12:14
 */

#include "global.h"

RT_TASK tServeur;
RT_TASK tconnect;
RT_TASK tdeconnect;
RT_TASK tmove;
RT_TASK tenvoyer;
RT_TASK tbatterie;
RT_TASK tcamera;
RT_TASK twatchdog;
RT_TASK tmission;

RT_MUTEX mutexEtat;
RT_MUTEX mutexMove;
RT_MUTEX mutexRobot;

RT_MUTEX mutexErreur;
RT_MUTEX mutexCam;
RT_MUTEX mutexServeur;
RT_MUTEX mutexPosition;
RT_MUTEX mutexArene;
RT_MUTEX mutexMissionData;
RT_MUTEX mutexMission;
RT_MUTEX mutexTentatives;
RT_MUTEX mutexEtatMission;

RT_SEM semWatchdog;
RT_SEM semConnecterRobot;
RT_SEM semDeconnecterRobot;
RT_SEM semEffectuerMission;

RT_QUEUE queueMsgGUI;

int etatCommMoniteur = 1;
int etatCommRobot = 1;
int etatCamera = ACTION_STOP_COMPUTE_POSITION;
etats_mission etatMission;

DRobot *robot;
DMovement *move;
DServer *serveur;
DPosition *position;
DArena *arene;
DMission *missionData;
DMission *mission;

int tentatives = 0;
int MSG_QUEUE_SIZE = 10;

int PRIORITY_TSERVEUR = 30;
int PRIORITY_TCONNECT = 20;
int PRIORITY_TMOVE = 10;
int PRIORITY_TENVOYER = 35;   // Pour éviter les segfaults, il faut que la priorité de tenvoyer soit supérieure à toutes celles des fonctions qui utilisent un write in queue
int PRIORITY_TBATTERIE = 5; 
int PRIORITY_TMISSION = 27;
int PRIORITY_TWATCHDOG = 40;
int PRIORITY_TCAMERA = 30;
