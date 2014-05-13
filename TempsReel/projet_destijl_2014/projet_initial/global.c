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

RT_SEM semWatchdog;
RT_SEM semConnecterRobot;
RT_SEM semDeconnecterRobot;

RT_QUEUE queueMsgGUI;

int etatCommMoniteur = 1;
int etatCommRobot = 1;
etats_mission etatMission;

DRobot *robot;
DMovement *move;
DServer *serveur;
DPosition *position;
DArena *arene;
DMission *missionData;
DMission *mission;

int MSG_QUEUE_SIZE = 10;

int PRIORITY_TSERVEUR = 30;
int PRIORITY_TCONNECT = 20;
int PRIORITY_TMOVE = 10;
int PRIORITY_TENVOYER = 25;
int PRIORITY_TBATTERIE = 5;   // on considere le deplacement plus important que la batterie
