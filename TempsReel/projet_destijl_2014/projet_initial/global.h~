/* 
 * File:   global.h
 * Author: pehladik
 *
 * Created on 12 janvier 2012, 10:11
 */

#ifndef GLOBAL_H
#define	GLOBAL_H

#include "includes.h"

//type énuméré reprenant les différents états d'une mission
typedef enum etats_mission {TERMINATED, COMPLETED, PENDING, TURN, MOVE, START} etats_mission;

/* @descripteurs des tâches */
extern RT_TASK tServeur;
extern RT_TASK tconnect;
extern RT_TASK tmove;
extern RT_TASK tenvoyer;
extern RT_TASK tbatterie;
extern RT_TASK tmission;
extern RT_TASK tcamera;
extern RT_TASK twatchdog;

/* @descripteurs des mutex */
extern RT_MUTEX mutexEtat;
extern RT_MUTEX mutexMove;
extern RT_MUTEX mutexRobot;
extern RT_MUTEX mutexMission;

extern RT_MUTEX mutexErreur;
extern RT_MUTEX mutexCam;
extern RT_MUTEX mutexServeur;
extern RT_MUTEX mutexPosition;
extern RT_MUTEX mutexArene;
extern RT_MUTEX mutexMissionData;   /// utilisé ou pas?
extern RT_MUTEX mutexTentatives;

/* @descripteurs des sempahore */
extern RT_SEM semConnecterRobot;

/* @descripteurs des files de messages */
extern RT_QUEUE queueMsgGUI;

/* @variables partagées */
extern int etatCommMoniteur;
extern int etatCommRobot;
extern etats_mission etatMission;
extern int tentatives;

extern DServer *serveur;
extern DRobot *robot;
extern DMovement *move;
extern DMission *mission;

/* @constantes */
extern int MSG_QUEUE_SIZE;
extern int PRIORITY_TSERVEUR;
extern int PRIORITY_TCONNECT;
extern int PRIORITY_TMOVE;
extern int PRIORITY_TENVOYER;
extern int PRIORITY_TBATTERIE;

#endif	/* GLOBAL_H */

