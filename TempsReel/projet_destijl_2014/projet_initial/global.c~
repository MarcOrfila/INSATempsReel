/* Institut National des Sciences Appliquées de Toulouse - Année universitaire 2013/2014

4ème année Informatique et Réseaux - Option Informatique

Groupe B2

Unité de Formation : Systèmes Concurrents et Temps Réel

Responsable d'UF : Claude Baron (claude.baron@insa-toulouse.fr)
Responsable Projet : Pierre-Emmanuel Hladik (pehladik@laas.fr)
Encadrante de TP : Carla Sauvanaud (csauvana@laas.fr)

Membres du Groupe : 

BIDEAUD Marie-Charlotte (bideaud@etud.insa-toulouse.fr)
DUGRAND Marjorie (dugrand@etud.insa-toulouse.fr)
FAYOLLE Nicolas (fayolle@etud.insa-toulouse.fr)
LETT Alexandre (lett@etud.insa-toulouse.fr)
ORFILA Marc (orfila@etud.insa-toulouse.fr)

Date de soutenance : 27 mai 2014 
Remise du rapport et du code : 30 mai 2014
*/

#include "global.h"


// Déclaration des tâches 

// Tâche exécutant la fonction Communiquer
RT_TASK tServeur;

// Tâche exécutant la fonction Connecter
RT_TASK tconnect;

// Tâche exécutant la fonction Deplacer
RT_TASK tmove;

// Tâche exécutant la fonction Envoyer
RT_TASK tenvoyer;

// Tâche exécutant la fonction etat_batterie 
RT_TASK tbatterie;

// Tâche exécutant la fonction Camera 
RT_TASK tcamera;

// Tâche exécutant la fonction Watchdog
RT_TASK twatchdog;

// Tâche exécutant la fonction mission_reach_coordinates
RT_TASK tmission;



// Déclaration des mutex 

// Mutex de protection de la variable etatCommRobot (empêche les accès concurrents)
RT_MUTEX mutexEtat;

// Mutex de protection de la variable etatCommMoniteur (empêche les accès concurrents)
RT_MUTEX mutexEtatMoniteur;

// Mutex de protection de la variable move (empêche les accès concurrents)
RT_MUTEX mutexMove;

// Mutex de protection de la variable Robot (empêche les accès concurrents)
RT_MUTEX mutexRobot;

// Mutex de protection de la variable tentatives (empêche les accès concurrents)
RT_MUTEX mutexErreur;

// Mutex de protection de la variable etatCamera (empêche les accès concurrents)
RT_MUTEX mutexCam;

// Mutex de protection de la variable serveur (empêche les accès concurrents)
RT_MUTEX mutexServeur;

// Mutex de protection de la variable position (empêche les accès concurrents)
RT_MUTEX mutexPosition;

// Mutex de protection de la variable arene (empêche les accès concurrents)
RT_MUTEX mutexArene;

// Mutex de protection de la variable mission (empêche les accès concurrents) 
RT_MUTEX mutexMission;

// Mutex de protection de la variable etatMission (empêche les accès concurrents)
RT_MUTEX mutexEtatMission;



// Déclaration des sémaphores

// Sémaphore de déclenchement de la routine de Watchdog
RT_SEM semWatchdog;

// Sémaphore de déclencher d'une tentative de connexion au Robot
RT_SEM semConnecterRobot;

// Sémaphore de déclenchement de la routine de Deplacer
RT_SEM semDeplacer;

// Sémaphore de déclenchement d'une mission 
RT_SEM semEffectuerMission;

// Sémaphore de déclenchement de la routine d'acquisition du niveau de batterie
RT_SEM semBatterie;



// Queue de messages 
RT_QUEUE queueMsgGUI;


// Déclaration des variables globales d'état 

/* Variable renseignant l'état de la communication avec le moniteur. 
Valeurs : 
0 si communication OK
=/= 0 sinon */
int etatCommMoniteur = 1;

/* Variable renseignant l'état de la communication avec le moniteur. 
Valeurs : 
0 si STATUS_OK
=/= 0 sinon 
FIXME : Avec watchdog, erreur STATUS_ERR_CHECKSUM ignorée en modifiant la valeur de STATUS_ERR_CHECKSUM à 0 (STATUS_OK) dans d_constantes.h */
int etatCommRobot = 1;

/* Variable renseignant l'état de fonctionnement de la camera
Valeurs : 
ACTION_STOP_COMPUTE_POSITION : Acquisition d'images sans calcul de position
ACTION_COMPUTE_CONTINUOUSLY_POSITION : Acquisition d'images avec calcul de position
ACTION_FIND_ARENA : Recherche d'arène 
ACTION_ARENA_IS_FOUND : Détection fructueuse d'arène
ACTION_ARENA_FAILED : Arrêt de recherche d'arène */
int etatCamera = ACTION_STOP_COMPUTE_POSITION;

/* Variable renseignant l'état de la mission 
Valeurs : 
NONE : Pas de mission 
START : Routine de début de mission 
PENDING : Mission en cours 
TERMINATED : Mission terminée*/ 
etats_mission etatMission = NONE;

// Variable renseignant le nombre de tentatives échouées d'interaction avec le robot consécutives 
int tentatives = 0;


// Déclaration des variables globales des structures de systèmes 

// Structure du robot 
DRobot *robot;

// Structure d'un mouvement 
DMovement *move;

// Structure du serveur 
DServer *serveur;

// Structure d'un position (robot ou objectif)
DPosition *position;

// Structure d'une arene 
DArena *arene;

// Structure d'une mission 
DMission *mission;

// Taille de la queue de messages 
int MSG_QUEUE_SIZE = 10;

// Déclaration des priorités des Threads 

/* Afin d'être sûr de bien gérer les pertes de connexion, le thread tWatchdog doit avoir la priorité la plus importante*/
int PRIORITY_TWATCHDOG = 99;

/* Suite à une multitude de "Segmentation Fault", la seule solution empirique est de relever la priorité de tEnvoyer au dessus de tous les threads utilisant la fonction write_in_queue*/
int PRIORITY_TENVOYER = 95;   

/* tCamera possède la 3ème priorité la plus importante car le déroulement de mission est conditionné par les informations récupérées par le thread Camera */
int PRIORITY_TCAMERA = 35;

/* Le thread mission doit être pré-empté au minimum par les autres threads, on place donc sa priorité à la 4ème position */
int PRIORITY_TMISSION = 30;

/* Tous les messages du serveur doivent être traités en priorité*/
int PRIORITY_TSERVEUR = 25;

/* Lors d'une demande de connexion, le thread tConnect a besoin d'une priorité moyenne car les autres threads sont bloqués par sémaphore à ce moment là. Avec cette priorité d'importance moyenne (5ème sur 7), on évite ainsi que lors de l'utilisation normale, les autres threads soient préemptés fréquemment par tConnect */
int PRIORITY_TCONNECT = 20;

/* La priorité de tMove est basse car la fonction de déplacement du robot est moins prioritaire que les threads capitaux pour le bon fonctionnement du robot, ET pour la réalisation de la mission. De plus, la très courte période de tMove permet de contrebalancer cette faible priorité avec une réactivité satisfaisante */
int PRIORITY_TMOVE = 10;

/* tBatterie possède la priorité la plus basse car cette fonctionnalité n'a aucun lien avec le bon fonctionnement du système */
int PRIORITY_TBATTERIE = 5; 


