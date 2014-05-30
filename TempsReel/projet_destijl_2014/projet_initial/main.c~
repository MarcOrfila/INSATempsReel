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


#include "includes.h"
#include "global.h"
#include "fonctions.h"

/**
 * \fn void initStruct(void)
 * \brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void initStruct(void);

/**
 * \fn void startTasks(void)
 * \brief Démarrage des tâches
 */
void startTasks(void);

/**
 * \fn void deleteTasks(void)
 * \brief Arrêt des tâches
 */
void deleteTasks(void);

int main(int argc, char**argv) {
    printf("#################################\n");
    printf("#      DE STIJL PROJECT         #\n");
    printf("#################################\n");

    //signal(SIGTERM, catch_signal);
    //signal(SIGINT, catch_signal);

    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT | MCL_FUTURE);
    /* For printing, please use rt_print_auto_init() and rt_printf () in rtdk.h
     * (The Real-Time printing library). rt_printf() is the same as printf()
     * except that it does not leave the primary mode, meaning that it is a
     * cheaper, faster version of printf() that avoids the use of system calls
     * and locks, instead using a local ring buffer per real-time thread along
     * with a process-based non-RT thread that periodically forwards the
     * contents to the output stream. main() must call rt_print_auto_init(1)
     * before any calls to rt_printf(). If you forget this part, you won't see
     * anything printed.
     */
    rt_print_auto_init(1);
    initStruct();
    startTasks();
	pause();
    deleteTasks();

    return 0;
}

void initStruct(void) {
    int err;
    /* Creation des mutex */
    
    // Mutex pour la variable globale etatCommRobot
    if (err = rt_mutex_create(&mutexEtat, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la variable globale etatCommMoniteur
    if (err = rt_mutex_create(&mutexMove, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la variable globale etatMission
    if (err = rt_mutex_create(&mutexEtatMission, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la structure globale mission 
    if (err = rt_mutex_create(&mutexMission, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la structure globale position
    if (err = rt_mutex_create(&mutexPosition, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la Structure globale Robot 
    if (err = rt_mutex_create(&mutexRobot, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la structure globale serveur
    if (err = rt_mutex_create(&mutexServeur, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la structure globale camera
    if (err = rt_mutex_create(&mutexCam, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la structure globale arene
    if (err = rt_mutex_create(&mutexArene, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la variable globale tentatives 
    if (err = rt_mutex_create(&mutexErreur, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Mutex pour la variable globale etatCommMoniteur
    if (err = rt_mutex_create(&mutexEtatMoniteur, NULL)) {
        rt_printf("Error mutex create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    /* Creation des semaphores */
    
    // Sémaphore de déclenchement d'une tentative de connexion avec le robot
    if (err = rt_sem_create(&semConnecterRobot, NULL, 0, S_FIFO)) {
        rt_printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
	// Sémaphore de déclenchement d'une mission 
    if (err = rt_sem_create(&semEffectuerMission, NULL, 0, S_FIFO)) {
        rt_printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Sémaphore de déclenchement de la routine du watchdog
    if (err = rt_sem_create(&semWatchdog, NULL, 0, S_FIFO)) {
        rt_printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Sémaphore de déclenchement de la routine de Déplacer
    if (err = rt_sem_create(&semDeplacer, NULL, 0, S_FIFO)) {
    	rt_printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Sémaphore de déclenchement de la récupération du niveau de batterie
	if (err = rt_sem_create(&semBatterie, NULL, 0, S_FIFO)) {
    	rt_printf("Error semaphore create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
	
    /* Creation des taches tenvoyer : envoi d'un message au moniteur*/
    
    // Création de la tâche tServeur
    if (err = rt_task_create(&tServeur, NULL, 0, PRIORITY_TSERVEUR, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tache tConnect 
    if (err = rt_task_create(&tconnect, NULL, 0, PRIORITY_TCONNECT, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche tmove 
    if (err = rt_task_create(&tmove, NULL, 0, PRIORITY_TMOVE, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche tenvoyer 
    if (err = rt_task_create(&tenvoyer, NULL, 0, PRIORITY_TENVOYER, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche twatchdog
    if (err = rt_task_create(&twatchdog, NULL, 0, PRIORITY_TWATCHDOG, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche tbatterie 
    if (err = rt_task_create(&tbatterie, NULL, 0, PRIORITY_TBATTERIE, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche tmission
    if (err = rt_task_create(&tmission, NULL, 0, PRIORITY_TMISSION, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Création de la tâche tcamera
    if (err = rt_task_create(&tcamera, NULL, 0, PRIORITY_TCAMERA, 0)) {
        rt_printf("Error task create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    
    /* Creation des files de messages */
    if (err = rt_queue_create(&queueMsgGUI, "MessageQueue", MSG_QUEUE_SIZE*sizeof(DMessage), MSG_QUEUE_SIZE, Q_FIFO)){
        rt_printf("Error msg queue create: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }

    /* Instanciation des structures globales du projet */
    robot = d_new_robot();
    move = d_new_movement();
    serveur = d_new_server();
    mission = d_new_mission();
    position = d_new_position();
    arene = NULL;
}

void startTasks() {
    int err;
    // Démarrage tâche tconnect (fonction connecter)
    if (err = rt_task_start(&tconnect, &connecter, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche tserveur (fonction communiquer
    if (err = rt_task_start(&tServeur, &communiquer, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche tmove (fonction deplacer)
    if (err = rt_task_start(&tmove, &deplacer, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche tenvoyer (fonction envoyer)
    if (err = rt_task_start(&tenvoyer, &envoyer, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche tbatterie (fonction etat_batterie)
    if (err = rt_task_start(&tbatterie, &etat_batterie, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche twatchdog (fonction watchdog)
    // HACK : Meilleure réactivité du mode start_insecurely du robot en ne lançant pas twatchdog
   	if (err = rt_task_start(&twatchdog, &watchdog, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
    // Démarrage tâche tmission (fonction mission_reach_coordinates)
    if (err = rt_task_start(&tmission, &mission_reach_coordinates, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }    
    // Démarrage tâche camera (fonction camera)
    if (err = rt_task_start(&tcamera, &camera, NULL)) {
        rt_printf("Error task start: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    }
}

// Suppression des tâches 
void deleteTasks() {
    rt_task_delete(&tServeur);
    rt_task_delete(&tconnect);
    rt_task_delete(&tmove);
    rt_task_delete(&tenvoyer);
    rt_task_delete(&tbatterie);
    rt_task_delete(&tmission);
    rt_task_delete(&tcamera);
    rt_task_delete(&twatchdog);
}
