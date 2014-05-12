#include "fonctions.h"

int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);

void envoyer(void * arg) {
    DMessage *msg;
    int err;

    while (1) {
        rt_printf("tenvoyer : Attente d'un message\n");
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            serveur->send(serveur, msg);
            msg->free(msg);
        } else {
            rt_printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void connecter(void * arg) {
    int status;
    DMessage *message;

    rt_printf("tconnect : Debut de l'exécution de tconnect\n");

    while (1) {
        rt_printf("tconnect : Attente du sémarphore semConnecterRobot\n");
        rt_sem_p(&semConnecterRobot, TM_INFINITE);
        rt_printf("tconnect : Ouverture de la communication avec le robot\n");
        status = robot->open_device(robot);

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        etatCommRobot = status;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {
            status = robot->start_insecurely(robot);    // Attente des ordres sans watchdog
            if (status == STATUS_OK){
                rt_printf("tconnect : Robot demarre\n");
            }
            // gerer les autres cas (cas d'erreurs)
        }

        message = d_new_message();
        message->put_state(message, status);

        rt_printf("tconnecter : Envoi message\n");
        message->print(message, 100);

        if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            message->free(message);
        }
    }
}

void communiquer(void *arg) {
 DMessage *msg = d_new_message();
    int var1 = 1;   // Var qui represente le nombre d'octets recus
    int num_msg = 0;

    rt_printf("tserver : Début de l'exécution de serveur\n");
    serveur->open(serveur, "8000");
    rt_printf("tserver : Connexion\n");
	// TODO  : connexion avec la camera
    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
    etatCommMoniteur = 0;
    rt_mutex_release(&mutexEtat);

    while (var1 > 0) {
        rt_printf("tserver : Attente d'un message\n");
        var1 = serveur->receive(serveur, msg); // gérer si la connexion avec le moniteur a été perdue --> reception failed
        num_msg++;
        if (var1 > 0) {
            switch (msg->get_type(msg)) {
                case MESSAGE_TYPE_ACTION:
                    rt_printf("tserver : Le message %d reçu est une action\n", num_msg);
                    DAction *action = d_new_action();
                    action->from_message(action, msg);
                    switch (action->get_order(action)) {    
                        case ACTION_CONNECT_ROBOT: 
                            rt_printf("tserver : Action connecter robot\n");
                            rt_sem_v(&semConnecterRobot);
                            break;
                        case ACTION_COMPUTE_CONTINUOUSLY_POSITION:
                            rt_printf("tserver : Calcul périodique position robot\n");
                            //lancer le calcul périodique de la position du robot
                            break;
                        case ACTION_FIND_ARENA:
                            rt_printf("tserver : Action recherche arene\n");
                            //lancer la recherche de l'arene
                            break;
                        case ACTION_ARENA_FAILED:
                            rt_printf("tserver : Arret recherche arene, retour acquisition image\n");
                            //stopper la recherche de l'arene
                            //relancer l'acquisition d'image
                            break;
                        case ACTION_ARENA_IS_FOUND:
                            rt_printf("tserver : Sauvegarde arene trouvee, retour acquisition image\n");
                            //sauvegarder l'image de l'arene
                            //relancer l'acquisition d'image
                            break;
                        default :
                        	rtprintf("ERREUR tserver : Type action non reconnu\n");
                        	break;
                        
                    }
                    break;
                case MESSAGE_TYPE_MOVEMENT:                 
                    rt_printf("tserver : Le message reçu %d est un mouvement\n", num_msg);
                    rt_mutex_acquire(&mutexMove, TM_INFINITE);
                    move->from_message(move, msg);
                    move->print(move);
                    rt_mutex_release(&mutexMove);
                    break;
                case MESSAGE_TYPE_MISSION: 
                    rt_printf("tserver : Le message reçu %d est une mission\n", num_msg);
                    mission->from_message(action,msg);
                    switch(mission->type) {
                    	case MISSION_TYPE_STOP : 
                    		//fin de la mission
                    		rt_mutex_acquire(&mutexMission, TM_INFINITE);
                    		etatMission = TERMINATED;
                    		rt_mutex_release(&mutexMission, TM_INFINITE);
                    		break;
                    	case MISSION_TYPE_REACH_COORDINATE :
                    		//assignement d'une mission
                    		rt_mutex_acquire(&mutexMission, TM_INFINITE);
                    		etatMission = START;
                    		rt_mutex_release(&mutexMission, TM_INFINITE);
                    		break;
                    	default :
                    		rt_printf("erreur, type %d non traité\n", mission->type);
                    		break;
                    };
                    break;
                default:
                    rt_printf("ERREUR tserver : Le message recu %d a un type indefini\n", num_msg);
                    break;
            }
        }
    }
}

void etat_batterie(void *arg) {
    int niveau = -1;  
    int status ;  
    DMessage * message;
    DBattery * batterie;
    
    rt_printf("tbatterie : Debut de l'éxecution periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
   
     while (1) {
     	//prend
     	//libere
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        rt_printf("tbatterie : Activation périodique\n");
        batterie = d_new_battery();
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status=etatCommRobot;
        rt_mutex_release(&mutexEtat);
	
	while (status == STATUS_OK) {
	    rt_mutex_acquire(&mutexRobot, TM_INFINITE);
    	    status = robot->get_vbat(robot, &niveau);   // Renvoie 0, 1 ou 2 (= etat de la batterie)
    	    rt_mutex_release(&mutexRobot);
    	    
    	    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
            etatCommRobot = status;
            rt_mutex_release(&mutexEtat);
        
    	    /*
    	    if (status == STATUS_OK) rt_printf("tbatterie2 : status OK\n");
    	    else if (status == STATUS_ERR_TIMEOUT) rt_printf("tbatterie2 : erreur timeout\n");
    	    else if (status == STATUS_ERR_UNKNOWN_CMD) rt_printf("tbatterie2 : erreur commande introuvable\n");
    	    else rt_printf("tbatterie2 : erreur params ou commande rejetés\n");
    	    // rt_printf("etat de la batterie : %d\n", batterie); 
    	    */
    	    
    	    batterie->set_level(batterie, niveau);
    	    
    	    message = d_new_message();
   	    message->put_battery_level(message, batterie);

   	    rt_printf("tbatterie : Envoi message\n");
   	    message->print(message, 100);

   	    if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                message->free(message);
   	    }
        }	
     }
} 

void deplacer(void *arg) {   // Argument = mouvement ?
    int status = 1;
    int gauche;
    int droite;
    DMessage *message;

    rt_printf("tmove : Debut de l'éxecution de periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);

    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);

        if (status == STATUS_OK) {
            rt_mutex_acquire(&mutexMove, TM_INFINITE);
            switch (move->get_direction(move)) {
                case DIRECTION_FORWARD:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_LEFT:
                    gauche = MOTEUR_ARRIERE_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
                case DIRECTION_RIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_ARRIERE_LENT;
                    break;
                case DIRECTION_STOP:
                    gauche = MOTEUR_STOP;
                    droite = MOTEUR_STOP;
                    break;
                case DIRECTION_STRAIGHT:
                    gauche = MOTEUR_AVANT_LENT;
                    droite = MOTEUR_AVANT_LENT;
                    break;
            }
            rt_mutex_release(&mutexMove);

            status = robot->set_motors(robot, gauche, droite);

            if (status != STATUS_OK) {
                rt_mutex_acquire(&mutexEtat, TM_INFINITE);
                etatCommRobot = status;
                rt_mutex_release(&mutexEtat);

                message = d_new_message();
                message->put_state(message, status);

                rt_printf("tmove : Envoi message\n");
                if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    message->free(message);
                }
            }
        }
    }
}

int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) {
    void *msg;
    int err;

    msg = rt_queue_alloc(msgQueue, size);
    memcpy(msg, &data, size);

    if ((err = rt_queue_send(msgQueue, msg, sizeof (DMessage), Q_NORMAL)) < 0) {
        rt_printf("Error msg queue send: %s\n", strerror(-err));
    }
    rt_queue_free(&queueMsgGUI, msg);

    return err;
}

