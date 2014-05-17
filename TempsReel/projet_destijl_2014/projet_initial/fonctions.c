#include "fonctions.h"
#include <math.h>

#define TODEGRE *180/3.14


int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);
int verifierPerteConnexion();
void connexionMoniteurPerdue();

void envoyer(void * arg) {
    DMessage *msg;
    int err;
// ceci est un commentaire
    while (1) {
        rt_printf("tenvoyer : Attente d'un message\n");
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            if (serveur->send(serveur, msg) < 0)
            	connexionMoniteurPerdue();
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
 DMission * missionLocal;
    int var1 = 1;   // Var qui represente le nombre d'octets recus
    int num_msg = 0;
    //int busy; //représente l'état occupé ou non du robot
    int again = 1;
    
    rt_printf("tserver : Début de l'exécution de serveur\n");
    if (serveur->open(serveur, "8000");
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
                            rt_mutex_acquire(&mutexCam, TM_INFINITE);
                            etatCamera = ACTION_COMPUTE_CONTINUOUSLY_POSITION;
                            rt_mutex_release(&mutexCam);
                            break;
                        case ACTION_FIND_ARENA:
                            rt_printf("tserver : Action recherche arene\n");
                            //lancer la recherche de l'arene
                            rt_mutex_acquire(&mutexCam, TM_INFINITE);
                            etatCamera = ACTION_FIND_ARENA;
                            rt_mutex_release(&mutexCam);
                            break;
                        case ACTION_ARENA_FAILED:
                            rt_printf("tserver : Arret recherche arene, retour acquisition image\n");
                            //stopper la recherche de l'arene
                            //relancer l'acquisition d'image
                            rt_mutex_acquire(&mutexCam, TM_INFINITE);
                            etatCamera = ACTION_ARENA_FAILED;
                            rt_mutex_release(&mutexCam);
                            break;
                        case ACTION_ARENA_IS_FOUND:
                            rt_printf("tserver : Sauvegarde arene trouvee, retour acquisition image\n");
                            //sauvegarder l'image de l'arene
                            //relancer l'acquisition d'image
                            rt_mutex_acquire(&mutexCam, TM_INFINITE);
                            etatCamera = ACTION_ARENA_IS_FOUND;
                            rt_mutex_release(&mutexCam);
                            
                            break;
                        default :
                        	rt_printf("ERREUR tserver : Type action non reconnu\n");
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
                    missionLocal = d_new_mission();
                    missionLocal->from_message(missionLocal,msg);
                    
                    switch(missionLocal->type) {
                    	case MISSION_TYPE_STOP : 
                    		//fin de la mission
                    		rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
                    		etatMission = TERMINATED;
                    		rt_mutex_release(&mutexEtatMission);
                    		
                    		//arrêt du robot
				//VERIFIER SI ON DOIT VERIFIER L'ETAT DU ROBOT AVANT
				again = 1;
				rt_mutex_acquire(&mutexRobot, TM_INFINITE);
				while (again) {
					if (d_robot_stop(robot) == STATUS_OK) {
						//réception du message par le robot ok
						again = 0;
						//on remet à zéro le nombre d'échecs consécutifs
                    				tentatives = 0;			      
					} else {
						//pb de réception du message par le robot
                    				//on vérifie si la connexion avec le robot a vraiment été perdue
                    				again = verifierPerteConnexion();
					}
				}
				rt_mutex_release(&mutexRobot);
                    		break;
                    		
                    	case MISSION_TYPE_REACH_COORDINATE :
                    		//assignement d'une mission
                    		rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
                    		//on vérifie si une mission est déjà en cours
                    		if (etatMission == NONE) {
                    			//aucune mission n'est en cours
                    			//on assigne la nouvelle mission
                    			etatMission = PENDING;
                    			rt_mutex_release(&mutexEtatMission);
                    			rt_mutex_acquire(&mutexMission, TM_INFINITE);
                    			mission = missionLocal;
                    			rt_mutex_release(&mutexMission);
                    			rt_sem_v(&semEffectuerMission);
                    		} else {
                    			//une mission est déjà en cours
                    			//pour l'instant on ne prend pas en compte l'ordre de mission reçu
                    			//TODO ranger l'ordre de mission dans une liste de missions en attente
                    			rt_mutex_release(&mutexEtatMission);
                    			missionLocal->free(missionLocal);
                    		}
                    		break;
                    	default :
                    		rt_printf("erreur, type %d non traité\n", missionLocal->type);
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
    
    rt_printf("tbatterie : Debut de l'éxecution periodique à 5s\n");
    rt_task_set_periodic(NULL, TM_NOW, 5000000000);
   
     while (1) {
     	//prend
     	//libere
     	// TODO : gérer le mutex 
     	
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);     
        rt_printf("tbatterie : Activation périodique\n");
        batterie = d_new_battery();
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status=etatCommRobot;
        rt_mutex_release(&mutexEtat);
	
	if (status == STATUS_OK) {
	    rt_mutex_acquire(&mutexRobot, TM_INFINITE);
    	    status = robot->get_vbat(robot, &niveau);   // Renvoie 0, 1 ou 2 (= etat de la batterie)
    	    rt_mutex_release(&mutexRobot);
    	    
    	    rt_mutex_acquire(&mutexEtat, TM_INFINITE);
            etatCommRobot = status;
            rt_mutex_release(&mutexEtat);
    	    
    	    batterie->set_level(batterie, niveau);
    	    
    	    message = d_new_message();
   	    message->put_battery_level(message, batterie);
   	    rt_printf("\n\n\ntbatterie : etat de la batterie = %d \n\n\n", niveau);

   	    rt_printf("tbatterie : Envoi message\n");
   	    message->print(message, 100);

   	    if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                message->free(message);	
   	    }
        }
        batterie->free(batterie);
     }
} 

void deplacer(void *arg) {   
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

void mission_reach_coordinates(void * arg) {

	DPosition * destination = d_new_position();
	DMessage * msg;
	float x_robot, y_robot, x_destination, y_destination, dx, dy, distance, angle, h_arene, w_arene;
	etats_mission etatMissionLocal = START;
	int busy, again, sens;
	
	while(1) {
		//attente d'une mission
		rt_printf("tmission : Attente du sémaphore semEffectuerMission\n");
		rt_sem_p(&semEffectuerMission, TM_INFINITE);
		etatMissionLocal = START;
		
		while (etatMissionLocal != NONE) {
			//récupération de l'état de la mission
			rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
			etatMissionLocal = etatMission;
			rt_mutex_release(&mutexEtatMission);
		
			switch(etatMissionLocal) {
		
				case START : //début de la mission
					//récupération de la destination
					rt_mutex_acquire(&mutexMission, TM_INFINITE);
					d_mission_get_position(mission, destination);
					rt_mutex_release(&mutexMission);
					x_destination = d_position_get_x(destination);
					y_destination = d_position_get_y(destination);
					//vérifier que la destination est bien dans l'arène
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					h_arene = arene -> d_arena_get_height();
					w_arene = arene -> d_arena_get_width();
					if (0 <= x_destination && x_destination <= w_arene
					    && 0 <= y_destination && y_destination <= h_arene) {
					    	//la destination est bien dans l'arene
					    	//mettre la caméra dans un état de calcul de position du robot
						rt_mutex_acquire(&mutexCam, TM_INFINITE);
						etatCamera = ACTION_COMPUTE_CONTINUOUSLY_POSITION;
						rt_mutex_release(&mutexCam);
					
					    	//on change l'état de la mission à PENDING
					    	rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					    	if (etatMission != TERMINATED)
					    		etatMission = PENDING;
						rt_mutex_release(&mutexEtatMission);
					} else {
						//la destination n'est pas dans l'arene
						rt_printf("tmission : erreur, la destination demandé n'est pas dans l'arene\n");
						//on change l'état de la mission à TERMINATED
						rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					    	etatMission = TERMINATED;
						rt_mutex_release(&mutexEtatMission);
					}
					break;
				
				case PENDING : //mission en cours à effectuer
					//récupération de la position
					rt_mutex_acquire(&mutexPosition,TM_INFINITE);
					x_robot = d_position_get_x(position);
					y_robot = d_position_get_y(position);
					rt_mutex_release(&mutexPosition);
				
					//on vérifie si le robot est arrivé à destination
					dx = x_robot - x_destination;
					dy = y_robot - y_destination;
		
					if ((dx == 0) && (dy == 0)) {
						//mission accomplie
						//on change l'état de la mission à TERMINATED
						rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					    	etatMission = TERMINATED;
						rt_mutex_release(&mutexEtatMission);
					} else {
					//mission non accomplie
					//le robot va devoir bouger (d'abord tourner puis se déplacer)
			
					//calcul de l'angle selon lequel le robot doit tourner
					if (dy == 0) {
						//le robot se trouve sur la même ligne que son objectif
						if (dx < 0) {
							//le robot doit aller sur sa droite
					       		angle = - d_position_get_orientation(&positionLocale)TODEGRE;
					       	} else {
					       		//le robot doit aller sur sa gauche
					       		angle = 180 - d_position_get_orientation(&positionLocale)TODEGRE;
					       	}
					} else if (dy > 0) {
						//le robot se trouve au-dessus de son objectif
						angle = - d_position_get_orientation(&positionLocale)TODEGRE + 270 - atan(dx/dy)TODEGRE;
					} else {
						//le robot se trouve en-dessous de son objectif
						angle = - d_position_get_orientation(&positionLocale)TODEGRE + 90 - atan(dx/dy)TODEGRE;
					}
			
					if (angle > 180)
						angle = angle - 360;
					if (angle < - 180)
						angle = angle + 360;
			
					if (angle > 0)	
						sens = HORAIRE;
					else
						sens = ANTI_HORAIRE;
					       
					//envoi du message au robot
					busy = 1;
					//on attend que le robot ait fini sa tâche
					rt_mutex_acquire(&mutexRobot, TM_INFINITE);
					while (busy == 1) {
						//GESTION DE LA PERTE DE CONNEXION
						again = 1;
						while(again) {
							if (d_robot_is_busy(robot, &busy) == STATUS_OK) {
					       			//réception du message par le robot ok
					       			again = 0;
					       			//on remet à zéro le nombre d'échecs consécutifs
					       			tentatives = 0;                
					      	 	} else {
					       			//pb de réception du message par le robot
								//on vérifie si la connexion avec le robot a vraiment été perdue
								again = verifierPerteConnexion();
							}
						}
					}
			
					//le robot a fini sa tâche
					//envoi de l'ordre de rotation
					again = 1;
					while (again) {
						if (d_robot_turn(robot, fabs(angle), sens) == STATUS_OK) {
							//réception du message par le robot ok
							again = 0;
							//on remet à zéro le nombre d'échecs consécutifs
							tentatives = 0;               
						} else {
							//pb de réception du message par le robot
							//on vérifie si la connexion avec le robot a vraiment été perdue
							again = verifierPerteConnexion();
						}
					}
					rt_mutex_release(&mutexRobot);
			
					//le robot a effectué sa rotation, il doit se déplacer
					//calcul de la distance à parcourir
					distance = sqrt(dx*dx + dy*dy);
					       	
					busy = 1;
			
					//on attend que le robot ait fini sa tâche
					rt_mutex_acquire(&mutexRobot, TM_INFINITE);
					while (busy == 1) {
						//GESTION DE LA PERTE DE CONNEXION
						again = 1;
						while (again) {
							if (d_robot_is_busy(robot, &busy) == STATUS_OK) {
								//réception du message par le robot ok
								again = 0;
								//on remet à zéro le nombre d'échecs consécutifs
								tentatives = 0;                 
							} else {
								//pb de réception du message par le robot
								//on vérifie si la connexion avec le robot a vraiment été perdue
								again = verifierPerteConnexion();
							}
						}	
					}
			
					//envoi de l'ordre de mouvement
					again = 1;
					while (again) {
						if (d_robot_move(robot, distance) == STATUS_OK) {
							//réception du message par le robot ok
							again = 0;
							//on remet à zéro le nombre d'échecs consécutifs
							tentatives = 0;                 
						} else {
							//pb de réception du message par le robot
							//on vérifie si la connexion avec le robot a vraiment été perdue
							again = verifierPerteConnexion();
						}	
					}
					rt_mutex_release(&mutexRobot);
				}
				break;
			
				case TERMINATED : //fin de la mission
					//envoi d'un message indiquant la fin de la mission
					msg = d_new_message();
					d_message_mission_terminate(msg,d_mission_get_id(mission));
					if (write_in_queue(&queueMsgGUI, msg, sizeof (DMessage)) < 0)
						msg->free(msg);
			
					//on indique qu'aucune mission n'est en cours
					//TODO : améliorer en implémentant la liste de missions à effectuer
					rt_mutex_acquire(&mutexEtatMission,TM_INFINITE);
					etatMission = NONE;
					rt_mutex_release(&mutexEtatMission);
					break;
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

int verifierPerteConnexion() {
	tentatives ++;
        
        if (tentatives >= 3) {
        	//arrêter tout ce qui est en rapport avec le robot
        	//TODO
        	return 0;
        }      
        return 1;
}



