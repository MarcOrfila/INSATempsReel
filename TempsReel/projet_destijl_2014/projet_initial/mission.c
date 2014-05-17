#include "mission.h"
#include <math.h>

#define TODEGRE *180/3.14

void mission_reach_coordinates(void * arg) {
	DPosition positionLocale;
	DPosition * destination = d_new_position();
	DMessage * msg;
	float x_robot, y_robot, x_destination, y_destination, dx, dy, distance, angle;
	//etats_mission etatMissionLocal;
	int busy, again, sens;
	
	while(1) {
		//attente d'une mission
		rt_printf("tmission : Attente du sémaphore semEffectuerMission\n");
		rt_sem_p(&semEffectuerMission, TM_INFINITE);
		
		//récupération de la destination
		rt_mutex_acquire(&mutexMission, TM_INFINITE);
		d_mission_get_position(mission, destination);
		rt_mutex_release(&mutexMission);
		x_destination = d_position_get_x(destination);
		y_destination = d_position_get_y(destination);
		
		//vérifier que la destination est bien dans l'arène
		if (0 <= x_destination && x_destination <=     
		// float(*get_height) (struct DArena *This); /*!< Appel de la fonction d_arena_get_height().*/
        //float(*get_width) (struct DArena *This); /*!< Appel de la fonction d_arena_get_width().*/
        )
		
		//récupération de la position
		//TODO : vérifier que le système est bien dans un état de calcul de position du robot
		//sinon l'y mettre
		rt_mutex_acquire(&mutexPosition,TM_INFINITE);
		positionLocale = *position;
		rt_mutex_release(&mutexPosition);
		x_robot = d_position_get_x(&positionLocale);
		y_robot = d_position_get_y(&positionLocale);
		
		//on vérifie si le robot est arrivé à destination
		dx = x_robot - x_destination;
		dy = y_robot - y_destination;
		
		if ((dx == 0) && (dy == 0)) {
			//mission accomplie
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
	}
}
