#include "fonctions.h"
#include <math.h>

#define TODEGRE *180.0/3.14
#define DELTA_D 40
#define DELTA_A 5
#define LARGEUR 770
#define LONGUEUR 560


int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);
int verifierPerteConnexion(int status);
void connexionMoniteurPerdue();

void camera (void * arg){
	// On déclare toutes les variables nécessaires à l'acquisition de l'image 
	DCamera *camera;
	DImage *image;
	DJpegimage *jpegimg;
	DMessage *message;
	
	int status;
	int etat;
	
	// déclaration d'un état de sauvegarde dans le cas où on a un problème (failure)
	int backupEtat = ACTION_STOP_COMPUTE_POSITION;
	
	// Instruction de contrôle (visible sur terminal)
	rt_printf("-------------------- EXECUTION INITIALE THREAD CAMERA -------------------- \n");
	
	//Périodicité du thread : 600 ms 
	rt_task_set_periodic(NULL,TM_NOW,600000000);
	
	// création de la caméra
	camera = d_new_camera();
	// Fonction de la structure équivalent à : camera -> d_camera_open(camera)
	camera -> open(camera);

	
	//Création de la routine d'Acquisition d'image 
	
	while(1){
		
		// Wait null (pour éviter la famine des threads moins prioritaires)
		rt_task_wait_period(NULL);
		
		// Récupération de l'état de la communication
		rt_mutex_acquire(&mutexEtat,TM_INFINITE);
		status = etatCommMoniteur;
		rt_mutex_release(&mutexEtat);
		
		if (status == STATUS_OK){
			
			//Si communication OK, Récupération de l'état de la caméra
			rt_mutex_acquire(&mutexCam,TM_INFINITE);
			etat = etatCamera;
			rt_mutex_release(&mutexCam);
			
			switch(etat){
				
				// Si on ne demande pas le calcul de la position (ou si la caméra est dans l'état initial), on fait la routine de capture d'image
				case ACTION_STOP_COMPUTE_POSITION : 

					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- RECUPERATION IMAGE SANS CALCUL DE POSITION THREAD CAMERA -------------------- \n");
					
					// Création d'une nouvelle image
					image = d_new_image();
					// Capture de l'image via la caméra
					camera -> get_frame(camera , image);
					// Création d'une image de format compressé (compression JPEG)
					jpegimg = d_new_jpegimage();
					
					if (image != NULL){
					// Compression de l'image acquise précedemment
						jpegimg -> compress(jpegimg,image);
					}
					
					// Envoi de l'image compressée au serveur
					
					message = d_new_message();
					message->put_jpeg_image(message,jpegimg);
					
					
					// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
					if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {
					
					// Fonction de la structure équivalente à d_message_free
						message->free(message);
					}
					
					// Routine de fin de traitement					
					backupEtat = etat ;
					if (jpegimg != NULL){
						jpegimg->free(jpegimg);
					}
					if (image != NULL){
						image->free(image);
					}

					break;
				
				
				// Si on demande le calcul de la position (fonctionnement usuel), on fait la même routine, plus le calcul de la position
				case ACTION_COMPUTE_CONTINUOUSLY_POSITION : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- RECUPERATION IMAGE AVEC CALCUL DE POSITION THREAD CAMERA -------------------- \n");
					
					// Création d'une nouvelle image
					image = d_new_image();
					
					// Capture de l'image via la caméra
					camera -> get_frame(camera , image);					
					//Récupération de la position
					rt_mutex_acquire(&mutexPosition, TM_INFINITE);					
					rt_mutex_acquire(&mutexArene, TM_INFINITE);					
					position = image->compute_robot_position(image, arene);					
					rt_mutex_release(&mutexArene);					
					if (position != NULL){
						// Si une position est effectivement calculée, on la dessine sur l'image
						d_imageshop_draw_position(image,position);
					
						// Envoi de la position au serveur
						message = d_new_message();
						message->put_position(message,position);
					
						// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
						if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {
					
						// Fonction de la structure équivalente à d_message_free
							//message->free(message);
						
						}
					}
					else{
						printf("------------------ POSITION TROP NULLE --------- \n");
					}
					
					rt_mutex_release(&mutexPosition);
					
					// Création d'une image de format compressé (compression JPEG)
					jpegimg = d_new_jpegimage();
					
					if (image != NULL){
					// Compression de l'image acquise précedemment
						jpegimg -> compress(jpegimg,image);
					}
					// Envoi de l'image compressée au serveur
					
					message = d_new_message();
					message->put_jpeg_image(message,jpegimg);
					
					
					// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
					if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {
					
					// Fonction de la structure équivalente à d_message_free
						//message->free(message);
					}
					
					// Routine de fin de traitement					
					backupEtat = etat ;
					if (jpegimg != NULL){
						jpegimg->free(jpegimg);
					}
					if (image != NULL){
						image->free(image);
					}
					
				break;
				
				// Demande de détection d'arène (ne devrait arriver qu'une fois à l'initialisation du système)	
				case ACTION_FIND_ARENA : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- DEMANDE DETECTION ARENE THREAD CAMERA -------------------- \n");
					
					// Création d'une nouvelle image
					image = d_new_image();
					
					// Capture de l'image via la caméra
					camera -> get_frame(camera , image);
					
					// Récupération de la position de l'arène 
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					arene = image->compute_arena_position(image);
					
					// Si une arène est effectivement détectée, on la dessine sur l'image 					
					if (arene != NULL){
						d_imageshop_draw_arena(image,arene);
					}
					rt_mutex_release(&mutexArene);
					
					// Création d'une image de format compressé (compression JPEG)
					jpegimg = d_new_jpegimage();
					
					if (image != NULL){
					// Compression de l'image acquise précedemment
						jpegimg -> compress(jpegimg,image);
					}
					
					// Envoi de l'image compressée au serveur
					
					message = d_new_message();
					message->put_jpeg_image(message,jpegimg);
					
					
					// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
					if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {
					
					// Fonction de la structure équivalente à d_message_free
						//message->free(message);
					}
					
					// Modification de l'étatCamera sur le dernier état de fonctionnement normal connu
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = backupEtat;
					rt_mutex_release(&mutexCam);
					
					
					// Routine de fin de traitement					
					if (jpegimg != NULL){
						jpegimg->free(jpegimg);
					}
					if (image != NULL){
						image->free(image);
					}
					
				break;
				
				// Echec de la recherche d'arène
				case ACTION_ARENA_FAILED : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- ECHEC DETECTION ARENE THREAD CAMERA -------------------- \n");
						
					// Nettoyage de la structure arène
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					//arene->free(arene);
					rt_mutex_release(&mutexArene);
					
					// Modification de l'étatCamera sur le dernier état de fonctionnement normal connu
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = backupEtat;
					rt_mutex_release(&mutexCam);
				break;
				
				// Succès de la recherche d'arène
				case ACTION_ARENA_IS_FOUND :
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- SUCCES DETECTION ARENE THREAD CAMERA -------------------- \n");
					
					// Modification de l'étatCamera sur le dernier état de fonctionnement normal connu
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = backupEtat;
					rt_mutex_release(&mutexCam);
				break;
				
			}
		}
	}
}
					
					

void envoyer(void * arg) {
    DMessage *msg;
    int err;
    int status;
	int size;
	
	rt_printf("tenvoyer : Debut de l'exécution de tenvoyer\n");

    while (1) {
    	// Attente d'un message     
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            
            //Récupération de l'état de la communication
            rt_mutex_acquire(&mutexEtat, TM_INFINITE);
			status = etatCommMoniteur;
			rt_mutex_release(&mutexEtat);
			
			if (status == STATUS_OK){	
				rt_mutex_acquire(&mutexServeur, TM_INFINITE);		
				size = serveur-> send(serveur, msg);		//envoi du message
				rt_mutex_release(&mutexServeur);									
				if (size <= 0){	//= communication perdue
					connexionMoniteurPerdue();					
				}
				// Free du message						        
		        msg->free(msg);									
       		} 
        	else {	//en cas d'erreur d'envoi
            	rt_printf("Error msg queue write: %s\n", strerror(-err));
            }
        }
    }
}

void connecter(void * arg) {
    // status initial de la communication : != STATUS_OK
    int status = 1;
    DMessage *message;

    rt_printf("tconnect : Debut de l'exécution de tconnect\n");

    while (1) {
        
        // Attente connection robot
        rt_printf("tconnect : Attente du sémaphore semConnecterRobot\n");
     	rt_sem_p(&semConnecterRobot, TM_INFINITE);
        
		// Recuperation de l'etat de la communication
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);
        
        // Si la connexion n'est pas encore faite, on la fait
        if ((status != STATUS_OK) && (status != STATUS_ERR_CHECKSUM)){
        	// Instruction de controle
        	rt_printf("-------------------- TENTATIVE OUVERTURE ROBOT THREAD CONNECTER -------------------- \n");
        	
        	// Ouverture du robot 
        	rt_mutex_acquire(&mutexRobot, TM_INFINITE);
        	status = robot -> open_device(robot);
        	rt_mutex_release(&mutexRobot);
        	
        	// Si l'ouverture a été faite, on démarre le robot
        	if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
        		// Instruction de controle
        		rt_printf("-------------------- TENTATIVE DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");
        		do
        		{
        		// Demarrage du robot
		    		rt_mutex_acquire(&mutexRobot, TM_INFINITE);
		    		status = robot -> start_insecurely(robot);
		    		rt_mutex_release(&mutexRobot);
        		}
        		while ((status != STATUS_OK) && (status != STATUS_ERR_CHECKSUM));
        		// Si le robot a démarré, on le renseigne a etatCommRobot et on lance le watchdog
        		rt_sem_v(&semDeplacer);
        		if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
        			// Instruction de controle
        			rt_printf("-------------------- SUCCES DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");
        			
        			// MaJ de etatCommRobot
        			rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        			etatCommRobot = status;
        			rt_mutex_release(&mutexEtat);
        			
        			rt_mutex_acquire(&mutexErreur, TM_INFINITE);
        			tentatives = 0;
        			rt_mutex_release(&mutexErreur);
        			
        			
        			// Lancement du watchdog
        			rt_sem_v(&semWatchdog);
        			// Lancement de la batterie 
        			rt_sem_v(&semBatterie);
        		}
        		// Si ce status n'est pas OK, c'est le démarrage du robot qui a echoué 
        		else {
        			// Instruction de controle 
        			rt_printf("-------------------- ECHEC DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");
        			rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        			etatCommRobot = status;
        			rt_mutex_release(&mutexEtat);
				
					
// Arret du robot et fermeture de la communication 
					rt_mutex_acquire(&mutexRobot, TM_INFINITE);
					robot -> stop(robot);
					robot -> close_com(robot);
					rt_mutex_release(&mutexRobot);

				}
			}
			// Si ce status n'est pas OK, c'est l'ouverture du robot qui a echoué
			else {
				// Instruction de controle
				rt_printf("-------------------- ECHEC OUVERTURE ROBOT THREAD CONNECTER -------------------- \n");
				
				// Fermeture de la communication
				rt_mutex_acquire(&mutexRobot, TM_INFINITE);
				robot -> close_com(robot);
				rt_mutex_release(&mutexRobot);
//				status = 1;

			}
			
			// Création et envoi d'un message renseignant l'état de la communication
			message = d_new_message();
			message -> put_state(message,status);
			if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0){
				message -> free(message);
			}
		}
		// Si ce status n'est pas OK, la connection est déjà établie
		else{
			rt_printf("-------------------- ROBOT DEJA CONNECTE THREAD CONNECTER -------------------- \n");
		}
	}
}
			
void communiquer(void *arg) {
	DMessage *msg = d_new_message();
	int byteSize = 1;   // Var qui represente le nombre d'octets recus
	int num_msg = 0;
	//représente l'état occupé ou non du robot
	int again = 1, status;
	
	rt_printf("tserver : Début de l'exécution de serveur\n");
	
	while(1){
		// Ouverture du serveur
		rt_mutex_acquire(&mutexServeur, TM_INFINITE);
    	status = serveur->open(serveur, "8000");
    	rt_mutex_release(&mutexServeur);
    	
    	// Si le statAborted (core dumped)
    	if (status == 0){
    		// Instruction de controle 
    		rt_printf("-------------------- SERVEUR OUVERT THREAD COMMUNIQUER -------------------- \n");
    		
    		// MaJ de l'état de la communication avec le moniteur 
    		rt_mutex_acquire(&mutexEtat, TM_INFINITE);
    		etatCommMoniteur = STATUS_OK;
    		rt_mutex_release(&mutexEtat);
    		
    		// Tant que la taille des messages reçus est positive, on effectue la routine de communication
    		while (byteSize > 0){
    			// La nouvelle taille du message reçu est actualisée, et le serveur reçoit un message
//    			rt_mutex_acquire(&mutexServeur, TM_INFINITE);		
    			byteSize = serveur -> receive(serveur, msg);
//    			rt_mutex_release(&mutexServeur);
    			// Nouveau message reçu 
    			num_msg++;
    			
    			// Si la taille du nouveau message est positive
    			if (byteSize > 0){  				
    				// En fonction du message, on effectue le traitement approprié
    				rt_printf("TYPE MESSAGE RECU: %c\n", msg-> get_type(msg));
    				switch(msg-> get_type(msg)){
    					// Si action, on crée nouvelle action, et en fonction de celle-ci, traitement approprié
    					case MESSAGE_TYPE_ACTION : 
    						rt_printf("tserver : Le message %d reçu est une action\n", num_msg);
			    			DAction *action = d_new_action();
			    			action->from_message(action, msg);
			    			switch(action->get_order(action)){
			  					// Si on veut connecter le robot, on libère le sémaphore qui lance la routine de "connecter"
			  					case ACTION_CONNECT_ROBOT: 
			  			        	rt_printf("tserver : Action connecter robot\n");
			  			        	rt_sem_v(&semConnecterRobot);
			  			        	break;
			  			        // Si on veut chercher l'arène, on spécifie à "camera" cette action 
			  			        case ACTION_FIND_ARENA : 
			            			rt_printf("tserver : Action recherche arene\n");
			            			//MaJ de etatCamera
			            			rt_mutex_acquire(&mutexCam, TM_INFINITE);
			           				etatCamera = ACTION_FIND_ARENA;
			           				rt_mutex_release(&mutexCam);
			           				break;
			           			// Si on veut arreter la recherche de l'arène, on spécifie à "camera" cette action
			           			case ACTION_ARENA_FAILED : 	           					  			        	
						            rt_printf("tserver : Arret recherche arene\n");
						            // MaJ de etatCamera
			           				rt_mutex_acquire(&mutexCam, TM_INFINITE);
			           				etatCamera = ACTION_ARENA_FAILED;
			            			rt_mutex_release(&mutexCam);
			            			break;
			            		// Si l'arene est trouvée, on spécifie à "camera" cette action	
			            		case ACTION_ARENA_IS_FOUND : 
						            rt_printf("tserver : Action arene trouvée\n");
						            // MaJ de etatCamera
			           				rt_mutex_acquire(&mutexCam, TM_INFINITE);
			           				etatCamera = ACTION_ARENA_IS_FOUND;
		            			rt_mutex_release(&mutexCam);
			            			break;
			            		// Si on souhaite calculer continuellement la position du robot		
								case ACTION_COMPUTE_CONTINUOUSLY_POSITION:
									rt_printf("tserver : Calcul périodique position robot\n");
									// Mart_sleepj de etatCamera
									rt_mutex_acquire(&mutexCam, TM_INFINITE);
									etatCamera = ACTION_COMPUTE_CONTINUOUSLY_POSITION;
									rt_mutex_release(&mutexCam);
									break; 
			            		case ACTION_STOP_COMPUTE_POSITION : 
			            			rt_printf("tserver : Stop calcul périodique position robot\n");
			            			// MaJ de etatCamera
			            			rt_mutex_acquire(&mutexCam, TM_INFINITE);
			            			etatCamera = ACTION_STOP_COMPUTE_POSITION;
			            			rt_mutex_release(&mutexCam);
			            			break;
			            	}
			            	break;
			            // Si mouvement, on affecte le message à la variable mouvement         				
			    		case MESSAGE_TYPE_MOVEMENT : 	
    						rt_printf("tserver : Le message reçu %d est un mouvement\n", num_msg);
    						rt_mutex_acquire(&mutexMove, TM_INFINITE);
    						move->from_message(move, msg);
    						rt_mutex_release(&mutexMove);
    						break;
    					case MESSAGE_TYPE_MISSION : 
    						rt_mutex_acquire(&mutexMission, TM_INFINITE);
    						mission -> from_message(mission, msg);
    						rt_mutex_release(&mutexMission);
    						switch (mission->type){
    							case MISSION_TYPE_REACH_COORDINATE : 
    								//assignement d'une mission
									rt_mutex_acquire(&mutexEtatMission,TM_INFINITE);
									etatMission = START;
									rt_mutex_release(&mutexEtatMission);
									rt_sem_v(&semEffectuerMission);
			    					break;
    							case MISSION_TYPE_STOP :
    								rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
    								etatMission = TERMINATED;
    								rt_mutex_release(&mutexEtatMission);
    								break;
    						}
    						break;
    				}
    			}
    			//Si la taille du message reçu est < 0
    			else{
    				rt_printf("tserver : taille du message reçu <0 \n");
    				
    				// MaJ de etatCommMoniteur
    				rt_mutex_acquire(&mutexEtatMoniteur, TM_INFINITE);
    				etatCommMoniteur = 1;
    				rt_mutex_release(&mutexEtatMoniteur);
    				// Fermeture du serveur
    				rt_mutex_acquire(&mutexServeur, TM_INFINITE);
    				serveur->close(serveur);
    				rt_mutex_release(&mutexServeur);
    			}
    		}
    	}
    	// Si le status n'était pas OK, problème d'ouverture du serveur
    	else{
    		rt_printf("tserveur : Erreur ouverture serveur \n");
    	}
    }
}  					

void etat_batterie(void *arg) {
    int niveau = -1;  
    int status ;  
    DMessage * message;
    DBattery * batterie;
    rt_sem_p(&semBatterie, TM_INFINITE);
    rt_printf("tbatterie : Debut de l'éxecution periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
   
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
	
		if (status == STATUS_OK){
			rt_mutex_acquire(&mutexRobot, TM_INFINITE);
			status = robot->get_vbat(robot, &niveau);   // Renvoie 0, 1 ou 2 (= etat de la batterie)
			rt_mutex_release(&mutexRobot);
			
			if (status == STATUS_OK){
			
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
		    verifierPerteConnexion(status);
		}
		if (batterie != NULL){
		batterie->free(batterie);
		}
     }
} 

void deplacer(void *arg) {   
    int status = 1;
    int gauche;
    int droite;
    DMessage *message;
    rt_printf("tmove : Debut de l'éxecution de periodique à 200 ms\n");
    rt_task_set_periodic(NULL, TM_NOW, 200000000);

	rt_sem_p(&semDeplacer,TM_INFINITE);
    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        rt_printf("tmove : Activation périodique\n");

        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);
		// Si la communication avec le robot est OK, on modifie les caractéristiques du moteur en fonction du message et de la vitesse
        if (status == STATUS_OK) {
            rt_mutex_acquire(&mutexMove, TM_INFINITE);
            switch (move->get_direction(move)) {
                case DIRECTION_FORWARD:
	            	// Les roues sont lentes si le curseur est à moins de 80%
	            	if(move->get_speed(move)<=80){
		                gauche = MOTEUR_ARRIERE_LENT;
		                droite = MOTEUR_ARRIERE_LENT;
		            }
		            else{
		            	gauche=MOTEUR_ARRIERE_RAPIDE;
		            	droite=MOTEUR_ARRIERE_RAPIDE;
		            }
                    break;
                case DIRECTION_LEFT:
                	if(move->get_speed(move)<=80){
	                    gauche = MOTEUR_ARRIERE_LENT;
    	                droite = MOTEUR_AVANT_LENT;
    	            }
    	            else{
    	            	gauche = MOTEUR_ARRIERE_RAPIDE;
    	            	droite = MOTEUR_AVANT_RAPIDE;
    	            }
                    break;
                case DIRECTION_RIGHT:
                	if(move->get_speed(move)<=80){
	                    gauche = MOTEUR_AVANT_LENT;
    	                droite = MOTEUR_ARRIERE_LENT;
    	                }
    	            else{
    	            	gauche = MOTEUR_AVANT_RAPIDE;
    	            	droite = MOTEUR_ARRIERE_RAPIDE;
    	            }
                    break;
                case DIRECTION_STOP:
                    gauche = MOTEUR_STOP;
                    droite = MOTEUR_STOP;
                    break;
                case DIRECTION_STRAIGHT:
                	if(move->get_speed(move)<=80){
	                    gauche = MOTEUR_AVANT_LENT;
    	                droite = MOTEUR_AVANT_LENT;
    	                }
	                else{
	                	gauche = MOTEUR_AVANT_RAPIDE;
	                	droite = MOTEUR_AVANT_RAPIDE;
	                }
                    break;
            }
            rt_mutex_release(&mutexMove);
			
			// On applique les modifications et on récupère le résultat dans status
            rt_mutex_acquire(&mutexRobot , TM_INFINITE);
            status = robot->set_motors(robot, gauche, droite);
			rt_mutex_release(&mutexRobot);
            if (status != STATUS_OK){
                rt_mutex_acquire(&mutexEtat, TM_INFINITE);
                etatCommRobot = status;
                rt_mutex_release(&mutexEtat);
                
                verifierPerteConnexion(status);
          
            }
        }
    }
}

void watchdog(void * arg) {
	int status;
	int connectionRobotOk = 0;
	// Instruction de controle
	rt_printf("-------------------- INITIALISATION WATCHDOG -------------------- \n");
	
	// Routine du watchdog
	while(1){
		// Attente que la connexion avec le robot soit faite
		rt_sem_p(&semWatchdog, TM_INFINITE);
		
		// Si le sémaphore est passé, alors la connection avec le robot a été faite
		connectionRobotOk = 1;
		
		// Periode du watchdog : 1s
		rt_task_set_periodic(NULL, TM_NOW, 1000000000);
		
		// Tant que la connection avec le robot est OK : routine de reload du watchdog
		while (connectionRobotOk){
			rt_task_wait_period(NULL);
			// Récupération de l'état de communication du robot (qui peut avoir été perdue entre-temps)
			rt_mutex_acquire(&mutexEtat, TM_INFINITE);
			status = etatCommRobot;
			rt_mutex_release(&mutexEtat);
		
			// Si la communication est toujours opérationnelle, on reload le watchdog du robot
			if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
				rt_mutex_acquire(&mutexRobot, TM_INFINITE);
				status = robot -> reload_wdt(robot);
				rt_printf("\n\n\n\n\nRELOAD WATCHDOG : status : %d \n\n\n\n\n\n", status);
				rt_mutex_release(&mutexRobot);
						
				verifierPerteConnexion(status);
				
			}
			// Si la communication n'est plus opérationnelle, on sort de la boucle while
			else{
				rt_printf("\n\nPERTE DE CONNEXION!!!!\n\n");
				if ((status != STATUS_OK) || (status != STATUS_ERR_CHECKSUM)){
					connectionRobotOk = 0;
				}
				
				verifierPerteConnexion(status);
				rt_task_set_periodic(NULL,TM_NOW, TM_INFINITE);
				
			}
		}
	}
}	


void mission_reach_coordinates(void * arg) {

	DPosition * destination = d_new_position();
	DMessage * msg;
	float o_robot, x_robot, y_robot, x_destination, y_destination, dx, dy, distance, angle, h_arene, w_arene;
	etats_mission etatMissionLocal = START;
	int busy, again, sens;
	int status;
	while(1) {
		//attente d'une mission
		rt_printf("tmission : Attente du sémaphore semEffectuerMission\n");
		rt_sem_p(&semEffectuerMission, TM_INFINITE);
		
		rt_printf("-------------------- EXECUTION INITIALE D'UNE MISSION -------------------- \n");
		
		//Périodicité du thread : 600 ms --> permet de se caller sur la réactualisation de la position du robot
		rt_task_set_periodic(NULL,TM_NOW,600000000);
		etatMissionLocal = START;
		while (etatMissionLocal != NONE) {
		    /* Attente de l'activation périodique */
        	rt_task_wait_period(NULL);
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
					rt_mutex_acquire(&mutexRobot, TM_INFINITE);
					status = robot->set_motors(robot, MOTEUR_STOP, MOTEUR_STOP);
					rt_mutex_release(&mutexRobot);
					verifierPerteConnexion(status);
					printf("------------- ON A PASSE LE SET MOTEURS --------------------------- \n");
					x_destination = d_position_get_x(destination);
					y_destination = d_position_get_y(destination);
					
					printf("----------- Avant dimensions arene ------------------------ \n");
					//récupérer les dimensions de l'arene
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					h_arene = arene -> get_height(arene);
					w_arene = arene -> get_width(arene);
					rt_mutex_release(&mutexArene);
					
					printf("--------------- ON A PASSE LES DIMENSIONS DE L'ARENE ---------------- \n");
					
					//mettre la caméra dans un état de calcul de position du robot
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = ACTION_COMPUTE_CONTINUOUSLY_POSITION;
					rt_mutex_release(&mutexCam);
					
					printf(" --------------------- ON A PASSE LE CHANGEMENT D'ETAT CAMERA ------------ \n");
					//on change l'état de la mission à PENDING
					rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					if (etatMission != TERMINATED){
						etatMission = PENDING;
					}
					rt_mutex_release(&mutexEtatMission);
					break;
				
				case PENDING : //mission en cours à effectuer
					//récupération de la position
					
					printf("-------------- ON EST CASE PENDING ------------ \n");
					rt_mutex_acquire(&mutexPosition,TM_INFINITE);
					if(position != NULL){
						x_robot = d_position_get_x(position);
						y_robot = d_position_get_y(position);
						o_robot = d_position_get_orientation(position);
					}
					rt_mutex_release(&mutexPosition);
					
					printf("------------- ON A PASSE L'ACQUISITION DE POSITION ----------- \n");
				
					//calcul de la distance à parcourir
					dx = (x_robot - x_destination) * LARGEUR/w_arene;
					dy = (y_robot - y_destination) * LONGUEUR/h_arene;
					distance = sqrt(dx*dx + dy*dy);
					//on vérifie si le robot est arrivé à destination
					if (distance <= DELTA_D) {
						//mission accomplie
						//on change l'état de la mission à TERMINATED
						rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					    	etatMission = TERMINATED;
						rt_mutex_release(&mutexEtatMission);
					} else {
						//mission non accomplie
						//le robot va devoir bouger (d'abord tourner puis se déplacer)
						
						
						//calcul de l'angle selon lequel le robot doit tourner
						angle = o_robot TODEGRE + atan2(-dy,dx) TODEGRE  -180.0 ;
						
						//angle = d_position_get_orientation(position)*180.0/3.14 + atan2(-dy,dx)*180.0/3.14 -180.0;
						//si l'angle est inférieur à un delta donné
						//on considère que le robot n'a pas besoin de tourner
						rt_mutex_acquire(&mutexRobot, TM_INFINITE);
						status = robot->set_motors(robot, MOTEUR_STOP, MOTEUR_STOP);
						rt_mutex_release(&mutexRobot);
						verifierPerteConnexion(status);
						rt_mutex_acquire(&mutexRobot,TM_INFINITE);
						status = d_robot_is_busy(robot, &busy);
						rt_mutex_release(&mutexRobot);
						verifierPerteConnexion(status);
						
						if (angle > 180)
							angle = angle - 360;
						if (angle < - 180)
							angle = angle + 360;
							
							printf("ANGLE : %f \n", angle);
							//envoi du message au robot
							busy = 1;
							//on attend que le robot ait fini sa tâche
							while (busy == 1) {
								//GESTION DE LA PERTE DE CONNEXION
								again = 0;
								while(!again) {
									rt_mutex_acquire(&mutexRobot, TM_INFINITE);
									status = d_robot_is_busy(robot, &busy);
									rt_mutex_release(&mutexRobot);
									again = verifierPerteConnexion(status);
								}
							}
							//le robot a fini sa tâche
							//envoi de l'ordre de rotation
							again = 0;
							while (!again) {
								busy = 1;
								
								if(fabs(angle) > DELTA_A) {								
									if (angle >= 0){
										do{
								
											rt_mutex_acquire(&mutexRobot, TM_INFINITE);
											status = d_robot_turn(robot, angle, HORAIRE);
											rt_mutex_release(&mutexRobot);
											rt_printf("---------- TURN1 -----------\n");
										}
										while(status !=0);
									}
									else{
										do{
											rt_mutex_acquire(&mutexRobot, TM_INFINITE);
											//status = d_robot_turn(robot, fabs(angle), ANTI_HORAIRE);
											status = d_robot_turn(robot, -angle, ANTI_HORAIRE);
											rt_mutex_release(&mutexRobot);
											rt_printf("---------- TURN2 -----------\n");											
										}
										while (status !=0); 
									}
								}
								do{
									status = d_robot_is_busy(robot, &busy);
								}								
								while (busy != 0);
								//sleep(1);
								do{
									rt_mutex_acquire(&mutexRobot, TM_INFINITE);
									status = d_robot_move(robot, distance/2);
									rt_mutex_release(&mutexRobot);
									rt_printf("---------- MOVE -----------\n");									
								}
								while (status != 0);
								
								do{
									status = d_robot_is_busy(robot, &busy);
								}								
								while (busy != 0);								
								again = verifierPerteConnexion(status);
								printf("---------------- AGAIN TURN : %d--------\n",again);
							}
							//rotation effectuée
						}    	
					break;
			
				case TERMINATED : //fin de la mission
					//envoi d'un message indiquant la fin de la mission					
					msg = d_new_message();
					d_message_mission_terminate(msg,d_mission_get_id(mission));
					if (write_in_queue(&queueMsgGUI, msg, sizeof (DMessage)) < 0){
						msg->free(msg);
					}
					do{
						rt_mutex_acquire(&mutexRobot, TM_INFINITE);
						status = d_robot_turn(robot, 180, ANTI_HORAIRE);
						rt_mutex_release(&mutexRobot);
					}
					while (status !=0); 					
					//on indique qu'aucune mission n'est en cours
					//TODO : améliorer en implémentant la liste de missions à effectuer
					rt_mutex_acquire(&mutexEtatMission,TM_INFINITE);
					etatMission = NONE;
					rt_mutex_release(&mutexEtatMission);
					break;
				default : rt_printf("pas de mission en attente\n");
			}
		}
		//fin de la mission
		//on enlève la périodicité du thread
		rt_task_set_periodic(NULL,TM_NOW,0);
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

/* Fonction appelee lorsqu'on a perdu la communication du moniteur */
void connexionMoniteurPerdue(){
	
	// MaJ de etatCommMoniteur
	rt_mutex_acquire(&mutexEtat, TM_INFINITE);
	etatCommMoniteur = 1;				
	rt_mutex_release(&mutexEtat);
	
	rt_sem_v(&semConnecterRobot);
					
}

int verifierPerteConnexion(int status) { 
	DMessage * message;
	
	printf("---------------------- STATUS VERIFIER PERTE CONNEXION ------------ \n", status);
	if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
		rt_mutex_acquire(&mutexErreur, TM_INFINITE);
		tentatives = 0; 
		rt_mutex_release(&mutexErreur);
	}
	else{
		rt_mutex_acquire(&mutexErreur,TM_INFINITE);
		tentatives++;
		rt_mutex_release(&mutexErreur);
		if (tentatives >= 10) { 
            message = d_new_message();			
			message->put_state(message, status);
            rt_printf("Verifier perte connexion : Envoi message\n");
            while (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            	message->free(message);
            }

			rt_printf("TROP DE PERTES DE CONNEXION : SHUTDOWN ! \n"); 
		return 0;
		}
	rt_sem_v(&semConnecterRobot); 
	}
	rt_mutex_release(&mutexErreur);
	rt_mutex_acquire(&mutexEtat,TM_INFINITE);
	etatCommRobot = status;
	rt_mutex_release(&mutexEtat);
	return 1; 
}

