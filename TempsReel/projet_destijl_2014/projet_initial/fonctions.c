#include "fonctions.h"
#include <math.h>

#define TODEGRE *180/3.14


int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);
int verifierPerteConnexion();
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
					
					// Compression de l'image acquise précedemment
					jpegimg -> compress(jpegimg,image);
					
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
					jpegimg->free(jpegimg);
					image->free(image);
					

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
							message->free(message);
						
						}
					}
					
					rt_mutex_release(&mutexPosition);
					
					// Création d'une image de format compressé (compression JPEG)
					jpegimg = d_new_jpegimage();
					
					// Compression de l'image acquise précedemment
					jpegimg -> compress(jpegimg,image);
					
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
					jpegimg->free(jpegimg);
					image->free(image);
					
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
					
					// Compression de l'image acquise précedemment
					jpegimg -> compress(jpegimg,image);
					
					// Envoi de l'image compressée au serveur
					
					message = d_new_message();
					message->put_jpeg_image(message,jpegimg);
					
					
					// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
					if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {
					
					// Fonction de la structure équivalente à d_message_free
						message->free(message);
					}
					
					// Modification de l'étatCamera sur le dernier état de fonctionnement normal connu
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = backupEtat;
					rt_mutex_release(&mutexCam);
					
					
					// Routine de fin de traitement					
					jpegimg->free(jpegimg);
					image->free(image);
					
				break;
				
				// Echec de la recherche d'arène
				case ACTION_ARENA_FAILED : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- ECHEC DETECTION ARENE THREAD CAMERA -------------------- \n");
						
					// Nettoyage de la structure arène
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					arene->free(arene);
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
				rt_printf("tenvoyer : On va envoyer un message de type %c \n", msg->get_type(msg));			
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
        //etatCommRobot = status;
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
        		if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
        			// Instruction de controle
        			rt_printf("-------------------- SUCCES DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");
        			
        			// MaJ de etatCommRobot
        			rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        			etatCommRobot = status;
        			rt_mutex_release(&mutexEtat);
        			
        			// Lancement du watchdog
        			rt_sem_v(&semWatchdog);
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
//					status = 1;

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
	DMission * missionLocal = d_new_mission();
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
									// Maj de etatCamera
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
    						rt_printf("Test 1\n"); 
    						missionLocal -> from_message(missionLocal, msg);
    						rt_printf("Test 2\n");
    						switch (missionLocal->type){
    							case MISSION_TYPE_REACH_COORDINATE : 
    								//assignement d'une mission
			    		rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
			    		//on vérifie si une mission est déjà en cours
			    		rt_printf("Test 3\n");
			    		if (etatMission == NONE) {
			    			//aucune mission n'est en cours
			    			//on assigne la nouvelle mission
			    			rt_printf("Test 4\n");
			    			etatMission = START;
			    			rt_mutex_release(&mutexEtatMission);
			    			rt_mutex_acquire(&mutexMission, TM_INFINITE);
			    			rt_printf("AVANT SEFGFAULT?\n");
			    			mission = missionLocal;
			    			rt_printf("APRES SEFGFAULT?\n");
			    			rt_mutex_release(&mutexMission);
			    			rt_sem_v(&semEffectuerMission);
			    		} else {
			    			//une mission est déjà en cours
			    			//pour l'instant on ne prend pas en compte l'ordre de mission reçu
			    			//TODO ranger l'ordre de mission dans une liste de missions en attente
			    			rt_printf("Test 5\n");
			    			rt_mutex_release(&mutexEtatMission);
			    			missionLocal->free(missionLocal);
			    		}
			    		break;
    							case MISSION_TYPE_STOP :
    								rt_printf("Test 6\n"); 
    								rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
    								etatMission = TERMINATED;
    								rt_mutex_release(&mutexEtatMission);
    								rt_printf("Test 7\n");
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
    				rt_printf("Test 8\n");
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
	
	
/*	do {
		rt_mutex_acquire(&mutexServeur, TM_INFINITE);
    		status = serveur->open(serveur, "8000");
    		rt_mutex_release(&mutexServeur);
    	} while (status != STATUS_OK);
    	
    	//connexion établie avec le moniteur
	rt_printf("tserver : Connexion\n");
	// TODO  : connexion avec la camera
	
	rt_mutex_acquire(&mutexEtat, TM_INFINITE);
	etatCommMoniteur = STATUS_OK;
	rt_mutex_release(&mutexEtat);
	    
	while (byteSize > 0) {
		rt_printf("tserver : Attente d'un message\n");
		rt_mutex_acquire(&mutexServeur, TM_INFINITE);
		byteSize = serveur->receive(serveur, msg);
		rt_mutex_release(&mutexServeur);
		num_msg++;
		if (byteSize < 0) {
			//connexion avec le serveur perdue
			connexionMoniteurPerdue();
		} else {
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
							rt_mutex_acquire(&mutexErreur, TM_INFINITE);
			    			tentatives = 0;			      
			    			rt_mutex_release(&mutexErreur);
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
	}*/


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
	
	if (status == STATUS_OK){
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

    rt_printf("tmove : Debut de l'éxecution de periodique à 200 ms\n");
    rt_task_set_periodic(NULL, TM_NOW, 200000000);

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

                message = d_new_message();
                message->put_state(message, status);

                rt_printf("tmove : Envoi message\n");
                while (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
                    message->free(message);
                }
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
		
			// Récupération de l'état de communication du robot (qui peut avoir été perdue entre-temps)
			rt_mutex_acquire(&mutexEtat, TM_INFINITE);
			status = etatCommRobot;
			rt_mutex_release(&mutexEtat);
		
			// Si la communication est toujours opérationnelle, on reload le watchdog du robot
			if ((status == STATUS_OK) || (status == STATUS_ERR_CHECKSUM)){
				rt_task_wait_period(NULL);
				//TODO status=checksum;
				rt_mutex_acquire(&mutexRobot, TM_INFINITE);
				status = robot -> reload_wdt(robot);
				rt_printf("\n\n\n\n\nRELOAD WATCHDOG : status : %d \n\n\n\n\n\n", status);
				rt_mutex_release(&mutexRobot);
			
				// Si le reload n'a pas fonctionné, on vérifie la perte de connexion
				if ((status != STATUS_OK) || (status != STATUS_ERR_CHECKSUM)){
					connectionRobotOk = verifierPerteConnexion();
				}
				else{
					tentatives = 0;
				}
			}
			// Si la communication n'est plus opérationnelle, on sort de la boucle while
			else{
				rt_printf("\n\nPERTE DE CONNEXION!!!!\n\n");
				if ((status != STATUS_OK) || (status != STATUS_ERR_CHECKSUM)){
					connectionRobotOk = verifierPerteConnexion();
					if(!connectionRobotOk){
						rt_mutex_acquire(&mutexEtat,TM_INFINITE);
						etatCommRobot = 1;
						rt_mutex_release(&mutexEtat);
					}
				}
				else{
					tentatives = 0;
				}
				
				/*connectionRobotOk = 0;
				rt_mutex_acquire(&mutexEtat, TM_INFINITE);
				etatCommRobot = 1;
				rt_mutex_release(&mutexEtat);
				rt_sem_v(&semConnecterRobot);
				//rt_task_set_periodic(NULL,TM_NOW, TM_INFINITE);*/
				
			}
		}
	}
}	

void mission_reach_coordinates(void * arg) {

	DPosition * destination = d_new_position();
	DMessage * msg;
	DPosition positionLocale;
	float x_robot, y_robot, x_destination, y_destination, dx, dy, distance, angle, h_arene, w_arene;
	etats_mission etatMissionLocal = START;
	int busy, again, sens;
	rt_printf("Test 10\n");
	while(1) {
		rt_printf("Test 11\n");
		//attente d'une mission
		rt_printf("tmission : Attente du sémaphore semEffectuerMission\n");
		rt_sem_p(&semEffectuerMission, TM_INFINITE);
		etatMissionLocal = START;
		rt_printf("Test 12\n");
		while (etatMissionLocal != NONE) {
		rt_printf("Test 13\n");
			//récupération de l'état de la mission
			rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
			etatMissionLocal = etatMission;
			rt_mutex_release(&mutexEtatMission);
		
			switch(etatMissionLocal) {
		
				case 
				START : //début de la mission
					//récupération de la destination
					rt_printf("Test 14\n");
					rt_mutex_acquire(&mutexMission, TM_INFINITE);
					d_mission_get_position(mission, destination);
					rt_mutex_release(&mutexMission);
					rt_printf("Test 15\n");
					x_destination = d_position_get_x(destination);
					y_destination = d_position_get_y(destination);
					//vérifier que la destination est bien dans l'arène
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					h_arene = arene -> get_height(arene);
					w_arene = arene -> get_width(arene);
					rt_mutex_release(&mutexArene);
					rt_printf("Test 15\n");
					if (0 <= x_destination && x_destination <= w_arene
					    && 0 <= y_destination && y_destination <= h_arene) {
					    rt_printf("Test 16\n");
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
						rt_printf("Test 17\n");
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
					rt_printf("Test 18\n");
					rt_mutex_acquire(&mutexPosition,TM_INFINITE);
					positionLocale = *position;
					rt_mutex_release(&mutexPosition);
					x_robot = d_position_get_x(&positionLocale);
					y_robot = d_position_get_y(&positionLocale);
				
					//on vérifie si le robot est arrivé à destination
					dx = x_robot - x_destination;
					dy = y_robot - y_destination;
			rt_printf("Test 19\n");
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
					       			rt_mutex_acquire(&mutexErreur, TM_INFINITE);
					       			tentatives = 0;                
					       			rt_mutex_release(&mutexErreur);
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
							rt_mutex_acquire(&mutexErreur, TM_INFINITE);
							tentatives = 0;               
							rt_mutex_release(&mutexErreur);
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
								rt_mutex_acquire(&mutexErreur, TM_INFINITE);
								tentatives = 0;               
								rt_mutex_release(&mutexErreur);
			
							} 
							else {
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
							rt_mutex_acquire(&mutexErreur, TM_INFINITE);
							tentatives = 0;                 
							rt_mutex_release(&mutexErreur);
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
				default : rt_printf("pas de mission en attente\n");
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

/* Fonction appelee lorsqu'on a perdu la communication du moniteur */
void connexionMoniteurPerdue(){
	
	// MaJ de etatCommMoniteur
	rt_mutex_acquire(&mutexEtat, TM_INFINITE);
	etatCommMoniteur = 1;				
	rt_mutex_release(&mutexEtat);
					
	
	// Fermeture du serveur
	/*rt_mutex_acquire(&mutexServeur, TM_INFINITE);
	serveur->close(serveur);		
	rt_mutex_release(&mutexServeur);*/
}

int verifierPerteConnexion() { 
	rt_mutex_acquire(&mutexErreur, TM_INFINITE);
	tentatives ++; 
	rt_mutex_release(&mutexErreur);
	if (tentatives >= 3) { 
		rt_task_delete(&tmission); 
		rt_task_delete(&tbatterie); 
		rt_task_delete(&tconnect); 
		rt_task_delete(&tmove); 
		rt_task_delete(&twatchdog);
		rt_task_delete(&tServeur);
		rt_task_delete(&tenvoyer);
		rt_task_delete(&tcamera);
		rt_printf("PLUS DE 3 PERTES DE CONNEXION : SHUTDOWN ! \n"); 
		return 0; } 
	rt_sem_v(&semConnecterRobot);
	return 1; 
}

