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

#include "fonctions.h"
#include <math.h>


#define TODEGRE *180.0/3.14 // Passage de radian à degré
#define DELTA_D 20 // Marge acceptée pour la distance entre le robot et l'objectif à atteindre
#define DELTA_A 5 // Marge acceptée pour l'angle entre le robot et l'objectif à atteindre
#define LARGEUR 770 // Largeur de l'arène
#define LONGUEUR 560 // Longueur de l'arène


/**
* \fn int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) 
* \brief Fonction d'écriture d'un message dans une queue de messages
* \param RT_QUEUE *msgQueue : Queue de messages dans laquelle écrire
* \param void * data : contenu du message à envoyer
* \param int size : taille du message à envoyer
* \return Succès de l'écriture dans la queue de messages :
* 	< 0 (numéro de l'erreur) si échec
*	0 sinon
*/		
int write_in_queue(RT_QUEUE *msgQueue, void * data, int size);

/**
* \fn int verifierPerteConnexion(int status)
* \brief Fonction de vérification d'une perte de connexion, accepte 3 échecs consécutifs avant de déclarer une perte effective de la communication avec le robot 
* \param int status : Statut de la connexion avec le robot
* \return Statut de la connexion :
*	0 si la connexion est effectivement perdue (plus de 3 erreurs consécutives)
*	1 si la connexion n'est pas considérée comme perdue
*/
int verifierPerteConnexion(int status);

/** 
* \fn void connexionMoniteurPerdue();
* \brief Fonction de nettoyage lancée lors de la perte de communication avec le moniteur
*/
void connexionMoniteurPerdue();

/**
* \fn void camera (void * arg)
* \brief Fonction exécutée par le thread tCamera, permet la détection d'arène, le calcul de position et l'envoi d'images au moniteur
*/
void camera (void * arg){
	// Déclaration des variables nécessaires à l'acquisition de l'image 
	DCamera *camera;
	DImage *image;
	DJpegimage *jpegimg;
	DMessage *message;
	
	// Variable locale : Statut de la connection 
	int status;
	// Variable locale : Etat de la camera 
	int etat;
	
	// déclaration d'un état de sauvegarde
	int backupEtat = ACTION_STOP_COMPUTE_POSITION;
	
	// Instruction de contrôle (visible sur terminal)
	rt_printf("-------------------- EXECUTION INITIALE THREAD CAMERA -------------------- \n");
	
	//Périodicité du thread : 600 ms 
	rt_task_set_periodic(NULL,TM_NOW,600000000);
	
	// création de la caméra
	camera = d_new_camera();
	// Fonction de la structure équivalent à : camera -> d_camera_open(camera)
	camera -> open(camera);

	
	//Routine d'Acquisition d'image 
	
	while(1){
		
		// Wait null (pour éviter la famine des threads moins prioritaires)
		rt_task_wait_period(NULL);
		
		// Récupération de l'état de la communication
		rt_mutex_acquire(&mutexEtatMoniteur,TM_INFINITE);
		status = etatCommMoniteur;
		rt_mutex_release(&mutexEtatMoniteur);
		
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
					
					// Compression de l'image acquise (seulement si l'image n'est pas vide)					
					if (image != NULL){
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
					rt_mutex_acquire(&mutexArene, TM_INFINITE);	
					// Calcul de la position (si l'arène n'est pas vide)
					if (arene != NULL){			
						rt_mutex_acquire(&mutexPosition, TM_INFINITE);					
						position = image->compute_robot_position(image, arene);									
						if (position != NULL){
							// Si une position est effectivement calculée, on la dessine sur l'image
							d_imageshop_draw_position(image,position);
					
							// Envoi de la position au serveur
							message = d_new_message();
							message->put_position(message,position);
					
							// Si le write_in_queue ne fonctionne pas (retour d'err est inf à 0) , on efface le message
							if (write_in_queue(&queueMsgGUI, message, sizeof(DMessage)) < 0) {						
							}
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
					}
					rt_mutex_release(&mutexArene);						
				break;
				
				// Demande de détection d'arène
				case ACTION_FIND_ARENA : 
		        	
		        	// Création d'une nouvelle arène
		        	rt_mutex_acquire(&mutexArene, TM_INFINITE);
		        	arene = d_new_arena();
		        	rt_mutex_release(&mutexArene);				
					
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
						message->free(message);
					}
					
					// Retour de l'étatCamera sur le dernier état de fonctionnement avant la detection d'arène
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
					arene->free(arene);
					arene = NULL;
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
					
					// Modification de l'étatCamera sur le dernier état de fonctionnement avant l'abandon de la detection d'arène
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = backupEtat;
					rt_mutex_release(&mutexCam);
				break;
				
			}
		}
	}
}
					
					
/**
* \fn void envoyer(void * arg)
* \brief Fonction exécutée par le thread tEnvoyer, permet l'envoi d'un message du superviseur au moniteur 
*/
void envoyer(void * arg) {
    // Déclaration des variables nécessaires à l'envoi d'un message 
    DMessage *msg;
    
    // Erreur de lecture dans la queue de messages
    int err;
    // Etat de la communication avec le moniteur
    int status;
    // Nombre d'octets envoyés
	int size;
	
	rt_printf("tenvoyer : Debut de l'exécution de tenvoyer\n");

    while (1){
    	// Attente d'un message dans la queue de messages    
        if ((err = rt_queue_read(&queueMsgGUI, &msg, sizeof (DMessage), TM_INFINITE)) >= 0) {
            rt_printf("tenvoyer : envoi d'un message au moniteur\n");
            
            //Récupération de l'état de la communication
            rt_mutex_acquire(&mutexEtatMoniteur, TM_INFINITE);
			status = etatCommMoniteur;
			rt_mutex_release(&mutexEtatMoniteur);
			
			// Si la communication avec le moniteur est OK, on envoie le message
			if (status == STATUS_OK){	
				// Envoi du message 
				rt_mutex_acquire(&mutexServeur, TM_INFINITE);		
				size = serveur-> send(serveur, msg);
				rt_mutex_release(&mutexServeur);									
				// Si le nombre d'octets envoyés est < 0, la connection avec le moniteur a été perdue
				if (size <= 0){	
					// Nettoyage de la structure
					connexionMoniteurPerdue();					
				}
				// Free du message						        
		        msg->free(msg);									
       		}
       	}
       	// Erreur de lecture de la queue de messages
        else {	
            rt_printf("Error msg queue write: %s\n", strerror(-err)); 
    	}
	}
}


/**
* \fn void connecter(void * arg)
* \brief Fonction exécutée par le thread tConnecter, effectue la procédure de connexion/reconnexion avec le robot 
*/
void connecter(void * arg) {
    // status initial de la communication avec le robot : Status not ok (différent de 0)
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
        if (status != STATUS_OK){
        	// Instruction de controle
        	rt_printf("-------------------- TENTATIVE OUVERTURE ROBOT THREAD CONNECTER -------------------- \n");
        	
        	// Ouverture du robot 
        	rt_mutex_acquire(&mutexRobot, TM_INFINITE);
        	status = robot -> open_device(robot);
        	rt_mutex_release(&mutexRobot);
        	
        	// Mise à jour de la perte/maintien de la connexion avec le robot
        	verifierPerteConnexion(status);
        	
        	// Si l'ouverture a été faite, on démarre le robot
        	if (status == STATUS_OK){
        		// Instruction de controle
        		rt_printf("-------------------- TENTATIVE DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");
        		// Tentative de démarrage du robot infinie (ne s'arrête que si le robot est connecté)
        		do{
		    		rt_mutex_acquire(&mutexRobot, TM_INFINITE);
		    		// FIXME : Possibilité d'utiliser status = robot -> start(robot) pour une plus grande sûreté de la connection
		    		// Le cas échéant, ne pas oublier de lancer le thread watchdog dans main.c
		    		// HACK : Pour une meilleure réactivité du système en "insecurely", commenter le déclenchement de watchdog dans main.c  
		    		status = robot -> start_insecurely(robot);	    		
		    		rt_mutex_release(&mutexRobot);
        		}
        		while (status != STATUS_OK);
        		// Si le robot a démarré, mise à jour de la perte/maintien de la connexion avec le robot et lancement des exécutions des threads communiquant avec le robot
        		verifierPerteConnexion(status);
        		rt_sem_v(&semDeplacer);
        		rt_sem_v(&semWatchdog);
        		rt_sem_v(&semBatterie);        		
    			// Instruction de controle
    			rt_printf("-------------------- SUCCES DEMARRAGE ROBOT THREAD CONNECTER -------------------- \n");    		
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

/**
* \fn void communiquer(void * arg)
* \brief Fonction exécutée par le thread tServeur, sert d'interface entre les messages envoyés via le moniteur, et le superviseur 
*/			
void communiquer(void *arg) {
	// Déclaration des variables nécessaires à la réception et au traitement des messages
	DMessage *msg = d_new_message();
	// variable contenant la taille des messages reçus
	int byteSize = 1;
	// compteur du numéro du message reçu 
	int num_msg = 0;
	// Statut de l'ouverture du serveur
	int status;
	
	rt_printf("tserver : Début de l'exécution de serveur\n");
	
	while(1){
		// Ouverture du serveur
		rt_mutex_acquire(&mutexServeur, TM_INFINITE);
    	status = serveur->open(serveur, "8000");
    	rt_mutex_release(&mutexServeur);
    	
    	// Si l'ouverture du serveur est réussie
    	if (status == 0){
    		// Instruction de controle 
    		rt_printf("-------------------- SERVEUR OUVERT THREAD COMMUNIQUER -------------------- \n");
    		
    		// MaJ de l'état de la communication avec le moniteur 
    		rt_mutex_acquire(&mutexEtatMoniteur, TM_INFINITE);
    		etatCommMoniteur = STATUS_OK;
    		rt_mutex_release(&mutexEtatMoniteur);
    		
    		// Tant que la taille des messages reçus est positive, on effectue la routine de communication
    		while (byteSize > 0){
    			// La nouvelle taille du message reçu est actualisée, et le serveur reçoit un message
    			// HACK : pas de mutex autour de serveur->receive pour éviter interblocage entre tenvoyer (serveur -> send) et tServeur. De plus, receive est en attente bloquante.		
    			byteSize = serveur -> receive(serveur, msg);
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
								// Si on ne souhaite pas calculer la position du robot
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
    					// Si Mission
    					case MESSAGE_TYPE_MISSION : 
    						rt_mutex_acquire(&mutexMission, TM_INFINITE);
    						mission -> from_message(mission, msg);
    						rt_mutex_release(&mutexMission);
    						switch (mission->type){
								//assignement d'une mission    						
    							case MISSION_TYPE_REACH_COORDINATE : 
									rt_mutex_acquire(&mutexEtatMission,TM_INFINITE);
									etatMission = START;
									rt_mutex_release(&mutexEtatMission);
									rt_sem_v(&semEffectuerMission);
			    					break;
			    				// Arret de la mission 
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
    				// Perte de communication, arrêt du système
    				connexionMoniteurPerdue();
    			}
    		}
    	}
    	// Si le status n'était pas OK, problème d'ouverture du serveur
    	else{
    		rt_printf("tserveur : Erreur ouverture serveur \n");
    	}
    }
}  					

/**
* \fn void etat_batterie(void * arg)
* \brief Fonction exécutée par le thread tbatterie, envoie l'état de la batterie au moniteur
*/	
void etat_batterie(void *arg) {
	// Niveau initial de la batterie UNKNOWN
    int niveau = -1;  
    // Statut de la communication avec le robot
    int status ;  
    // Message à envoyer
    DMessage * message;
    // Déclaration de la batterie 
    DBattery * batterie;
    
    // Attente de la connexion du robot
    rt_sem_p(&semBatterie, TM_INFINITE);
    rt_printf("tbatterie : Debut de l'éxecution periodique à 1s\n");
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
   
	// Routine de récupération de l'état de la batterie  
   	while (1) {     	
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);     
        rt_printf("tbatterie : Activation périodique\n");
        
        // Création d'une nouvelle structure de batterie 
        batterie = d_new_battery();
        
        // Récupération de l'état de la communication avec le robot
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status=etatCommRobot;
        rt_mutex_release(&mutexEtat);
		
		// Si la communication avec robot est OK
		if (status == STATUS_OK){
			// Récupération du niveau de la batterie. 0 : LOW, 1 : MEDIUM, 2 : HIGH
			rt_mutex_acquire(&mutexRobot, TM_INFINITE);
			status = robot->get_vbat(robot, &niveau);   // Renvoie 0, 1 ou 2 (= etat de la batterie)
			rt_mutex_release(&mutexRobot);
			
			// Mise à jour globale de l'état de la communication avec le robot
			verifierPerteConnexion(status);
			
			// Si la récupération de la batterie est réussie
			if (status == STATUS_OK){
				// Remplissage de la structure Batterie
				batterie->set_level(batterie, niveau);
				
				// Envoi du message renseignant l'état de la batterie
				message = d_new_message();
	   	    	message->put_battery_level(message, batterie);
	   	    	rt_printf("tbatterie : Envoi message\n");
	   	    	
	   	    	if (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
		        	    message->free(message);	
	   	    	}
		    }
		}
		// Nettoyage de la structure batterie
		if (batterie != NULL){
		batterie->free(batterie);
		}
     }
} 

/**
* \fn void deplacer(void * arg)
* \brief Fonction exécutée par le thread tmove, permet le déplacement manuel du robot
*/	
void deplacer(void *arg) {   
    // Statut de la communication avec le robot (initialisation : STATUS NOT OK)
    int status = 1;
    // Etat de la roue gauche
    int gauche;
    // Etat de la roue droite
    int droite;
    // Etat de la mission locale
    etats_mission etatMissionLocal = NONE;

    
    rt_printf("tmove : Debut de l'éxecution de periodique à 200 ms\n");
    rt_task_set_periodic(NULL, TM_NOW, 200000000);
	// Attente de la connexion du robot
	rt_sem_p(&semDeplacer,TM_INFINITE);
    while (1) {
        /* Attente de l'activation périodique */
        rt_task_wait_period(NULL);
        rt_printf("tmove : Activation périodique\n");
		
		// Récupération de l'état de la communication avec le robot
        rt_mutex_acquire(&mutexEtat, TM_INFINITE);
        status = etatCommRobot;
        rt_mutex_release(&mutexEtat);
        
        // Récupération de l'état actuel de la mission 
        rt_mutex_acquire(&mutexEtatMission,TM_INFINITE);
        etatMissionLocal = etatMission;
        rt_mutex_release(&mutexEtatMission);
		// Si la communication avec le robot est OK, on modifie les caractéristiques du moteur en fonction du message et de la vitesse
        // Si une mission est en cours, les mouvements du robot ne doivent pas interférer
        if (status == STATUS_OK && etatMissionLocal != PENDING) {
            rt_mutex_acquire(&mutexMove, TM_INFINITE);
            switch (move->get_direction(move)) {
                // FIXME La lib fournie comporte un contresens car DIRECTION_FORWARD envoie le robot en arrière, confusion avec DIRECTION_STRAIGHT
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
                // Direction gauche : Roue gauche en arrière, roue droite en avant
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
                // Direction droite : Roue gauche en avant, roue droite en arrière
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
                // Stop : Roues gauche et droite stop
                case DIRECTION_STOP:
                    gauche = MOTEUR_STOP;
                    droite = MOTEUR_STOP;
                    break;
                // Direction en avant : Roues gauche et droite en avant
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
			
			verifierPerteConnexion(status);
        }
    }
}
/**
* \fn void watchdog(void * arg)
* \brief Fonction exécutée par le thread twatchdog, permet le rechargement du watchdog du robot (utile uniquement en start secure)
*/	
void watchdog(void * arg) {
	// Etat de la communication avec le robot
	int status;
	// Booleen pour la boucle de reload du watchdog
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
			if (status == STATUS_OK){
				rt_mutex_acquire(&mutexRobot, TM_INFINITE);
				status = robot -> reload_wdt(robot);
				rt_printf("\n\n\n\n\nRELOAD WATCHDOG : status : %d \n\n\n\n\n\n", status);
				rt_mutex_release(&mutexRobot);
				if (status == STATUS_ERR_CHECKSUM){
					status == STATUS_OK;
				}		
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

/**
* \fn void mission_reach_coordinates(void * arg)
* \brief Fonction exécutée par le thread tmission, permet au robot d'aller au point de l'arène indiquée par l'utilisateur
*/
void mission_reach_coordinates(void * arg) {
	// Position de la destination finale
	DPosition * destination = d_new_position();
	// Message à envoyer
	DMessage * msg;
	// Variables de position du robot, de la destination, de la distance, de l'angle et des dimensions de l'arène
	float o_robot, x_robot, y_robot, x_destination, y_destination, dx, dy, distance, angle, h_arene, w_arene;
	// Etat local de la mission, START à l'initialisation 
	etats_mission etatMissionLocal = START;
	// Variable à 1 si le robot est occupé, 0 si il ne fait rien
	int busy;
	// Variable qui détermine si une action doit être refaite 
	int again;
	// Etat de la communication avec le robot
	int status;
	while(1) {
		//attente d'une mission
		rt_printf("tmission : Attente du sémaphore semEffectuerMission\n");
		
		// Attente de l'envoi d'une mission 
		rt_sem_p(&semEffectuerMission, TM_INFINITE);
		
		rt_printf("-------------------- EXECUTION INITIALE D'UNE MISSION -------------------- \n");
		
		//Périodicité du thread : 600 ms --> permet de se caller sur la réactualisation de la position du robot
		rt_task_set_periodic(NULL,TM_NOW,600000000);
		etatMissionLocal = START;
		// Routine de mission 
		while (etatMissionLocal != NONE) {
		    /* Attente de l'activation périodique */
        	rt_task_wait_period(NULL);
			
			//récupération de l'état de la mission
			rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
			etatMissionLocal = etatMission;
			rt_mutex_release(&mutexEtatMission);
		
			switch(etatMissionLocal) {
				// Initialisation d'une mission 
				case START :
					//récupération de la destination
					rt_mutex_acquire(&mutexMission, TM_INFINITE);
					d_mission_get_position(mission, destination);
					rt_mutex_release(&mutexMission);
					// Arrêt du robot
					rt_mutex_acquire(&mutexRobot, TM_INFINITE);
					status = robot->set_motors(robot, MOTEUR_STOP, MOTEUR_STOP);
					rt_mutex_release(&mutexRobot);
					// Mise à jour de l'état de la communication avec le robot
					verifierPerteConnexion(status);
					// Affectation de la destination 
					x_destination = d_position_get_x(destination);
					y_destination = d_position_get_y(destination);
					
					//récupération des dimensions de l'arene
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					h_arene = arene -> get_height(arene);
					w_arene = arene -> get_width(arene);
					rt_mutex_release(&mutexArene);					
				
					//mettre la caméra dans un état de calcul de position du robot
					rt_mutex_acquire(&mutexCam, TM_INFINITE);
					etatCamera = ACTION_COMPUTE_CONTINUOUSLY_POSITION;
					rt_mutex_release(&mutexCam);
					
					//on change l'état de la mission à PENDING
					rt_mutex_acquire(&mutexEtatMission, TM_INFINITE);
					if (etatMission != TERMINATED){
						etatMission = PENDING;
					}
					rt_mutex_release(&mutexEtatMission);
					break;
				//mission non-terminée
				case PENDING : 
					//récupération de la position
					rt_mutex_acquire(&mutexPosition,TM_INFINITE);
					if(position != NULL){
						x_robot = d_position_get_x(position);
						y_robot = d_position_get_y(position);
						o_robot = d_position_get_orientation(position);
					}
					rt_mutex_release(&mutexPosition);					
				
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
						// L'angle à tourner c'est l'orientation, plus l'angle par rapport à l'objectif - 180.0 pour avoir le bon angle
						angle = o_robot TODEGRE + atan2(-dy,dx) TODEGRE  -180.0 ;						
						
						// Arret du robot 
						rt_mutex_acquire(&mutexRobot, TM_INFINITE);
						status = robot->set_motors(robot, MOTEUR_STOP, MOTEUR_STOP);
						rt_mutex_release(&mutexRobot);
						
						// Mise à jour de l'état de la communication 
						verifierPerteConnexion(status);
						
						// Changement de l'angle pour avoir un angle entre -180 et 180 degrés
						if (angle > 180){
							angle = angle - 360;
						}
						if (angle < - 180){
							angle = angle + 360;
						}							
						// On considère le robot occupé
						busy = 1;
						
						// Tant que le robot est occupé 
						while (busy == 1) {
							
							again = 0;
							// Tant que la communication avec le robot ne fonctionne pas 
							while(!again) {
								// Le robot est-il occupé ?
								rt_mutex_acquire(&mutexRobot, TM_INFINITE);
								status = d_robot_is_busy(robot, &busy);
								rt_mutex_release(&mutexRobot);
								// Mise à jour du statut du robot
								again = verifierPerteConnexion(status);
							}
						}
						//le robot a fini sa tâche
						//envoi de l'ordre tourner + avancer 
						again = 0;
						
						// Tant que la communication avec le robot n'a pas fonctionné
						while (!again) {
							busy = 1;
							
							// Si l'angle à tourner est trop faible, on ne tourne pas 
							if(fabs(angle) > DELTA_A) {								
								// Si l'angle est positif, on tourne de "Angle" dans le sens horaire
								if (angle >= 0){
									// On tente de tourner tant qu'on a pas réussi
									do{
							
										rt_mutex_acquire(&mutexRobot, TM_INFINITE);
										status = d_robot_turn(robot, angle, HORAIRE);
										rt_mutex_release(&mutexRobot);
										// Mise à jour de l'état de la communication avec le robot										
										verifierPerteConnexion(status);									
									}
									while(status !=0);
								}
								// Si l'angle est négatif, on tourne de (-angle) (ou |angle|) dans le sens anti-horaire
								else{
									// On tente de tourner tant qu'on a pas réussi								
									do{
										rt_mutex_acquire(&mutexRobot, TM_INFINITE);
										status = d_robot_turn(robot, -angle, ANTI_HORAIRE);
										rt_mutex_release(&mutexRobot);	
										// Mise à jour de l'état de la communication avec le robot
										verifierPerteConnexion(status);										
									}
									while (status !=0); 
								}
							}
							// On attend que le robot aie tourné du bon angle
							do{
								rt_mutex_acquire(&mutexRobot, TM_INFINITE);
								status = d_robot_is_busy(robot, &busy);
								rt_mutex_release(&mutexRobot);
								verifierPerteConnexion(status);
							}								
							while (busy != 0);
							// On parcourt la moitié de la distance entre le robot et l'objectif
							do{
								rt_mutex_acquire(&mutexRobot, TM_INFINITE);
								status = d_robot_move(robot, distance/2);
								rt_mutex_release(&mutexRobot);	
								verifierPerteConnexion(status);							
							}
							while (status != 0);
							
							// On attend que le robot aie avancé entièrement 
							do{
								rt_mutex_acquire(&mutexRobot,TM_INFINITE);
								status = d_robot_is_busy(robot, &busy);
								rt_mutex_release(&mutexRobot);
								verifierPerteConnexion(status);
							}								
							while (busy != 0);
							// On refait la boucle si on avait perdu la connection								
							again = verifierPerteConnexion(status);
						}
					}    	
				break;
				// Fin de la mission 
				case TERMINATED :
				
					//envoi d'un message indiquant la fin de la mission					
					msg = d_new_message();
					d_message_mission_terminate(msg,d_mission_get_id(mission));
					// Si le write_in_queue échoue, on efface la structure du message 
					if (write_in_queue(&queueMsgGUI, msg, sizeof (DMessage)) < 0){
						msg->free(msg);
					}
					// Demi-tour du robot pour indiquer la fin de mission et éviter de se retrouver face à un mur
					do{
						rt_mutex_acquire(&mutexRobot, TM_INFINITE);
						status = d_robot_turn(robot, 180, ANTI_HORAIRE);
						rt_mutex_release(&mutexRobot);
						verifierPerteConnexion(status);
					}
					while (status !=0); 					
					// L'état de la mission devient NONE, car plus de mission
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

/**
* \fn int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) 
* \brief Fonction d'écriture d'un message dans une queue de messages
* \param RT_QUEUE *msgQueue : Queue de messages dans laquelle écrire
* \param void * data : contenu du message à envoyer
* \param int size : taille du message à envoyer
* \return Succès de l'écriture dans la queue de messages :
* 	< 0 (numéro de l'erreur) si échec
*	0 sinon
*/	
int write_in_queue(RT_QUEUE *msgQueue, void * data, int size) {
    // Message à inscrire dans la queue de messages  
    void *msg;
    // numéro d'erreur 
    int err;

	// Allocation de mémoire pour le message à envoyer
    msg = rt_queue_alloc(msgQueue, size);
    
    // Copie dans le message de la data passée en argument
    memcpy(msg, &data, size);

	// Envoi du message dans la queue de messages
    if ((err = rt_queue_send(msgQueue, msg, sizeof (DMessage), Q_NORMAL)) < 0) {
        rt_printf("Error msg queue send: %s\n", strerror(-err));
    }
    // Nettoyage de la structure de queue 
    rt_queue_free(&queueMsgGUI, msg);

    return err;
}

/** 
* \fn void connexionMoniteurPerdue();
* \brief Fonction de nettoyage lancée lors de la perte de communication avec le moniteur
*/
void connexionMoniteurPerdue(){
	
	// Communication avec le moniteur perdue
	rt_mutex_acquire(&mutexEtatMoniteur, TM_INFINITE);
	etatCommMoniteur = 1;				
	rt_mutex_release(&mutexEtatMoniteur);
	
	// Suppression des tâches 
    rt_task_delete(&tServeur);
    rt_task_delete(&tconnect);
    rt_task_delete(&tmove);
    rt_task_delete(&tenvoyer);
    rt_task_delete(&tbatterie);
    rt_task_delete(&tmission);
    rt_task_delete(&tcamera);
    rt_task_delete(&twatchdog);
    
    // Nettoyage des variables globales 
    d_arena_free(arene);
    d_robot_free(robot);
    d_position_free(position);
    d_movement_free(move);
    d_server_free(serveur);
					
}


/**
* \fn int verifierPerteConnexion(int status)
* \brief Fonction de vérification d'une perte de connexion, accepte 3 échecs consécutifs avant de déclarer une perte effective de la communication avec le robot 
* \param int status : Statut de la connexion avec le robot
* \return Statut de la connexion :
*	0 si la connexion est effectivement perdue (plus de 3 erreurs consécutives)
*	1 si la connexion n'est pas considérée comme perdue
*/
int verifierPerteConnexion(int status) { 
	
	// Message à envoyé au moniteur
	DMessage * message;
	
	printf("---------------------- STATUS VERIFIER PERTE CONNEXION ------------ \n");
	rt_mutex_acquire(&mutexErreur, TM_INFINITE);
	// Si le statut passé en argument est OK, on remet le nombre de tentatives à 0
	if (status == STATUS_OK){
		// Mise à jour du statut de la communication avec le robot
		rt_mutex_acquire(&mutexEtat,TM_INFINITE);
		etatCommRobot = status;
		rt_mutex_release(&mutexEtat);	
		tentatives = 0; 
	}
	// Si le statut de la communication n'est pas OK
	else{
		// On incrémente le compteur de tentatives
		tentatives++;
		// Si le nombre de tentatives de communication avec le robot est supérieure à 3, on considère la communication perdue
		if (tentatives >= 10) { 
			// Envoi d'un message au moniteur
            message = d_new_message();			
			message->put_state(message, status);
            while (write_in_queue(&queueMsgGUI, message, sizeof (DMessage)) < 0) {
            	message->free(message);
            }
		rt_printf("TROP DE PERTES DE CONNEXION : SHUTDOWN ! \n"); 
		return 0;
		}
		// Mise à jour du statut de la communication avec le robot
		rt_mutex_acquire(&mutexEtat,TM_INFINITE);
		etatCommRobot = status;
		rt_mutex_release(&mutexEtat);
		// Si le status n'était pas OK, on tente une reconnexion 		
		rt_sem_v(&semConnecterRobot); 
	}
	rt_mutex_release(&mutexErreur);
	// En résumé, si la communication est OK, OU si elle ne l'est pas mais que ça fait moins de 3 fois consécutives, on n'envoie pas un signl de perte de connecteur au moniteur
	return 1; 
}

