void camera (void * arg){
	// On déclare toutes les variables nécessaires à l'acquisition de l'image 
	Dcamera *camera;
	DImage *image;
	DJpegimage *jpegimg;
	Dmessage *message;
	
	int status;
	int etat;
	
	// déclaration d'un état de sauvegarde dans le cas où on a un problème initial
	int backupEtat = ACTION_STOP_COMPUTE_POSITION;
	
	// Instruction de contrôle (visible sur terminal)
	rt_printf("-------------------- EXECUTION INITIALE THREAD CAMERA -------------------- \n");
	
	//Périodicité du thread : 600 ms 
	rt_task_set_periodic(NULL,TM_NOW, 600 000 000);
	
	// création de la caméra
	camera = d_new_camera;
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
					camera -> d_camera_get_frame(camera , image);

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
					img->free(img);
					
					break;
				
				
				// Si on demande le calcul de la position (fonctionnement usuel), on fait la même routine, plus le calcul de la position
				case ACTION_COMPUTE_CONTINUOUSLY_POSITION : 
				
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- RECUPERATION IMAGE AVEC CALCUL DE POSITION THREAD CAMERA -------------------- \n");
					
					// Création d'une nouvelle image
					image = d_new_image();
					
					// Capture de l'image via la caméra
					camera -> d_camera_get_frame(camera , image);
					
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
					img->free(img);
					
				break;
				
				// Demande de détection d'arène (ne devrait arriver qu'une fois à l'initialisation du système)	
				case ACTION_FIND_ARENA : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- DEMANDE DETECTION ARENE THREAD CAMERA -------------------- \n");
					
					// Création d'une nouvelle image
					image = d_new_image();
					
					// Capture de l'image via la caméra
					camera -> d_camera_get_frame(camera , image);
					
					// Récupération de la position de l'arène 
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					arene = img->compute_arena_position(img);
					
					// Si une arène est effectivement détectée, on la dessine sur l'image 					
					if (arene != NULL){
						d_imageshop_draw_arena(image,arene);
					}
					rt_mutex_release(&mutexArene, TM_INFINITE);
					
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
					img->free(img);
					
				break;
				
				// Echec de la recherche d'arène
				case ACTION_ARENA_FAILED : 
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- ECHEC DETECTION ARENE THREAD CAMERA -------------------- \n");
						
					// Nettoyage de la structure arène
					rt_mutex_acquire(&mutexArene, TM_INFINITE);
					arene->free(arene);
					rt_mutex_release(&mutexArene);
				
				break;
				
				// Succès de la recherche d'arène
				case ACTION_ARENA_IS_FOUND :
					// Instruction de contrôle (visible sur terminal) 
					rt_printf("-------------------- ECHEC DETECTION ARENE THREAD CAMERA -------------------- \n");
					
				break;
				
			}
		}
	}
}
					
					
