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
