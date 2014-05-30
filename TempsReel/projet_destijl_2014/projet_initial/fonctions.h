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


#ifndef FONCTIONS_H
#define	FONCTIONS_H

#include "global.h"
#include "includes.h"

#ifdef	__cplusplus
extern "C" {
#endif
        
        /**
		* \fn void camera (void * arg)
		* \brief Fonction exécutée par le thread tCamera, permet la détection d'arène, le calcul de position et l'envoi d'images au moniteur
		*/
		void camera(void *arg);
		
		/**
		* \fn void envoyer(void * arg)
		* \brief Fonction exécutée par le thread tEnvoyer, permet l'envoi d'un message du superviseur au moniteur 
		*/
        void envoyer(void *arg);
        
		/**
		* \fn void connecter(void * arg)
		* \brief Fonction exécutée par le thread tConnecter, effectue la procédure de connexion/reconnexion avec le robot 
		*/        		
        void connecter (void * arg);
        
        /**
		* \fn void communiquer(void * arg)
		* \brief Fonction exécutée par le thread tServeur, sert d'interface entre les messages envoyés via le moniteur, et le superviseur 
		*/		
        void communiquer(void *arg);
        
        /**
		* \fn void etat_batterie(void * arg)
		* \brief Fonction exécutée par le thread tbatterie, envoie l'état de la batterie au moniteur
		*/
        void etat_batterie(void *arg);
        
        /**
		* \fn void deplacer(void * arg)
		* \brief Fonction exécutée par le thread tmove, permet le déplacement manuel du robot
		*/
        void deplacer(void *arg);
        
        /**
		* \fn void watchdog(void * arg)
		* \brief Fonction exécutée par le thread twatchdog, permet le rechargement du watchdog du robot (utile uniquement en start secure)
		*/	
        void watchdog(void *arg);
        
        /**
		* \fn void mission_reach_coordinates(void * arg)
		* \brief Fonction exécutée par le thread tmission, permet au robot d'aller au point de l'arène indiquée par l'utilisateur
		*/
        void mission_reach_coordinates(void * arg);
        
#ifdef	__cplusplus
}
#endif

#endif	/* FONCTIONS_H */

