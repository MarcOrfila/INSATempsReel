make
if [ $? -eq 0 ]; then 
	../upload.sh $1 robot
	wait
	java -jar ../../moniteur-2-1-5-beta/moniteur-2-1-5-beta/Moniteur.jar
else
	echo "Erreur à la compilation";
fi;

#PIDROBOT=`ps -ef | grep "gedit" | tr '  ' ' ' | cut -d ' ' -f2`
#kill -9 $PIDPIDROBOT
#ssh insa@geitp-trs$1 exec /home/insa/tp_trs/robot
