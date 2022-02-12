# 

Hello Gennaro.

Ci-dessous, je t'explique ce que j'ai prévu, afin de faire fonctionner Ylo2 avec Wolf...

moteurs : 

	moteurs : QDD100 : https://mjbots.com/products/qdd100-beta-2
	controlleurs inclus dans les QDD100 : https://mjbots.com/products/moteus-r4-8
	fonctionnement des controlleurs : https://github.com/mjbots/moteus/blob/main/docs/reference.md
	librairie moteus à partir de laquelle je travaille : based on https://github.com/raess1/k3lso3_ros2/tree/master/k3lso_moteus (je dois l'adapter pour ylo2)

controlleur CAN :

	PCAN-M.2 Four Channel : https://www.peak-system.com/PCAN-M-2.473.0.html?&L=1
	utilisation du driver propriétaire (peak-linux-driver-8.14.0), à la place du socketcan
	
	La carte PCAN comprend 4 sorties :
	PCAN_PCIBUS1 : controlleurs 1,2,3
	PCAN_PCIBUS2 : controlleurs 4,5,6
	PCAN_PCIBUS3 : controlleurs 7,8,9,32 (32 est la power board / voltage / amperage)
	PCAN_PCIBUS4 : controlleurs 10,11,12

Structure du code :

	un hpp et un cpp
	
	une fonction rezero, pour s'assurer que les moteurs sont dans la bonne position de départ, avant d'exécuter le stand-up,
	une fonction stop, pour couper le torque
	une fonction read, qui lit les données en provenance des moteus (position, velocity, torque)
	une fonction read_imu, qui lit les données de mon imu (myahrs sous usb2, à 100Hz)
	une fonction write, qui envoit la commande torque aux moteus
	une fonction write_imu qui envoit les données imu a Wolf ??   
	
	-> You said: "Also, in the write() function you need to provide the imu informations"
	Je ne vois pas dans le code où l'imu est appelé, dans la fonction write()
	
Je continue mon code et je te ferai parvenir les fichiers très bientôt.

Bon week end, Gennaro
