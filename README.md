**Project Name :** TurtleBOSS

**Developpeurs :** Arthur Josi - Thibaut Desfachelles - Olivier Blazevic 

**Les .py :**

Dans le package simple_controller, on peut retrouver deux fichiers pythons que nous avons écrit. L'un permet la detection des objects avec le placement des marqueurs dans rviz ("detection_et_marqueurs.py"), l'autre permet l'exploration autonome de l'environnement ("exploration_autonome.py").

**Les launch files pour le turtlebot réel sont placés dans le package simple_controller :**


-> Ouverture de la carte du labyrinth n°1 ou 2 dans RVIZ et possibilité de goal :
turtlebot_moveTo_salle1.launch
turtlebot_moveTo_salle2.launch

-> Création de carte avec le robot dans RVIZ (avec ou sans rosbag): Teleoperation requise (en ssh), avec ou sans detection d'objets et affichage de marqueurs.
turtlebot_mapping.launch 
turtlebot_mapping_from_rosbag.launch
turtlebot_mapping_and_detect.launch	
		-

-> Ouverture de la carte du labyrinth n°1 ou 2 dans RVIZ et detection d'objet avec positionnement de marqueurs :
turtlebot_move_inmap1_and_detect.launch	
turtlebot_move_inmap2_and_detect.launch	

-> Exploration autonome avec ou sans detection d'objets 
turtlebot_explo_auto.launch
turtlebot_explo_auto_and_detect.launch



**SIMULATION - Les launch files sont situés dans le package gazebo :**

-> Simulation gazebo avec positionnement de points objectifs dans rviz : 
	roslaunch gazebo gazebo_moveTo.launch 

-> Simulation gazebo avec création d'une carte de l'environnement :
	roslaunch gazebo gazebo_mapping.launch

-> Simulation de gazebo avec exploration autonome : La simulation ne semble pas fonctionner, sans doute un problème de topic mais non vérification par manque de temps.
	roslaunch gazebo_exploration_autonome.launch

-> La simulation de detection de bouteille n'aura pas été présentée pour des raisons évidentes. 

**VIDEO**
La video ayant été faite avec un mauvais logiciel de montage, une bande de pub désagréable s'est placée au centre.
Merci d'être compréhensible. Le lien est le suivant : 
