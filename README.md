**Projet : TurtleBOSS**

**Developpeurs : Arthur Josi - Thibaut Desfachelles - Olivier Blazevic** 


Les scripts pythons :
==
Dans le package simple_controller, on peut retrouver deux fichiers pythons que nous avons écrit. L'un permet la detection des objects avec le placement des marqueurs dans rviz (**"detection_et_marqueurs.py"**), l'autre permet l'exploration autonome de l'environnement (**"exploration_autonome.py"**).

---
Les launch files pour le turtlebot réel sont placés dans le package simple_controller :
==
**-> Ouverture de la carte du labyrinth n°1 ou 2 dans RVIZ et possibilité de goal :**  
roslaunch simple_controller turtlebot_moveTo_salle1.launch  
roslaunch simple_controller turtlebot_moveTo_salle2.launch

**-> Création de carte avec le robot dans RVIZ (avec ou sans rosbag): Teleoperation requise (en ssh), avec ou sans detection d'objets et affichage de marqueurs.**  
roslaunch simple_controller turtlebot_mapping.launch  
roslaunch simple_controller turtlebot_mapping_from_rosbag.launch   
roslaunch simple_controller turtlebot_mapping_and_detect.launch  
		
**-> Ouverture de la carte du labyrinth n°1 ou 2 dans RVIZ et detection d'objet avec positionnement de marqueurs :**  
roslaunch simple_controller turtlebot_move_inmap1_and_detect.launch 	 
roslaunch simple_controller turtlebot_move_inmap2_and_detect.launch 	 

**-> Exploration autonome avec ou sans detection d'objets**   
roslaunch simple_controller turtlebot_explo_auto.launch  
roslaunch simple_controller turtlebot_explo_auto_and_detect.launch  

---

SIMULATION - Les launch files sont situés dans le package gazebo :
==
**-> Simulation gazebo avec positionnement de points objectifs dans rviz :**  
roslaunch gazebo gazebo_moveTo.launch 

**-> Simulation gazebo avec création d'une carte de l'environnement :**  
roslaunch gazebo gazebo_mapping.launch

**-> Simulation de gazebo avec exploration autonome : La simulation ne semble pas fonctionner, sans doute un problème de topic mais non vérification par manque de temps.**   
roslaunch gazebo_exploration_autonome.launch

**-> La simulation de detection de bouteille n'aura pas été présentée pour des raisons évidentes.** 

---
VIDEO
==
La video ayant été faite avec un mauvais logiciel de montage, une bande de pub désagréable s'est placée au centre.
Merci d'être compréhensible. Le lien est le suivant :  
https://l.facebook.com/l.php?u=https%3A%2F%2Fwww.dropbox.com%2Fs%2Fvy14sfsyd6fm67i%2FMa%2520vid%25C3%25A9o.mp4%3Fdl%3D0%26fbclid%3DIwAR27iF0VGoZd6aceU6w4YeHX1wxf8V8LkQL-w2KhxJq4EnAwkNLb00CohkE&h=AT2uHU8APiUooRDk9FLj_Dv6hgdtx8sCMByVdh2f2E2hj8UB6kAO1v0JCINe9USwLlpxG21SMt12d2yu56_oFKbXVr_IdqOVFi-whQsJgXkpUKCf48TvagwpjFcPaPy2yUA9Pw
