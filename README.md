# Blue_rov_planification
- Le but de ce code Python est de simuler le blurov2 grâce à Ardusub dans Qgroundcontrole et dans un monde crée dans Unity. 
Le monde Unity reprend une parti du port de la pointe rouge a Marseille.

# Prérequis ↓↓ documentation pour installation ↓↓

- Ubuntu 20
- python 3.8
- QgroundControle
- ArduPilot avec ArduSub
- PyMavlink
- PyNav
- Unity 2021.3.18f1
- ROS Noetic



# Installations Ubuntu 20.04 :
- Pour installer Ubuntu nous avons deux posibilitées. Changer totalement de système d'exploitation soit faire un Dual Boot. Un Dual Boot va permettre d'avoir deux système d'exploitation sur le même ordinateur et de le choisir au démarrage. De cette manière on peut avoir Ubuntu et Window par exemple. 
Dans les deux cas nous devont faire un clé qui va permettre d'installer Ubuntu : 

## Création de la clé USB d'installation d'Ubuntu :
- Télécharger l'ISO sur le site officiel de Ubuntu : https://releases.ubuntu.com/focal/?_ga=2.34191777.1471201453.1681201194-365250785.1681201194

- Télécharger Rufus portable : https://rufus.ie/fr/

- Branche votre Clé USB d'au moins 8Go, puis séléctioner l'ISO préalablement télécharger. Enfin cliquer sur démarrer. Attention le contenu de la clé sera supprimer. Une fois terminer vous pouvez passer la suite  **Installations Ubuntu 20.04 en Dual Boot** OU **Installation d'uniquement Ubuntu 20.04**. 

 <img src="https://user-images.githubusercontent.com/122261448/231146007-533f3aba-a112-4c86-9467-520755a0dd3c.png" width="600" /> 

## Installations Ubuntu 20.04 en Dual Boot : 
- Dans un premier temps on doit désactiver le Secure Boot.

### Désactiver le Secure Boot sur un PC UEFI :

- En cas de problème ce référer à cette documentation  :https://lecrabeinfo.net/desactiver-secure-boot-windows-carte-mere-uefi.html

- Afin de désactiver le Secure Boot entrer dans le BIOS/UEFI de votre ordinateur. Sur windows 11 rédémarrer l'ordinateur et appuyer sur **F12** lors du démarage. Aller dans **Options avancées** puis **Changer les paramètres du microprogramme UEFI**. Normalement le BIOS ce lance. Le paramètres du Secure Boot ce trouve généralement dans les paramètres de Boot ou de Secutity. Désactiver le. Sauvegarder et redémarrez votre PC.
### Désactiver le démarrage rapide de Windows 11 :
- Aller dans le Panneau de configuration ensuite Matériel et audio puis Options d'alimentation enfin **Choisir l'action des boutons d'alimentations**
- Cliquer sur **Modifier des paramètres actuellement non disponibles**.
- Déchochez la case **Activer le démarrage rapide**.

### Partitionner le disque système :

- Cliquer sur touche-windows+R et lancer : 
         
         diskmgmt.msc 

- Séléctionner le disque avec la partition Windows, faite un clic droit et cliquer sur **Réduire le volume**.
Rentrer la taille voulu de la partiton de Ubuntu (personnellement j'ai attribué 150Go ) puis cliquer sur **Réduire**.

### Installation d'Ubuntu 20.04 :

- Redémarre votre PC, lors du démarrage cliquer sur **F12**.  Séléctionner Démmarer a partir d'un périphérique puis séléctionner votre clé. 
- Votre PC redémarre une nouvelle fois. Séléctionner Ubuntu. Une vérification des fichiers ce lance. 
 
- Une fois finie une page s'ouvre. Séléctionner **Installer Ubuntu**.
         
   - Séléctionner la langue du clavier voulu (puis continuer)
   - Séléctionner ensuite **Installation minimale** ainsi que **Télécharger les mises à jour pendant l’installation de Ubuntu** et **Installer un logiciel tiers pour le matériel graphique et Wi-Fi et des formats de média supplémentaires** (puis continuer). 
   - Séléctionner **Autre chose**, double cliquer sur l'espace préalablement libérer et ajouter un "/" puis cliquer sur installer. 
   - Séléctionner votre pays (puis continuer). 
   - Enfin rentrer votre nom d’utilisateur, votre mot de passe, le nom du PC (puis Continuer).

- Il ne reste plus cas redémarrer le PC. Le message suivant va s'afficher : "Please remove the installation meduim, then press ENTER". Il suffit donc de retirer la clé puis d'appuyer sur entrer. 



## Installations Ubuntu 20.04 uniquement:

- Redémarre votre PC, lors du démarrage cliquer sur **F12**.  Séléctionner Démmarer a partir d'un périphérique puis séléctionner votre clé. 
- Votre PC redémarre une nouvelle fois. Séléctionner Ubuntu. Une vérification des fichiers ce lance. 
 
- Une fois finie une page s'ouvre. Séléctionner **Installer Ubuntu**.
         
   - Séléctionner la langue du clavier voulu (puis continuer)
   - Séléctionner ensuite **Installation minimale** ainsi que **Télécharger les mises à jour pendant l’installation de Ubuntu** et **Installer un logiciel tiers pour le matériel graphique et Wi-Fi et des formats de média supplémentaires** (puis continuer). 
   - Séléctionner **Effacer le disque et installer Ubuntu**, double cliquer sur l'espace préalablement libérer et ajouter un "/" puis cliquer sur installer. 
   - Séléctionner votre pays (puis continuer). 
   - Enfin rentrer votre nom d’utilisateur, votre mot de passe, le nom du PC (puis Continuer).

- Il ne reste plus cas redémarrer le PC. Le message suivant va s'afficher : "Please remove the installation meduim, then press ENTER". Il suffit donc de retirer la clé puis d'appuyer sur entrer. 

## Visual Studio Code
To work on this projet we use Visual Studio Code. It's a simple IDE witch allow to work with different programming languages ( Where we mainly use Python).  To install Visual studio code on Ubuntu, 2 solution :

- 1 use the Ubuntu Software, search "code" et launch the installation.

- 2 Download the .deb where : https://code.visualstudio.com/download and run this command in a terminal :

         sudo apt install ./<file>.deb
         
         
Once the IDE launch, go into Extensions and install **Python**.


<img src="https://user-images.githubusercontent.com/122261448/231142348-6e99f545-2b9b-48b0-9ec5-f0dc8ae71dc9.png" width="400" />



## Installer Python 3.8 : 
- update to have the last version of Python :

         sudo apt update
         sudo apt -y upgrade
                  
- To verify if the good version of Python is install run : 

         python3 -V
         
the result must be : Python 3.8.10

## Installation de ROS Noetic
- add packages.ros.org to the sources list :

         sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

         sudo apt install curl 

         curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
         
- Install ROS Noetic package :

         sudo apt update
         sudo apt install ros-noetic-desktop-full
         
- Sources automatisation :

         source /opt/ros/noetic/setup.bash

         echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

         source ~/.bashrc
         
- Python3/ROS dependencies

         sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

         sudo apt install python3-catkin-tools python3-osrf-pycommon
         
- Rosdep initialisation  :

         sudo rosdep init

         rosdep update
         
- Cloning the main repository:
 
      git clone  https://github.com/ELTGR/Mission_Planning.git

- Ceate a dircetory name " catkin_ws " into the main directory in witch one create a directory name " src "

- Clone the ROS TCP Endpoint packages into src :

      cd Mission_Planning/catkin_ws/src

      git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
      
- Go into the Mission_Planning/catkin_ws directory and run : 

         catkin build
         
 - You must obtain this:    
      
 <img src="https://user-images.githubusercontent.com/122261448/231142898-501e7eb3-9679-43fc-abac-b45a204b17c0.png" width="600" /> 

- Update the environment variables

      echo 'source $HOME/Mission_Planning/catkin_ws/devel/setup.bash' >> ~/.bashrc

      source ~/.bashrc
      
## Install Qgroundcontrol 

- In case of probleme refer to this documentation : https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

         sudo usermod -a -G dialout $USER
         
         sudo apt-get remove modemmanager -y
         
         sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
         
         sudo apt install libqt5gui5 -y
         
         sudo apt install libfuse2 -y
         
- Logout and login to make effective the changes.
 
 Download QGroundControl.AppImage : https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

 - To launch Qground : 
 
         chmod +x ./QGroundControl.AppImage
         ./QGroundControl.AppImage  (or double click)
         
               
## Install ArduPilot

- In case of probleme refer to this documentation : https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

         sudo apt-get update

         sudo apt-get install git

         sudo apt-get install gitk git-gui       
  
         git clone https://github.com/your-github-userid/ardupilot
         
         cd ardupilot
         
         git submodule update --init --recursive
      
         Tools/environment_install/install-prereqs-ubuntu.sh -y
 
         . ~/.profile
         
- Logout and login to make effective the changes.


## Install PyMavlink
- In case of probleme refer to this documentation : https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

         sudo python -m pip install --upgrade future lxml

## Install Navpy   :

- Open a terminal and run :

          pip install NavPy


## Install Unity 2021.3.18f1
### Install Unity 
:
- Add Unity to the repository : 

         sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
         
- Public key :

         wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add -
         
- Install and Update Unity :

         sudo apt update
         sudo apt-get install unityhub
     
### Install 2021.3.18f1:

- Launch UnityHub and ingnore the proposition. In **Installs** click on **Install Editor**.
Go into **Archive**   click **download archive**, on the website look for the **version 2021.3.18**.Ones found click **Unity Hub**. The download must start into UnityHub.

- We clone the repository of the Unity word (attention don't clone it into the Bluerov2_dock_scan):

         git clone https://github.com/Gregtmlg/Mission_Planning.git
         
- We will verify if all usefull Unity packages are install. For this go in Window > Manager. 
- In projet list packages, verify the presence of : Burst, Addressables and ROS TCP Connector. 

### Install Unity Repository :

work in progress

##    Ardupilot modification   :
- In the Ardupilot/Tool/autotest.location.txt add this line : 
 
         Marseille=43.243908,5.363613,0,90



##   bluerov_node modification de  :

- In the **get_yaml_data()** fonction file **bluerov_node** in th repository **mission_planning** modify the path: 

   Config_scan = "/home/eliott/Desktop/Mission_Planning/mission_planning/bridge/Config_scan.yaml"



## Explanation of the communication system

<img src="https://user-images.githubusercontent.com/122261448/230575616-42fc961a-43e4-4340-aa1e-d88e82931272.jpg" width="1000" />

###      Communication between the BlueRov2 and Python script : 

<img src="https://user-images.githubusercontent.com/122261448/230575819-90deafe4-0a40-43dd-98d3-a16354ca939a.jpg" width="450" />

To communicate with the ROV we use PyMavlink. This library provides us some usefull fonction to received and send data. 

Look script " bridge.py " 

First we have to establish the communication settings

<img src="https://user-images.githubusercontent.com/122261448/230570370-56251835-b07b-47f3-926b-f53c2280552f.png" width="600" />

Where we have enter the main communication setting like adresse and baurate.
Next with the **get_data()**, **get_all_msgs()** and **update()** will allow us to received all the ROV data.
In **get_bluerov_position_data** we decide to get some precise data like x, y, z, yaw witch are the rov position and orientation in the NED.
To give some positions order to the ROV, we use the fonction **set_position_target_local_ned()** 

<img src="https://user-images.githubusercontent.com/122261448/231151892-54c8cdc2-0be4-4415-a288-238422a0ce33.png" width="400" />


###       Communication between Python script and ROS : 

<img src="https://user-images.githubusercontent.com/122261448/230576048-76cbda38-1670-4e5e-a84a-b92fcad3e9b2.jpg" width="400" />
ROS allow us to create topics in witch some messages can be write or read. Thanks to that some programme witch don't have the same programmation language can communicate. 
It's exatly what we do here. We transfere some data of our Python script into a C# script to update the ROV position in Unity.

Look script  " bluerov_node.py  " 
In **__init__** into the class **BlueRov** we define a topic named **/local_position**. It's defined in **pud** that means we will write without read what the topic containe.
In **def_create_position_msg(self)** we received the ROV position data from PyMavlink.

At the end of the fonction we publish the data into the topic previously defined




###      Communication between ROS and the C# script of Unity : 

<img src="https://user-images.githubusercontent.com/122261448/230582932-8fa5c320-011b-4071-a8f7-b3d256a838ed.jpg" width="400" />

To update the ROV position into Unity we use a C# script named **UpdateBluerovPose.cs** this script is in the **Asset** directory of the Unity projet.
To make the link between the ROV and the file, you just have to create an empty box, click on it, go into properties and clikc on **add Component** and select the script. Next put the 3D ROV model into the empty box.

<img src="https://user-images.githubusercontent.com/122261448/231139812-f7cb5af1-342d-492d-8968-53d116716424.png" width="1200" />

We now look how read the data of the ROS topic.

Look  " UpdateBluerovPose.cs " script the **Asset** directory of the Unity projet. 

<img src="https://user-images.githubusercontent.com/122261448/230583657-7248f54e-ba7d-4c5c-8d00-0ac3c9f1fcc7.png" width="600" />

In the fonction **void Start()** we create an **Subscribre** witch will read the data of the **/BlueRov2/local_position** topic.
We can found the callback named **OdomChange** witch allow us to received the infomations.The rest of the script make some conversion. 



#   User guide : 

- Open a terminal, launch the ROS TCP Endpoint with :
 
                  roslaunch ros_tcp_endpoint endpoint.launch
                  
- In the ardupilot/ardusub repository open a second terminal andcrun :

         sim_vehicle.py -L Marseille  -S 1  --out=udp:0.0.0.0:14550 --map --console

" -S 1 " is the speed control if u want to increase it, go into /home/%sessions_name%/.local/bin/mavproxy.py and modify the hearthbeat value. Then launch the command with the same value write in the mavproxy.py script

- Launch QgroundControl.

- Launch the Unity projet.

- In the mission_planning directory run **bluerov_node.py**.

