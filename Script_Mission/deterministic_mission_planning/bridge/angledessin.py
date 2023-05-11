import matplotlib.pyplot as plt
import math 
import numpy as np
def dessin() :


 
    
 

    fichier = open("log_position.txt", "r")
    donee_ligne=fichier.readlines() 
    fichier.close()
    

    for i in range(1,len(donee_ligne)):

        donnee=donee_ligne[i].split(',')
        rov_angle=float(donnee[4])*180/math.pi

        plt.plot(i,rov_angle,'.b')
   
    for i in range(2,len(donee_ligne)):

        donnee=donee_ligne[i].split(',')
        point_act=[float(donnee[1]),float(donnee[2])]
        donnee=donee_ligne[i-1].split(',')
        point_pre=[float(donnee[1]),float(donnee[2])]

        p0=[float(donnee[1])+10000,float(donnee[2])]

        p1=point_pre
        p2=point_act

        v0 = np.array(p0) - np.array(p1)
        v1 = np.array(p2) - np.array(p1)
        
        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        #print(np.degrees(angle))
        plt.plot(i,np.degrees(angle),'.r')

    plt.show()
dessin()



