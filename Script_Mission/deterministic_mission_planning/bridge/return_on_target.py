
import math
from random import *
import visualisation_rov_way as vs_rov

def target_coordinates(distance_recording_cible,nbr_target_passing) : 

    print("\n---------START TARGET SCAN---------")
    print("Looking for target position")
    
    fichier = open("sonar_time.txt", "r")
    time=fichier.readlines()
    
    fichier.close()

    fichier = open("log_position.txt", "r")
    log=fichier.readlines() 
    fichier.close()
        

    new_waypoint_x=[]
    new_waypoint_y=[]
    target_point_x=[]
    target_point_y=[]
    x=[]
    y=[]
    donee_ligne=[]

    #print(len(time))

    for i in range(0,len(time)) :
        minimum_line2add=[math.inf,0]
        j=time[i].split(',')
        
        for l in range(1, len(log)) :

            time_log=log[l].split(',')[0].split(':')
            time_sonar=j[0].split(':')

            #check houre
            #print(time_log[0],"=",time_sonar[0])
            if int(time_log[0])==int(time_sonar[0]) :
                #check minute
                #print(time_log[1],"=",time_sonar[1])
                if int(time_log[1])==int(time_sonar[1]) :
                    #¢heck second
                    
                    if abs(float(time_log[2])-float(time_sonar[2])) < minimum_line2add[0] and abs(float(time_log[2])-float(time_sonar[2])) <= 1.5  and minimum_line2add != 0.0:
                        minimum_line2add=[abs(float(time_log[2])-float(time_sonar[2])),l]
                        #print("inside : ", minimum_line2add)
        #print(minimum_line2add)
        #print(log[minimum_line2add[1]])
        donee_ligne.append(log[minimum_line2add[1]])
                        
                        
 
    donnee=[]
    positonrov=[]
    print("Creation of the new waypoint")
    for i in range(0,len(time)) :
        
        donnee=donee_ligne[i].split(',')
        #print(donnee)
        rov_angle=float(donnee[4])
       
        rov_position_x=float(donnee[1])
        rov_position_y=float(donnee[2])

        target_x=rov_position_x+((distance_recording_cible/2)*math.cos(rov_angle))
        target_y=rov_position_y+((distance_recording_cible/2)*math.sin(rov_angle))
        
        start_stopx, start_stopy = point_star_stop(target_x, target_y,distance_recording_cible,nbr_target_passing)

        x.append(start_stopx)
        y.append(start_stopy)
        
        target_point_x.append(target_x)
        target_point_y.append(target_y)
        new_waypoint_x.append(start_stopx)
        new_waypoint_y.append(start_stopy)
        
    #uncommente this line to see the way of the rov 
    vs_rov.dessin_pos(new_waypoint_x,new_waypoint_y,target_point_x,target_point_y)




    

    return x,y


def point_star_stop(x,y,range_sensor,nbr_target_passing) : 

    start_stopx=[]
    start_stopy=[]  

    angle = 180/nbr_target_passing
    angle=math.radians(angle)
    
    for i in range(nbr_target_passing) : 
        
        if i%2 == 0 :

            point_start_x = x + (range_sensor * math.cos(angle*i))
            point_start_y = y + (range_sensor * math.sin(angle*i))

            point_stop_x = x - (range_sensor * math.cos(angle*i))
            point_stop_y = y - (range_sensor * math.sin(angle*i)) 

        else :       
                                  
            point_start_x = x - (range_sensor * math.cos(angle*i))
            point_start_y = y - (range_sensor * math.sin(angle*i))

            point_stop_x = x + (range_sensor * math.cos(angle*i))
            point_stop_y = y + (range_sensor * math.sin(angle*i))

        start_stopx.append(point_start_x)
        start_stopy.append(point_start_y)

        start_stopx.append(point_stop_x)
        start_stopy.append(point_stop_y)

    return start_stopx,start_stopy

