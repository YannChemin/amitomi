#!/usr/bin/env python
import numpy as np
#Navigation test engine
#Loads fake datasets and runs the skipper through the data

releasepoint = np.genfromtxt('releasepoint001.csv',delimiter=',')
waypoints = np.genfromtxt('waypoints001.csv',delimiter=',')
upwinddir = np.genfromtxt('upwinddir001.csv',delimiter=',')
print 'releasepoint = ', releasepoint
print 'waypoints = w', waypoints
print 'upwinddir = ', upwinddir, '[0.0=North]'

#set training velocity at 1m/s (motorbike dataset)
v=1.0

#Sort the waypoints by most upwind first
#Step 1: create a bounding box for the waypoints coordinates
def bbox(waypoints):
    #First Column is latitude
    bbox_north=waypoints[:,0].max()
    bbox_south=waypoints[:,0].min()
    bbox_east=waypoints[:,1].max()
    bbox_west=waypoints[:,1].min()
    print 'BOUNDING BOX'
    print '\t',bbox_north
    print bbox_west,bbox_east
    print '\t',bbox_south
    return(bbox_north,bbox_south,bbox_east,bbox_west)

bnd = bbox(waypoints)

#Step 2: define the central point of the bounding box
def bbox_center(bnd):
    lat = bnd[1] + (bnd[0]-bnd[1])/2.0
    lon = bnd[3] + (bnd[2]-bnd[3])/2.0
    return(lat,lon)

bnd_center = bbox_center(bnd)

#Step 3: define the half diagonal length
def bbox_half_diag_length(bnd,bnd_center):
    #load upper right coordinates
    ur_lat = bnd[0]
    ur_lon = bnd[2]
    #"Euclidian" distance
    d = np.sqrt(pow((ur_lat-bnd_center[0]),2)+pow((ur_lon-bnd_center[1]),2))
    #return radius of bearing circle
    return(d)

radius = bbox_half_diag_length(bnd,bnd_center)

#step 4: Define upwind virtual waypoint with radius distance from bbox center
def upwind_virtual_waypoint(upwinddir,bnd_center,radius):
    #set the triangle with upwinddir, radius, bnd_center
    #conversion from clock-wise bearing to arithmetic East-bound X axis start of angle
    temp = upwinddir - 90.0
    if(temp<0):
        temp += 360.0 
    a = (temp * (-1)) + 360.0
    #compute circle coordinates with bbox center point offset
    lat=bnd_center[0]+radius*np.sin(a) 
    lon=bnd_center[1]+radius*np.cos(a)
    return(lat,lon)

upwwp = upwind_virtual_waypoint(upwinddir,bnd_center,radius)

#step 5: find most upwind waypoint by distance to virtual upwind waypoint
def upwind_waypoint_find(waypoints,upw):
    distance = []
    dmin = 10000000
    for i in range (len(waypoints)):
        distance.append(np.sqrt(pow((upw[0]-waypoints[i,0]),2)+pow((upw[1]-waypoints[i,1]),2)))
        if(distance[i]<dmin):
            dmin = distance[i]
            upwind_wp_idx = i

    print upwind_wp_idx
    print distance
    return(waypoints[upwind_wp_idx,0],waypoints[upwind_wp_idx,1])

first_waypoint = upwind_waypoint_find(waypoints,upwwp)

#At this point the boat can start travelling to the first waypoint
import os
#delete previously used waypoints
os.system("rm -f skipper_waypoints.py &")
os.system("python skipper.py "+first_waypoint[0]+" "+first_waypoint[1]+" &")
#Skipper keeps trying to load the waypoint list on course to first_waypoint
#If not ready by first_waypoint, it will failsafe back to release point

#Meanwhile the sorting of the remaining waypoints can happen
print "#step 6: Create matrices of distance and angle from wind"
#step 6: Create matrices of distance and angle from wind
def distangle(waypoints,upwinddir):
        #make matrices of heading <-> upwbearing angles and distances between waypoints
        angle = np.zeros((len(waypoints),len(waypoints)))
        distance = np.zeros((len(waypoints),len(waypoints)))
        for i in range(len(waypoints)):
                for j in range(len(waypoints)):
                        if(i!=j):
                                distance[i,j] = np.sqrt(pow((waypoints[j,0]-waypoints[i,0]),2)+pow((waypoints[j,1]-waypoints[i,1]),2))
                                #make an upwind waypoint from wpt i with a distance (i,j)
                                upw = upwind_virtual_waypoint(upwinddir,waypoints[i,0:2],distance[i,j])
                                #create vector i,upw
                                i_upw = [upw[0]-waypoints[i,0],upw[1]-waypoints[i,1]]
                                #create vector i,j
                                i_j = [waypoints[j,0]-waypoints[i,0],waypoints[j,1]-waypoints[i,1]]
                                #angle of i_j with i_upw
                                angle[i,j] = atan2(i_j[0],i_j[1])-atan2(i_upw[0],i_upw[1])
                                if(angle[i,j]<0):
                                        angle[i,j] += 2*np.pi
                                if(angle[i,j]>360.0):
                                        angle[i,j] -= 2*np.pi
                                angle[i,j] *= (360.0 / (2*np.pi))
                        else:
                                distance[i,j] = np.nan
                                angle[i,j] = np.nan

        return(distance,angle)

distM, angleM = distangle(waypoints,upwinddir)

print "#step 7: Route waypoints"
# 1- the distance of any wpt from virtual upwind wpt (dist2upw)
# 2- the distance of any wapt to any other wpt (distM)
# 3- the angle from wind of any two waypoints vector (angleM)
def route(waypoints, distM, angleM, first_waypoint):
        #sudo apt-get install python-openopt python-networkx
        import openopt as oo
        from numpy import sin, cos
        import networkx as nx

        N=len(waypoints)
        G = nx.Graph()
        G.add_edges_from([(i,j,{'angle': 360.0-angleM[i,j], 'distance': distM[i,j]}) for i in range(N) for j in range(N) if i != j ])
        objfn = lambda values: values['distance'] + 10*values['angle']
        p = oo.TSP(G, objective = objfn, start = first_waypoint[2], returnToStart=False)
        r = p.solve('glpk')
        print("nodes = ", r.nodes)
        print("edges = ", r.edges)
        route_list_lat = np.zeros(N)
        route_list_lon = np.zeros(N)
        route_list_idx = np.zeros(N)
        for i in range(N):
                route_list_lat[i] = waypoints[r.nodes[i]][0]
                route_list_lon[i] = waypoints[r.nodes[i]][1]
                route_list_idx[i] = r.nodes[i]
        return(route_list_lat,route_list_lon,route_list_idx)

rlat,rlon,ridx = route(waypoints, distM, angleM, first_waypoint)

import pylab as pb
pb.plot(releasepoint[1],releasepoint[0],"o",c="red")
pb.plot(rlon,rlat,"o")
for i in range(len(rlon)):
        pb.text(rlon[i],rlat[i],"#"+str(i)+"#"+str(ridx[i]))
pb.show()

