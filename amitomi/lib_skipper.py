def bbox(waypoints):
    #Step 1: create a bounding box for the waypoints coordinates
 
def bbox_center(bnd):
    #Step 2: define the central point of the bounding box
 
def bbox_half_diag_length(bnd,bnd_center):
    #Step 3: define the half diagonale length
 
def upwind_virtual_waypoint(upwinddir,bnd_center,radius):
    #step 4: Define upwind virtual waypoint with radius distance from bbox center
 
def upwind_waypoint_find(waypoints,upw):
    #step 5: find most upwind waypoint by distance to virtual upwind waypoint
 
def waypoint_dist_to_firstwp(waypoints,first_waypoint):
    #step 6: waypoints distance to first waypoint
 
def distangle(waypoints,upwinddir,upw):
    #step 7: Create matrices of distance and angle from wind
 
#Boat should start sailing to first waypoint from here...

def route(waypoints, distM, angleM, first_waypoint):
    #step 8: Create a downwind route through the waypoints by ordering them

#End of routing steps, the final route has been computed and stored on disk


#Navigational elements
def getbearing():
    # Read and display the raw magnetometer readings

def makeactualpos(last_bearing, last_true_gps_pos, last_kalman_pos, last_time, last_velocity, last_attitude):


#Still Working on those...
def navigate(mode,waypoint):    
    

def survey(mode,waypoint):
    #set course to that waypoint
