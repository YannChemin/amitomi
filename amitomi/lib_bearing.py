import math as M

def bearing(lat1, lat2, lon1, lon2):
	"""
	http://stackoverflow.com/questions/17624310/geopy-calculating-gps-heading-bearing
	"""
	dLon = lon2 - lon1
	y = M.sin(dLon) * M.cos(lat2)
	x = M.cos(lat1)*M.sin(lat2)-M.sin(lat1)*M.cos(lat2)*M.cos(dLon)
	brng = M.atan2(y, x).toDeg()
	if (brng < 0):
		brng += 360
	return brng

