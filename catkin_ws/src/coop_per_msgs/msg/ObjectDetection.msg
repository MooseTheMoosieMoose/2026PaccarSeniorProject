#Position in Global Space
#Note that 32 bit floats have a worst case accuracy around ~1.7 meters
#While a 64 bit float has a worset case accuracy around ~3.16 nanometers
float64 lat_y
float64 long_x
float64 altitude_z

#The objects dimensions in meters
float32 depth
float32 width
float32 height

#The objects rotation along its up/down (Z) axis
float32 rotation

#The class name and its confidence score
string class_name
float32 confidence
