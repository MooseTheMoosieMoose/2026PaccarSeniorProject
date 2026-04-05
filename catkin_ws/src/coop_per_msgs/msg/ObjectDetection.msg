#Position in Global Space
#Note that 32 bit floats have a worst case accuracy around ~1.7 meters
#While a 64 bit float has a worset case accuracy around ~3.16 nanometers
float64 latitude
float64 longitude
float64 altitude

#The objects dimensions in meters
float32 length
float32 width
float32 height

#The objects rotation along its up/down (Z) axis
float32 rotation

#The UNIX epoch timestamp
float64 timestamp

#The class name and its confidence score
string class
float32 confidence
