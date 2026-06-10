uint8 DATA_SIZE_8_BIT=0
uint8 DATA_SIZE_16_BIT=1
uint8 DATA_SIZE_24_BIT=2
uint8 DATA_SIZE_32_BIT=3

std_msgs/Header header

uint32 ping_id
uint16 message_version
uint16 part_number
uint32 status

float64 frequency
float64 temperature
float64 pressure
float64 heading
float64 pitch
float64 roll
float64 speed_of_sound
float64 ping_start_time

uint8 data_size
string image_encoding
uint16 n_ranges
uint16 n_beams
float64 range
float64 range_resolution

int16[] bearings
uint8[] image_data
