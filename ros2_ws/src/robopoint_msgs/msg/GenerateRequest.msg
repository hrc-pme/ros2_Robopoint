# msg/GenerateRequest.msg
string request_id
string prompt
sensor_msgs/Image[] images
float32 temperature
float32 top_p
int32 max_new_tokens
string stop