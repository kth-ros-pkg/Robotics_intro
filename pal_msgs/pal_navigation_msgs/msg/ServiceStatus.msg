# Navigation ServiceStatus msg 
string serviceName

#true if the service has been started
bool    is_running

#expected frequency of control loop (in hertz)
float32 update_frequency

#Timestamp of the latest loop cycle execution
time    latest_run_time

#true if the service is running and working fine
bool    status
