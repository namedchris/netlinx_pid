module_name = 'pid_controller' (dev this)
(***********************************************************)
(*  FILE_LAST_MODIFIED_ON: 01/22/2024  AT: 10:31:35        *)
(***********************************************************)

define_constant
//timeline
PID_LOOP_ID = 1

//levels
SETPOINT_LEVEL_ID = 1
INPUT_LEVEL_ID = 2
OUTPUT_LEVEL_ID = 3

define_variable

//level values
float setpoint
float input
float output


float error
float last_error
float i_error
float d_error
float d_input
float last_input

//gain settings
float kp
float ki
float kd

//output limits
float min_output
float max_output

slong intervals[1]

integer enabled

define_start
create_level this, SETPOINT_LEVEL_ID, setpoint
create_level this, INPUT_LEVEL_ID, input
create_level this, OUTPUT_LEVEL_ID, output

intervals[1] = 1000

timeline_create(
    PID_LOOP_ID,
    intervals,
    length_array(intervals),
    TIMELINE_ABSOLUTE,
    TIMELINE_REPEAT
)

enabled = TRUE

define_function set_min_output(float value){
    min_output = value
    if (min_output > max_output) min_output = max_output
    if (min_output > output) output = min_output
    if (min_output > i_error) i_error = min_output
}

define_function set_max_output(float value){
    max_output = value
    if (max_output < min_output) max_output = min_output
    if (max_output < output) output = max_output
    if (max_output < i_error) i_error = max_output
}

define_function set_interval(slong value){
    stack_var float new_interval
    stack_var float old_interval
    stack_var float ratio
    new_interval = type_cast(value)
    old_interval = type_cast(intervals[1])
    ratio = new_interval / old_interval
    ki = ki * ratio
    kd = kd / ratio
    intervals[1]  = type_cast(new_interval)
    timeline_reload(PID_LOOP_ID, intervals, length_array(intervals))     
}

define_function re_init(){
    last_input = input
    i_error = output
     if (min_output > i_error) i_error = min_output
     else if (max_output < i_error) i_error = max_output
}

define_function enable(){
    if (!enabled){
        timeline_restart(PID_LOOP_ID)
        enabled = TRUE
        re_init()
    }   
}

define_function disable(){
    if (enabled){
        timeline_pause(PID_LOOP_ID)
        enabled = FALSE
    }
}

define_event

data_event[this]{
    command:{
        stack_var char command_string[16]
        stack_var float f_value
        stack_var slong l_value
        command_string = remove_string(data.text, ' ', 1)
        f_value = atof(data.text)
        l_value = atol(data.text)
        switch (command_string){
            case 'set_Kp': Kp = f_value
            case 'set_Ki': Ki = f_value
            case 'set_Kd': Kd = f_value
            case 'set_min_output': set_min_output(f_value)
            case 'set_max_output': set_max_output(f_value)
            case 'set_interval': set_interval(l_value)
            case 'set_enable': enable()
            case 'set_disable': disable()
        }
    }
}

timeline_event[PID_LOOP_ID]{
    error = setpoint - input
    i_error = i_error + ki * error
    if  (i_error > max_output) i_error = max_output
        else if (i_error < min_output) i_error = min_output
    d_input   = input - last_input
    output = kp * error +  i_error + kd * d_input
    if  (output > max_output) output = max_output
        else if (output < min_output) output = min_output
    last_input = input
    send_string this, ftoa(output)
}