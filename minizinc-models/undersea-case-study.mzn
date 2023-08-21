% 8/21/2023 undersea case study (autonomous underwater vehicle)
% this formulation is changed in that the speed change is not considered

%%%%%%%%%%%%%
% CONSTANTS %
%%%%%%%%%%%%%

int: sensor_count = 3; % total number of sensors
int: look_ahead = 5; % step for lookahead (each step represents a second)

float: sensor_1_energy_reading = 3;
float: sensor_2_energy_reading = 2.4;
float: sensor_3_energy_reading = 2.1;

float: sensor_1_energy_on = 10;
float: sensor_2_energy_on = 8;
float: sensor_3_energy_on = 5;

float: sensor_1_energy_off = 2;
float: sensor_2_energy_off = 1.5;
float: sensor_3_energy_off = 1;

int: sensor_1_frequency = 5;
int: sensor_2_frequency = 4;
int: sensor_3_frequency = 4;

%%%%%%%%%%%%%%%
% UNCERTAINTY %
%%%%%%%%%%%%%%%

% environment uncertainties
% current status s_0 (0: failed, 1: operational)
% after the failure, the sensor cease to be operational. Cost energy when turn on/off
array[1..sensor_count] of 0..1: current_sensor_status = [1,1,1];

%%%%%%%%%%%
% ACTIONS %
%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%
% 0: turn off sensor
% 1: turn on sensor
% 2: no operation
%%%%%%%%%%%%%%%%%%%%%%


% action of the sensors
% action list needs to allow simultanous actions across different sensors
array[1..look_ahead - 1] of var 0..2: sensor_1_actions;
array[1..look_ahead - 1] of var 0..2: sensor_2_actions;
array[1..look_ahead - 1] of var 0..2: sensor_3_actions;

%%%%%%%%%%
% STATES %
%%%%%%%%%%

% operation states of the sensors (0: on, 1: off)
array[1..look_ahead] of var 0..1: sensor_1_on_off_states;
array[1..look_ahead] of var 0..1: sensor_2_on_off_states;
array[1..look_ahead] of var 0..1: sensor_3_on_off_states;

% energy consumption states of the sensors
% note that the floating point must be bounded
array[1..look_ahead] of var 0.0..20.0: sensor_1_energy_states;
array[1..look_ahead] of var 0.0..20.0: sensor_2_energy_states;
array[1..look_ahead] of var 0.0..20.0: sensor_3_energy_states;

% freqency states of the sensors 
array[1..look_ahead] of var 0..100: sensor_1_frequency_states;
array[1..look_ahead] of var 0..100: sensor_2_frequency_states;
array[1..look_ahead] of var 0..100: sensor_3_frequency_states;

%%%%%%%%%%%%%%%%%%
% INITIAL STATES %
%%%%%%%%%%%%%%%%%%

% define initial states
% all sensors are turned on
constraint sensor_1_on_off_states[1] = 1;
constraint sensor_2_on_off_states[1] = 1;
constraint sensor_3_on_off_states[1] = 1;

%%%%%%%%%%
% MODELS %
%%%%%%%%%%

predicate model(var 0..2: action, 
var 0..1: curr_on_off_state, var 0..1: next_on_off_state, 
var float: curr_energy_state, var float: next_energy_state,
var 0..100: curr_frequency_state, var 0..100: next_frequency_state, 
var float: energy_on, var float: energy_off, var float: energy_reading, var int: sensor_frequency) =
  
  % case when sensor is shut off
  if action = 0 then (
    next_on_off_state = 0 /\
    next_energy_state = energy_off /\
    next_frequency_state = 0 % degraded service
  )
  % case when the sensor is turned on
  elseif action = 1 then (
    next_on_off_state = 1 /\
    next_energy_state = energy_on + energy_reading /\
    next_frequency_state = sensor_frequency
  )
  % no operation (action = 2), frequency and energy consumption remains the same
  else (
    next_on_off_state = curr_on_off_state
     /\
    if curr_on_off_state = 0 then (
      next_energy_state = 0 /\
      next_frequency_state = 0
    )
    else (
      next_energy_state = energy_reading /\
      next_frequency_state = sensor_frequency
    )
    endif
  )
  endif;
    
% propagate for sensor 1
constraint
  forall (idx in 1..look_ahead - 1) (
    model(sensor_1_actions[idx], 
                 sensor_1_on_off_states[idx],
                 sensor_1_on_off_states[idx + 1],
                 sensor_1_energy_states[idx],
                 sensor_1_energy_states[idx + 1],
                 sensor_1_frequency_states[idx],
                 sensor_1_frequency_states[idx + 1],
                 sensor_1_energy_on,
                 sensor_1_energy_off,
                 sensor_1_energy_reading,
                 sensor_1_frequency
                 )
                 
                 /\
                 
    model(sensor_2_actions[idx], 
                 sensor_2_on_off_states[idx],
                 sensor_2_on_off_states[idx + 1],
                 sensor_2_energy_states[idx],
                 sensor_2_energy_states[idx + 1],
                 sensor_2_frequency_states[idx],
                 sensor_2_frequency_states[idx + 1],
                 sensor_2_energy_on,
                 sensor_2_energy_off,
                 sensor_2_energy_reading,
                 sensor_2_frequency
                 )
                /\
                
    model(sensor_3_actions[idx], 
                 sensor_3_on_off_states[idx],
                 sensor_3_on_off_states[idx + 1],
                 sensor_3_energy_states[idx],
                 sensor_3_energy_states[idx + 1],
                 sensor_3_frequency_states[idx],
                 sensor_3_frequency_states[idx + 1],
                 sensor_3_energy_on,
                 sensor_3_energy_off,
                 sensor_3_energy_reading,
                 sensor_3_frequency
                 )
             
   );
   
   
% output ["\(current_sensor_states)"]

% requirements
% G_[0, 5](readings > 20) % aka, sensor freq > 4 Hz
% G_[0, 5](cumulative energy consumption <= 120 J) % aka, sensor energy consumption < 24 J / s

% constraints 1: The UUV should take at least 20 readings of sufficient accuracy per 10 meters of mission distance 
% arraysensor_failure_list 
% declare action list
% array[1..SIGNAL_LENGTH - 1] of var 0..1: action_list;
