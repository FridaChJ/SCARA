#------------------------------------------------------------------------------
#---------------------------INIT-----------------------------------------------
#------------------------------------------------------------------------------
#==========================IMPORTS===========================================
#Import and define classes so can be later used in code
import time #Used ofr delays, timing loops, or measuring performance
import json #Essential for sending/receiving structured data
import threading #Useful when you do not want one task to block everything else
from mqtt_client import MQTTClient #Import MQTT class
#from kinematics import Kinematics #Import Kinematics class
#==========================CONSTANTS===========================================
#-------------------------MQTT Topics-----------------------------------------
COMMANDS      = "robot/commands"
ENCODERS      = "robot/encoders"
MOTOR_ANGLES  = "robot/motor_angles"
#-------------------------Loop timing-----------------------------------------
#Defines the loop speed
"""
  What if this didn't exist?
    - Programm runs as fast as possible, unstable
    - Different speed every iteration
    - Bad for control systems
  This ensures that the loops follow the same frequency, are more predictable and stable
"""
LOOP_HZ       = 50 #Loop runs 50 times per seconds
LOOP_PERIOD   = 1.0 / LOOP_HZ #Timer per loop, eachloop takes 20 ms
#-------------------------GUI Timeout-----------------------------------------
#If no command arrives from the GUI, manual mode activates to allow to set angles to each joint via terminal
GUI_TIMEOUT   = 2.0 
#=============================VARIABLES ===========================================
#---------------------------Shared States----------------------------------------
#GUI mode. We create the variables that will later hold the values sent by MQTT
current_command         = "idle"  #Receives the commands from GUI
current_encoder_angles  = {"j1": 0.0, "j2": 0.0, "j3": 0.0, "j4": 0.0} # Dictionary that stores the latest encoder angles (received via MQTT)
last_gui_time           = 0.0 #Stores when the last commands was received
#Manual mode
manual_target_angles    = None
manual_lock             = threading.Lock() #Prevents two threads from modifying data at the same time
#---------------------------MQTT Callbacks----------------------------------------
# MQTT callbacks update the variables when new data arrives
def on_command(command:str) -> None: #This function returns nothing
  global current_command, last_gui_time #Global used to modigy the variables outside the function and avoid that the function creates local variables
  current_command = command #Stores new command
  last_gui_time   = time.time() #Stores when last command was received

def on_encoders(angles:dict) -> None: #This function returns nothing
  global current_encoder_angles
  current_encoder_angles = angles #Stores new angles

def on_message(topic: str, payload: dict):
    if topic == COMMANDS:
        on_command(payload ["command"])
    elif topic == ENCODERS:
        on_encoders(payload)
#------------------------------------------------------------------------------
#---------------------------MANUAL MODE-----------------------------------------------
#------------------------------------------------------------------------------
#==========================CONSTANTS===========================================

JOINT_MAP = {
    "j1": "j1",
    "j2": "j2",
    "j3": "j3",
    "j4": "j4",
}
ANGLE_MIN = -360.0
ANGLE_MAX  =  360.0
#==========================FUNCTIONS===========================================

def parse_manual_input(line:str, current_angles: dict) -> dict | None:
  """
      Returns a full MotorAngles dict with the specified joints updated,
      or None if the line is empty or unparseable.
  """
  line = line.strip() #Remove spaces
  if not line:
      return None
  #Start from the last known angles so unspecified joints hold position
  updated    = dict(current_angles) #Starts from current position
  parsed_any = False

  for token in line.split(): #Separates the line in tokens and iterates in each of them
    if ":" not in token:
      print(f"[MANUAL] Missing : in: '{token}'")
      continue
    
    key, _, raw_value = token.partition(":") #Makes this ("j1", ":", "45")
    key = key.lower().strip() #Formats the key 

    if key not in JOINT_MAP:
      print(f"[MANUAL] Unknown joint '{key}'. Valid: {list(JOINT_MAP.keys())}")
      continue
    delta = None
    try:
      delta = float(raw_value)
    except ValueError:
      print(f"[MANUAL] '{raw_value}' is not a valid number for '{key}'")
    
    new_angle  = current_angles[JOINT_MAP[key]] + delta
    clamped    = max(ANGLE_MIN, min(ANGLE_MAX, new_angle))

    if clamped != new_angle:
        print(f"  [Manual] '{key}' clamped: {new_angle:.2f} → {clamped:.2f}")

    updated[JOINT_MAP[key]] = clamped
    parsed_any = True
  return updated if parsed_any else None

def print_manual_help() -> None:
  print()
  print("  ┌─ Manual joint control ──────────────────────────────────┐")
  print("  │ Format:  j1:angle  j2:angle  j3:angle  j4:angle         │")
  print("  │  COMMANDS                                               │")
  print("  │   home   → send all joints to 0.0                       │")
  print("  │   status  → print current encoder angles                │")
  print("  │   instructions    → show this message                   │")
  print("  └─────────────────────────────────────────────────────────┘")
  print()

#Terminal input thread
def terminal_input_thread() -> None:
   """
    Runs in a background thread.
    Reads lines from stdin and updates manual_target_angles.
    The main loop only applies these values when the GUI is inactive.
    """
   global manual_target_angles #Uses a variable that is outside the function
   print_manual_help() 

   while True:
      try:
        line = input() #Reads line input
      except EOFError: #Except if End of File reached, no more input can be read
        break #Gets out to avoid program to crash

      stripped = line.strip().lower() #Formats the line
      if stripped == 'home': #If home is typed
        with manual_lock: #Applies lock to avoid that another thread modifies this data
          manual_target_angles = {"s1": 0.0, "s2": 0.0, "dc1": 0.0, "dc2": 0.0} #Sets angles to the home position
          print("  [Manual] All joints reset to HOME")
          continue
      elif stripped == 'status': #If status is typed
        print(f"[MANUAL] Position angles: {current_encoder_angles}") #Prints current angles
        continue
      elif stripped in ('instructions', '?', 'help'): #If either of those str are typed
        print_manual_help() #Prints help manual
        continue
      else: #Case were the angles are typed
        parsed = parse_manual_input(stripped, current_encoder_angles) #Checks if format id right and returns dict with angles
        if parsed: #Ifparse is TRUE
          with manual_lock: #Applies lock to avoid that another thread modifies this data
            manual_target_angles = parsed 
          print(f"[MANUAL] Angles set: {parsed}")

#-------------------------------------------------------------------------
#------------------------Main-------------------------
#-------------------------------------------------------------------------
def main() -> None:
  global manual_target_angles  # ← add this line
  #Instantiate main classes
  mqtt = MQTTClient(
    broker_ip            ="192.168.4.87", 
    port                 =1883,
    username             ="Friii",
    password             ="12345678",
    on_message_callback  = on_message
  )
  #Kinematics = Kinematics()
  
  #MQTT Handler
  mqtt.connect() #Connect to broker
  mqtt.subscribe(COMMANDS) #Susbscribe to hear commands from GUI
  mqtt.subscribe(ENCODERS) #Subscribe to hear encoders data from ESP

  #Start Terminal Input Thread
  input_thread = threading.Thread(target=terminal_input_thread, daemon=True) #Calls input_thread
  input_thread.start() #Starts thread
  print(f"[BRAIN] Running at {LOOP_HZ} Hz") #Logs in terminal
  print(f"[RAIN] GUI timeout: {GUI_TIMEOUT}s — manual mode activates when GUI goes silent") #Logs in terminal
  #=============================MAIN LOOP ===========================================
  while True:
    loop_start = time.time()
    gui_active = (time.time() - last_gui_time) < GUI_TIMEOUT #Calculates the tme that the GUI has not sent any commands

    if gui_active: #If GUI is sending commands
      #GUI mode
      #target_angles = kinematics.compute_angles(current_command, current_encoder_angles) #Set target angles to the ones computed by kinematics
      pass
    else: #If GUI stops sending commands fro more than 2s
      #Manual mode
      with manual_lock:
        if manual_target_angles is not None: #If there is no manual input yet
          target_angles = manual_target_angles
          manual_target_angles = None
          mqtt.publish(MOTOR_ANGLES,  target_angles) #sends target angles to broker
        #else:
          #No manual input yet, hold current position
          #target_angles = dict(current_encoder_angles) IN CASE THAT THE MOTOR DO NOT HOLD CHECK THIS
         # pass
    
    

    elapsed    = time.time() - loop_start
    sleep_time = LOOP_PERIOD - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)


if __name__ == "__main__":
    main()
