import pygame
import numpy as np
import time
from enum import Enum
import threading
import os
from queue import Queue
from vehicle_parser import load_all_vehicles, VehicleConfig

# Initialize pygame mixer with lower frequency for better control
pygame.mixer.init(frequency=22050, size=-16, channels=2) # Increased channels to 2 for engine sounds

class EngineState(Enum):
    OFF = 0
    STARTING = 1
    RUNNING = 2
    STOPPING = 3
    PARKING_BRAKE = 4

class AudioEngine:
    def __init__(self):
        # Create audio channels
        self.idle_channel = pygame.mixer.Channel(0) # Channel for idle sound
        self.rev_channel = pygame.mixer.Channel(1)  # Channel for rev sound
        self.effects_channel = pygame.mixer.Channel(2) # Channel for effects
        self.horn_channel = pygame.mixer.Channel(3)  # Channel for horn
        self.siren_channel = pygame.mixer.Channel(4) # Channel for siren
        
        # Sound buffers (loaded as pygame.mixer.Sound objects now)
        self.sound_buffers = {}
        self.current_vehicle = None
        
        # Engine state
        self.engine_state = EngineState.OFF
        self.engine_on = False
        self.engine_start = False
        self.engine_running = False
        self.engine_stop = False
        
        # Physics parameters
        self.throttle = 0.0
        self.current_rpm = 0
        self.target_rpm = 0
        # Default acc/dec, will be overwritten by vehicle config
        self.acceleration = 5 
        self.deceleration = 5
        self.engine_mass = 1000  # kg (less relevant for this physics model)
        self.dt = 0.01  # 100Hz physics update
        
        # Volume control
        self.master_volume = 100 # 0-100 scale
        # Volumes derived from config and throttle/RPM
        self.idle_volume = 0.0 # 0.0-1.0 scale for Pygame channel
        self.rev_volume = 0.0  # 0.0-1.0 scale for Pygame channel
        
        # Thread control
        self.running = True
        self.physics_thread = None
        self.debug_thread = None # Add debug thread attribute
        self.sound_events = Queue() # For potential future use with effects
        
        # Debug variables (to be updated by update_volumes_and_pitch and printed by debug thread)
        self._debug_total_target_volume = 0.0
        self._debug_idle_volume = 0.0
        self._debug_rev_volume = 0.0
        # Store blend points and blends for debug too
        self._debug_rev_switch = 0
        self._debug_idle_end = 0
        self._debug_idle_blend = 0.0
        self._debug_rev_blend = 0.0
        
        # Sample rate control (less directly used with Pygame channels, pitch is preferred)
        self.sample_rate = 22050
        self.max_sample_interval = 4000000 / self.sample_rate # From ESP32 code
        self.min_sample_interval = self.max_sample_interval * 100 / 500 # From ESP32 code (MAX_RPM_PERCENTAGE = 500)
        self.current_sample_interval = self.max_sample_interval
        
        # Pitch control parameters (tuned examples)
        self.min_pitch = 0.8 # Pitch at idle RPM
        self.max_pitch = 1.5 # Pitch at max RPM
        self.base_rpm_for_pitch = 50 # RPM considered idle for pitch scaling
        self.max_rpm_for_pitch = 500 # Max RPM for pitch scaling


    def load_vehicle(self, vehicle_config):
        """Load all sounds for a vehicle and set vehicle-specific parameters."""
        self.current_vehicle = vehicle_config
        self.sound_buffers.clear()
        
        # Set vehicle-specific physics parameters
        self.acceleration = getattr(vehicle_config, 'acc', 5) # Default to 5 if not in config
        self.deceleration = getattr(vehicle_config, 'dec', 5) # Default to 5 if not in config
        
        # Stop any currently playing sounds on engine channels
        self.idle_channel.stop()
        self.rev_channel.stop()
        
        # Load engine sounds (idle and rev)
        idle_sound_name = getattr(vehicle_config, 'idle_sound', None)
        rev_sound_name = getattr(vehicle_config, 'rev_sound', None)
        has_rev_sound_flag = getattr(vehicle_config, 'has_rev_sound', False)

        if idle_sound_name:
            idle_sound_path = os.path.join("sounds", idle_sound_name)
            if os.path.exists(idle_sound_path):
                try:
                    self.sound_buffers['idle'] = pygame.mixer.Sound(idle_sound_path)
                    print(f"Loaded idle sound: {idle_sound_name}")
                except Exception as e:
                    print(f"Error loading idle sound {idle_sound_name}: {e}")
            else:
                 print(f"Error: Idle sound file not found: {idle_sound_path}")
        else:
             print("Warning: No idle sound defined for this vehicle.")
             
        # Load rev sound only if defined and flag is set
        if has_rev_sound_flag and rev_sound_name:
             rev_sound_path = os.path.join("sounds", rev_sound_name)
             if os.path.exists(rev_sound_path):
                 try:
                     self.sound_buffers['rev'] = pygame.mixer.Sound(rev_sound_path)
                     print(f"Loaded rev sound: {rev_sound_name}")
                     print(f"[Sound Load Status] Rev sound loading: Success - {rev_sound_name}") # Debug print
                 except Exception as e:
                    print(f"Error loading rev sound {rev_sound_name}: {e}")
                    print(f"[Sound Load Status] Rev sound loading: Failed - {rev_sound_name} ({e})") # Debug print
             else:
                print(f"Error: Rev sound file not found: {rev_sound_path}")
                print(f"[Sound Load Status] Rev sound loading: File Not Found - {rev_sound_path}") # Debug print
        elif rev_sound_name and not has_rev_sound_flag:
             print(f"Info: Rev sound '{rev_sound_name}' defined but not enabled (REV_SOUND not defined in vehicle config).")
             print(f"[Sound Load Status] Rev sound loading: Disabled by config - {rev_sound_name}") # Debug print
        else:
             # No rev sound defined in config
             print("[Sound Load Status] Rev sound loading: Not defined in vehicle config.") # Debug print

        # Load other sounds
        for sound_type in ['start', 'horn', 'siren']:
             sound_name = getattr(vehicle_config, f"{sound_type}_sound", None)
             if sound_name:
                sound_path = os.path.join("sounds", sound_name)
                if os.path.exists(sound_path):
                     try:
                        self.sound_buffers[sound_type] = pygame.mixer.Sound(sound_path)
                        print(f"Loaded {sound_type} sound: {sound_name}")
                     except Exception as e:
                         print(f"Error loading {sound_type} sound {sound_name}: {e}")
                else:
                     print(f"Error: {sound_type.capitalize()} sound file not found: {sound_path}")
             # else: print(f"Info: No {sound_type} sound defined for this vehicle.") # Optional debug

        # Check if essential engine sounds are loaded
        if 'idle' not in self.sound_buffers:
             print("Warning: Idle sound not loaded. Engine sound will not work.")
        # Note: Rev sound is optional, engine will use idle sound with pitch/volume if rev is missing.

    def update_physics(self):
        """Update engine physics simulation."""
        # Debug print: Show throttle and current RPM at the start of update_physics
        # print(f"[Physics] Throttle: {self.throttle:.2f}, Current RPM: {self.current_rpm}")

        if self.engine_state == EngineState.RUNNING and self.current_vehicle:
            # Calculate target RPM based on throttle (linear map 0-500 like ESP32 example)
            # Map throttle from 0.0-1.0 to 0-500 range
            target_rpm = int(self.throttle * 500)
            # Ensure minimum target RPM is idle RPM if above 0 throttle
            # Assuming idle RPM is 50 based on previous analysis, could be vehicle config parameter
            idle_rpm_target = getattr(self.current_vehicle, 'idle_rpm_target', 50)
            if self.throttle > 0:
                 target_rpm = max(idle_rpm_target, target_rpm)
            else:
                 target_rpm = idle_rpm_target # At zero throttle, target is idle RPM

            # Get acceleration/deceleration values from instance (loaded from config)
            acc = self.acceleration
            dec = self.deceleration

            # Engine RPM Acceleration / Deceleration (Core Inertia Simulation) similar to ESP32
            rpm_diff = target_rpm - self.current_rpm

            if rpm_diff > 0:
                 # Accelerate engine
                self.current_rpm = min(self.current_rpm + acc, target_rpm)
            elif rpm_diff < 0:
                 # Decelerate engine
                self.current_rpm = max(self.current_rpm - dec, target_rpm)

            # Ensure minimum idle RPM when engine is running
            # This check is important if dec makes RPM drop below idle_rpm_target even if target is > idle_rpm_target
            if self.current_rpm < idle_rpm_target and self.throttle > 0:
                 self.current_rpm = idle_rpm_target
            # If throttle is 0 and RPM drops, let it go to the true idle target

            # Debug print: Show target RPM and updated current RPM
            # print(f"[Physics] Target RPM: {target_rpm}, Updated Current RPM: {self.current_rpm}")

        elif self.engine_state == EngineState.STOPPING:
             # Decelerate to 0 RPM when stopping
            dec = self.deceleration # Use deceleration value from instance
            self.current_rpm = max(self.current_rpm - dec, 0)
            if self.current_rpm == 0:
                 # Transition to OFF state once RPM reaches 0
                 self.engine_state = EngineState.OFF
                 self.engine_running = False
                 # Stop all engine sound channels
                 self.idle_channel.stop()
                 self.rev_channel.stop()
                 print("Engine off.")

        else:
            self.current_rpm = 0 # RPM is 0 when OFF or STARTING (before running)

    def update_volumes_and_pitch(self):
        """Update volume and pitch levels for engine sounds based on throttle and RPM.
           This implements the blending/crossfading using channel volumes.
        """
        # Debug print: Show throttle and current RPM at the start of update_volumes_and_pitch
        # print(f"[AudioUpdate] Throttle: {self.throttle:.2f}, Current RPM: {self.current_rpm}") # Moved to debug thread

        if not self.current_vehicle or not self.engine_running:
            # Set volumes to 0 when engine is off
            self.idle_channel.set_volume(0)
            self.rev_channel.set_volume(0)
            # print("[AudioUpdate] Engine not running, volumes set to 0.") # Moved to debug thread
            return

        # Calculate throttle and RPM dependent volumes (similar to ESP32 mapThrottle)
        throttle_500 = int(self.throttle * 500) # Scale throttle 0-1 to 0-500 range
        throttle_500 = max(0, min(500, throttle_500)) # Ensure bounds

        # Get volume percentages from vehicle config
        engine_idle_vol_perc = getattr(self.current_vehicle, 'engine_idle_volume', 100)
        full_throttle_vol_perc = getattr(self.current_vehicle, 'full_throttle_volume', 100)
        engine_rev_vol_perc = getattr(self.current_vehicle, 'engine_rev_volume', 100)
        idle_base_vol_perc = getattr(self.current_vehicle, 'idle_volume', 100)
        rev_base_vol_perc = getattr(self.current_vehicle, 'rev_volume', 100)

        # Interpolate throttle-dependent engine volumes (0-100 scale)
        # ESP32 maps throttle 0-500 to engineIdleVolumePercentage-fullThrottleVolumePercentage
        calc_engine_idle_vol = np.interp(throttle_500, [0, 500], [engine_idle_vol_perc, full_throttle_vol_perc])
        calc_engine_rev_vol = np.interp(throttle_500, [0, 500], [engine_rev_vol_perc, full_throttle_vol_perc])

        # Determine the blend ratio between idle and rev sounds based on RPM
        # ESP32 uses revSwitchPoint and idleEndPoint (get from vehicle config, use defaults if not present)
        rev_switch_point = getattr(self.current_vehicle, 'revSwitchPoint', 280)
        idle_end_point = getattr(self.current_vehicle, 'idleEndPoint', 300)

        idle_blend = 0.0
        rev_blend = 0.0

        blending_rpm = max(0, min(500, self.current_rpm)) # Ensure current_rpm is within 0-500 range for blending

        if blending_rpm < rev_switch_point:
            # Mostly idle below revSwitchPoint
            idle_blend = 1.0
            rev_blend = 0.0
        elif blending_rpm < idle_end_point:
            # Blend between revSwitchPoint and idleEndPoint (linear interpolation)
            blend_range = idle_end_point - rev_switch_point
            if blend_range > 0:
                rev_weight = (blending_rpm - rev_switch_point) / float(blend_range)
                idle_blend = 1.0 - rev_weight
                rev_blend = rev_weight
            else:
                # Should not happen, default to full idle
                idle_blend = 1.0
                rev_blend = 0.0
        else:
            # Mostly rev above idleEndPoint
            idle_blend = 0.0
            rev_blend = 1.0

        # --- C++ mapThrottle inspired Volume Calculation ---
        # Calculate throttle-dependent base volumes for idle and rev sounds separately (0-100 scale)
        # Similar to throttleDependentVolume and throttleDependentRevVolume in C++
        throttle_dependent_idle_vol_perc = np.interp(throttle_500, [0, 500], [engine_idle_vol_perc, full_throttle_vol_perc])
        throttle_dependent_rev_vol_perc = np.interp(throttle_500, [0, 500], [engine_rev_vol_perc, full_throttle_vol_perc])

        # Scale these throttle-dependent volumes by master volume and convert to Pygame's 0.0-1.0 scale
        scaled_throttle_idle_vol = (throttle_dependent_idle_vol_perc / 100.0) * (self.master_volume / 100.0)
        scaled_throttle_rev_vol = (throttle_dependent_rev_vol_perc / 100.0) * (self.master_volume / 100.0)

        # Apply the RPM-based blending to the scaled throttle-dependent volumes
        # This is conceptually similar to how the blended amplitude is calculated in the C++ main loop
        final_idle_volume_pygame_scale = scaled_throttle_idle_vol * idle_blend
        final_rev_volume_pygame_scale = scaled_throttle_rev_vol * rev_blend

        # Constrain volumes to Pygame's range (0.0 to 1.0)
        final_idle_volume_pygame_scale = max(0.0, min(1.0, final_idle_volume_pygame_scale))
        final_rev_volume_pygame_scale = max(0.0, min(1.0, final_rev_volume_pygame_scale))

        # Set volumes on the channels
        self.idle_channel.set_volume(final_idle_volume_pygame_scale)
        # Only set rev channel volume if rev sound is loaded
        if 'rev' in self.sound_buffers:
            self.rev_channel.set_volume(final_rev_volume_pygame_scale)
        else:
             self.rev_channel.set_volume(0) # Ensure rev channel is silent if no rev sound

        # --- Store debug values for debug thread ---
        self._debug_total_target_volume = final_idle_volume_pygame_scale + final_rev_volume_pygame_scale
        self._debug_idle_volume = final_idle_volume_pygame_scale
        self._debug_rev_volume = final_rev_volume_pygame_scale
        self._debug_rev_switch = rev_switch_point
        self._debug_idle_end = idle_end_point
        self._debug_idle_blend = idle_blend
        self._debug_rev_blend = rev_blend

        # Debug printing (optional) - Now handled by debug_printer_thread_func
        # print(f"[DEBUG 2Hz] RPM: {self.current_rpm}, Thr: {self.throttle:.2f}, RevSwitch: {rev_switch_point}, IdleEnd: {idle_end_point}, IdleBlend: {idle_blend:.2f}, RevBlend: {rev_blend:.2f}, RawTarget%: {raw_target_engine_vol_perc:.1f}, TotalTargetVol: {self._debug_total_target_volume:.2f}, IdleVol: {final_idle_volume_pygame_scale:.2f}, RevVol: {final_rev_volume_pygame_scale:.2f}")

    def debug_printer_thread_func(self):
        """Prints debug information at a controlled rate (20Hz)."""
        debug_rate = 20 # Hz - Increased debug rate
        sleep_time = 1.0 / debug_rate

        # Open log file in write mode to clear it on each run
        log_file_path = "log.txt"
        try:
            with open(log_file_path, 'w') as log_file:
                log_file.write("RC Engine Sound Simulator Debug Log\n")
                log_file.write("----------------------------------\n")
                while self.running:
                    if self.current_vehicle and (self.engine_state == EngineState.RUNNING or self.engine_state == EngineState.STOPPING):
                        # Read calculated values from instance variables and write to file
                        log_entry = f"[DEBUG 20Hz] RPM: {self.current_rpm}, Thr: {self.throttle:.2f}, RevSwitch: {self._debug_rev_switch}, IdleEnd: {self._debug_idle_end}, IdleBlend: {self._debug_idle_blend:.2f}, RevBlend: {self._debug_rev_blend:.2f}, TotalTargetVol: {self._debug_total_target_volume:.2f}, IdleVol: {self._debug_idle_volume:.2f}, RevVol: {self._debug_rev_volume:.2f}\n"
                        log_file.write(log_entry)
                        log_file.flush() # Ensure data is written to disk immediately

                    # Wait for the next debug print interval
                    time.sleep(sleep_time)

                log_file.write("[DEBUG 20Hz] Debug printer thread stopping.\n")
        except Exception as e:
            print(f"Error writing to log.txt: {e}")

        # print("[DEBUG 2Hz] Debug printer thread stopping.") # This will no longer be printed to console

    def physics_thread_func(self):
        """Main physics and audio update loop."""
        while self.running:
            # Only run physics and audio updates if a vehicle is selected
            if self.current_vehicle:
                # State machine logic
                if self.engine_state == EngineState.OFF:
                    if self.engine_on:
                         self.engine_state = EngineState.STARTING
                         print("Engine state: STARTING")
                         # Play start sound on engine channel (or a dedicated channel)
                         if 'start' in self.sound_buffers:
                             start_sound = self.sound_buffers['start']
                             # Use idle channel for start sound for simplicity in this model
                             self.idle_channel.play(start_sound)
                             print("Playing start sound.")
                         else:
                             print("Start sound not loaded.")
                         self.current_rpm = 0 # Ensure RPM is 0 at start of starting

                elif self.engine_state == EngineState.STARTING:
                    # Wait for start sound to finish (check the channel playing it)
                    if 'start' in self.sound_buffers:
                        if not self.idle_channel.get_busy(): # Check the channel where start sound was played
                            self.engine_state = EngineState.RUNNING
                            self.engine_running = True
                            print("Engine state: RUNNING")
                            # Start playing idle and rev sounds looping
                            if 'idle' in self.sound_buffers:
                                self.idle_channel.play(self.sound_buffers['idle'], loops=-1)
                                print("Started playing idle sound loop.")
                            if 'rev' in self.sound_buffers:
                                # Play rev sound loop on its channel
                                self.rev_channel.play(self.sound_buffers['rev'], loops=-1)
                                print("Started playing rev sound loop.")
                                # Ensure volumes and pitch are updated immediately upon entering RUNNING state
                                self.update_physics() # Update RPM to idle level
                                self.update_volumes_and_pitch() # Set initial volumes and pitch
                        else:
                            # If no start sound, transition directly to RUNNING after a small delay
                            time.sleep(0.5) # Simulate a brief startup time
                            self.engine_state = EngineState.RUNNING
                            self.engine_running = True
                            print("Engine state: RUNNING (no start sound)")
                            # Start playing idle and rev sounds
                            if 'idle' in self.sound_buffers:
                                self.idle_channel.play(self.sound_buffers['idle'], loops=-1)
                                print("Started playing idle sound loop.")
                            if 'rev' in self.sound_buffers:
                                self.rev_channel.play(self.sound_buffers['rev'], loops=-1)
                                print("Started playing rev sound loop.")
                            self.update_physics()
                            self.update_volumes_and_pitch()

                elif self.engine_state == EngineState.RUNNING:
                    # Update physics and audio parameters continuously
                    self.update_physics()
                    self.update_volumes_and_pitch() # Update volumes and pitch based on RPM/throttle

                    if not self.engine_on:
                        self.engine_state = EngineState.STOPPING
                        print("Engine state: STOPPING")
                        # physics will handle RPM decay to 0 and transition to OFF

                elif self.engine_state == EngineState.STOPPING:
                     # Physics will handle RPM decay and transition to OFF
                     self.update_physics()
                     self.update_volumes_and_pitch() # Continue updating volumes/pitch as RPM decays
                     # Optionally play a stopping sound here if available
                     # Ensure engine sound stops when RPM hits 0 in update_physics -> OFF transition

                elif self.engine_state == EngineState.PARKING_BRAKE:
                     # This state doesn't seem used in Python state machine, can refine or remove
                     pass # Currently does nothing

            else:
                # If no vehicle is selected, ensure engine channels are stopped
                if self.idle_channel.get_busy():
                     self.idle_channel.stop()
                if self.rev_channel.get_busy():
                     self.rev_channel.stop()
                # print("[AudioEngine] No vehicle selected, engine channels stopped.") # Optional debug
                # Keep other channels (effects, horn, siren) as they might be used independently

            # Control thread update rate
            time.sleep(self.dt) # physics update rate (e.g., 100Hz if dt is 0.01)

    def start(self):
        """Start the audio engine."""
        # Ensure sounds directory exists for loading
        sounds_dir = os.path.join(os.path.dirname(__file__), "sounds")
        if not os.path.exists(sounds_dir):
             print(f"Error: 'sounds' directory not found at {sounds_dir}. Please run sound_converter.py first.")
             return False # Indicate failure to start

        # Ensure enough Pygame mixer channels are available
        if pygame.mixer.get_num_channels() < 5: # Need at least 5 channels for idle, rev, effects, horn, siren
            print(f"Warning: Pygame mixer only has {pygame.mixer.get_num_channels()} channels. Attempting to set to 5.")
            try:
                pygame.mixer.set_num_channels(5)
                print("Pygame mixer channels set to 5.")
            except Exception as e:
                print(f"Error setting Pygame mixer channels: {e}. Some sounds may not play.")

        self.running = True # Ensure running flag is True before starting thread
        self.physics_thread = threading.Thread(target=self.physics_thread_func)
        self.physics_thread.daemon = True # Allow program to exit even if thread is running
        self.physics_thread.start()

        # Start the debug printer thread
        self.debug_thread = threading.Thread(target=self.debug_printer_thread_func)
        self.debug_thread.daemon = True # Allow program to exit even if thread is running
        self.debug_thread.start()

        return True # Indicate success

    def stop(self):
        """Stop the audio engine."""
        self.running = False
        # Use a timeout on join to prevent the main thread from hanging indefinitely
        if self.physics_thread and self.physics_thread.is_alive():
            self.physics_thread.join(timeout=self.dt * 5) # Give it a short time to finish current loop
            if self.physics_thread.is_alive():
                print("Warning: Physics thread did not terminate gracefully.")

        # Signal debug thread to stop and join it
        if self.debug_thread and self.debug_thread.is_alive():
             # The debug thread has a sleep, so a short join timeout should work
             self.debug_thread.join(timeout=1.0) # Give it up to 1 second to stop
             if self.debug_thread.is_alive():
                 print("Warning: Debug printer thread did not terminate gracefully.")

        # Stop all channels before quitting mixer
        pygame.mixer.stop()
        pygame.mixer.quit()
        print("Pygame mixer quit.")

    def toggle_engine(self):
        """Toggle engine state."""
        if not self.current_vehicle:
            print("Please select a vehicle first")
            return
            
        # Check if idle sound is loaded before allowing engine to start
        if not self.engine_on and 'idle' not in self.sound_buffers:
             print("Engine cannot start: Idle sound not loaded for this vehicle.")
             return

        self.engine_on = not self.engine_on
        if self.engine_on:
            # State transition will be handled in physics_thread_func
            # Ensure engine state is STARTING if engine_on becomes True
            if self.engine_state == EngineState.OFF:
                 # The physics thread will detect engine_on and transition state
                 pass # State transition handled in physics_thread_func
            print("Engine command: START")
        else:
            # State transition to STOPPING will be handled in physics_thread_func
             if self.engine_state == EngineState.RUNNING or self.engine_state == EngineState.STARTING:
                  pass # State transition handled in physics_thread_func
             print("Engine command: STOP")

    def set_throttle(self, value):
        """Set throttle value (0.0 to 1.0)."""
        self.throttle = max(0.0, min(1.0, value))
        # print(f"Throttle set to {self.throttle*100:.0f}%") # Keep this print in main loop command handling

def main():
    # First, convert the sound files if they don't exist
    # Note: The conversion script needs to be in the same directory or callable.
    # Assumes sound_converter.py exists and has a convert_all_sounds function.
    sounds_dir_check = os.path.join(os.path.dirname(__file__), "sounds")
    if not os.path.exists(sounds_dir_check) or not os.listdir(sounds_dir_check):
        print("Sounds directory is empty or missing. Attempting to run sound converter...")
        try:
            import sound_converter
            # Assumes source sounds are in ../src/vehicles/sounds relative to the python script
            source_sounds_dir = os.path.join(os.path.dirname(__file__), "..", "src", "vehicles", "sounds")
            sound_converter.convert_all_sounds(source_sounds_dir, sounds_dir_check)
            print("Sound conversion attempted.")
        except ImportError:
            print("Error: sound_converter.py not found. Cannot convert sounds.")
        except Exception as e:
            print(f"Error during sound conversion: {e}")
        
    # Load vehicle configurations
    vehicles_dir = os.path.join(os.path.dirname(__file__), "..", "src", "vehicles")
    vehicles = load_all_vehicles(vehicles_dir)
    
    if not vehicles:
         print(f"Error: No vehicle configurations found in {vehicles_dir}.")
         print("Please ensure .h files are present and vehicle_parser.py can parse them.")
         return # Exit if no vehicles loaded

    # Create case-insensitive vehicle mapping and sorted list of names
    vehicle_map = {name.lower(): name for name in vehicles.keys()}
    sorted_vehicle_names = sorted(vehicles.keys())
    
    # Create and start audio engine
    audio = AudioEngine()
    if not audio.start(): # Check if audio engine started successfully (e.g., sounds dir check)
        print("Failed to start audio engine. Exiting.")
        return
    
    print("\nRC Engine Sound Simulator")
    print("Commands:")
    print("  l - List available vehicles")
    print("  s <name or no.> - Select vehicle")
    print("  e - Toggle engine")
    print("  h - Toggle horn")
    print("  i - Toggle siren")
    print("  t <0.0-1.0> - Set throttle (e.g. t 0.5 for 50%)")
    print("  q - Quit")
    
    try:
        while True:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == 'l':
                print("\nAvailable vehicles:")
                for i, name in enumerate(sorted_vehicle_names):
                    print(f"  {i}: {name}")
                    
            elif cmd.startswith('s '):
                select_param = cmd[2:].strip()
                selected_vehicle_name = None
                
                # Try to select by number first
                try:
                    vehicle_index = int(select_param)
                    if 0 <= vehicle_index < len(sorted_vehicle_names):
                        selected_vehicle_name = sorted_vehicle_names[vehicle_index]
                    else:
                        print(f"Invalid vehicle number: {select_param}. Use a number between 0 and {len(sorted_vehicle_names) - 1}.")
                        print("Available vehicles:")
                        for i, name in enumerate(sorted_vehicle_names):
                             print(f"  {i}: {name}")
                         
                except ValueError:
                    # If not a number, try to select by name (case-insensitive)
                    vehicle_name_lower = select_param.lower()
                    if vehicle_name_lower in vehicle_map:
                        selected_vehicle_name = vehicle_map[vehicle_name_lower]
                    else:
                        print(f"Vehicle '{select_param}' not found by name or number. Available vehicles:")
                        for i, name in enumerate(sorted_vehicle_names):
                             print(f"  {i}: {name}")

                if selected_vehicle_name:
                     # When selecting a new vehicle, load its sounds into the audio engine
                     audio.load_vehicle(vehicles[selected_vehicle_name])
                     print(f"Selected vehicle: {selected_vehicle_name}")
                     
            elif cmd == 'e':
                audio.toggle_engine()
                
            elif cmd == 'h':
                 # Play horn sound once
                 if audio.current_vehicle and 'horn' in audio.sound_buffers:
                      horn_sound = audio.sound_buffers['horn']
                      # Play on horn channel
                      audio.horn_channel.play(horn_sound)
                 else:
                      print("No horn sound available for current vehicle")

            elif cmd == 'i':
                 # Play siren sound once
                 if audio.current_vehicle and 'siren' in audio.sound_buffers:
                      siren_sound = audio.sound_buffers['siren']
                      # Play on siren channel
                      audio.siren_channel.play(siren_sound)
                 else:
                      print("No siren sound available for current vehicle")
                    
            elif cmd.startswith('t '):
                try:
                    throttle = float(cmd[2:].strip())
                    if 0.0 <= throttle <= 1.0:
                        audio.set_throttle(throttle)
                        # The physics thread will pick up this throttle change
                    else:
                        print("Throttle must be between 0.0 and 1.0")
                except ValueError:
                    print("Invalid throttle value. Use a number between 0.0 and 1.0")
                    
            elif cmd == 'q':
                break
            else:
                print("Unknown command. Available commands:")
                print("  l - List available vehicles")
                print("  s <name or no.> - Select vehicle")
                print("  e - Toggle engine")
                print("  h - Toggle horn")
                print("  i - Toggle siren")
                print("  t <0.0-1.0> - Set throttle")
                print("  q - Quit")
                
    except KeyboardInterrupt:
        pass # Exit gracefully on Ctrl+C
    finally:
        print("\nShutting down...")
        audio.stop() # Ensure stop is called on exit
        # pygame.quit() is called inside audio.stop()
        print("Goodbye!")

if __name__ == "__main__":
    main() 