import pygame
import numpy as np
import time
from enum import Enum
import threading
import os
from queue import Queue
from vehicle_parser import load_all_vehicles, VehicleConfig

def constrain(value, min_value, max_value):
    """Clamps a value to a specified range."""
    return max(min_value, min(value, max_value))

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
        self.throttle_input = 0.0 # 0.0-1.0, from user
        self._current_throttle_faded_500 = 0.0 # Faded throttle, 0-500 scale
        self.current_rpm = 0
        
        # These will be loaded from vehicle_config
        self.acc_cfg = 3 
        self.dec_cfg = 1
        self.time_base_ms_cfg = 2 # Default C++ timeBase
        self._min_rpm_cfg = 0
        self._max_rpm_cfg = 500
        
        self.master_volume = 100
        
        # Thread control
        self.running = True
        self.physics_thread = None
        self.debug_thread = None
        
        # Debug variables
        self._debug_faded_throttle_500 = 0.0
        self._debug_target_rpm = 0.0
        self._debug_idle_channel_vol = 0.0
        self._debug_rev_channel_vol = 0.0
        self._debug_a1_multi_perc = 0.0 # For rev sound blending

        # Debug volume calculation details initialization
        self._debug_engine_idle_vol_perc_cfg = 0.0
        self._debug_full_throttle_vol_perc_cfg = 0.0
        self._debug_throttle_map_output_perc = 0.0
        self._debug_engine_rev_vol_perc_cfg = 0.0
        self._debug_base_idle_scaler_cfg_perc = 0.0
        self._debug_base_rev_scaler_cfg_perc = 0.0
        self._debug_rev_switch_point_cfg = 0.0
        self._debug_idle_end_point_cfg = 0.0
        self._debug_idle_vol_proportion_perc_cfg = 0.0
        self._debug_throttle_map_output_for_rev_perc = 0.0

        # Engine mass simulation variables (from C++ engineMassSimulation)
        self._last_throttle_input_500 = 0.0 # On 0-500 scale
        self._wastegate_trigger = False
        self._blowoff_trigger = False
        self._wastegate_millis = 0
        self._blowoff_millis = 0
        
        self.dt = 0.01 # 100Hz physics update (10ms)
        self.debug_log_rate_hz = 20 # Default debug log rate (20 Hz)
        
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
        self.acc_cfg = getattr(vehicle_config, 'acc', 3) # Default from Beetle
        self.dec_cfg = getattr(vehicle_config, 'dec', 1) # Default from Beetle
        print(f"[DEBUG Load] acc_cfg: {self.acc_cfg}, dec_cfg: {self.dec_cfg}")
        # In C++, minRpm=0, maxRpm=500 are implicit in maps.
        self._min_rpm_cfg = getattr(vehicle_config, 'min_rpm', 0)
        self._max_rpm_cfg = getattr(vehicle_config, 'max_rpm', 500)
        
        # Stop any currently playing sounds on engine channels
        self.idle_channel.stop()
        self.rev_channel.stop()
        
        # Load engine sounds (idle and rev)
        idle_sound_name = getattr(vehicle_config, 'idle_sound', None)
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
             
        if getattr(vehicle_config, 'has_rev_sound', False):
            rev_sound_name = getattr(vehicle_config, 'rev_sound', None)
            if rev_sound_name:
                rev_sound_path = os.path.join("sounds", rev_sound_name)
                if os.path.exists(rev_sound_path):
                    try:
                        self.sound_buffers['rev'] = pygame.mixer.Sound(rev_sound_path)
                        print(f"Loaded rev sound: {rev_sound_name}")
                    except Exception as e:
                        print(f"Error loading rev sound {rev_sound_name}: {e}")
                else:
                    print(f"Error: Rev sound file not found: {rev_sound_path}")
            else:
                print("Warning: Rev sound enabled in config but no file specified.")

        # Load other sounds
        # Debugging start sound loading
        start_sound_name = getattr(vehicle_config, 'start_sound', None)
        print(f"[DEBUG Load] Start sound name from config: {start_sound_name}")
        if start_sound_name:
            start_sound_path = os.path.join("sounds", start_sound_name)
            print(f"[DEBUG Load] Looking for start sound at: {start_sound_path}")
            # Check if file exists before attempting to load
            if os.path.exists(start_sound_path):
                 try:
                      self.sound_buffers['start'] = pygame.mixer.Sound(start_sound_path)
                      print(f"Loaded start sound: {start_sound_name}")
                 except Exception as e:
                      print(f"Error loading start sound {start_sound_name}: {e}")
            else:
                 print(f"Error: Start sound file not found: {start_sound_path}")

        # Continue loading other sounds (horn, siren, etc.)
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
            # Calculate target RPM based on the non-faded throttle input
            target_throttle_500 = int(self.throttle_input * 500)
            
            # Replicate C++ time-based throttle interpolation from mapThrottle
            # C++ uses a fixed 500us interval and step of 16
            # In Python, we use dt and need to adjust the step size proportionally
            # We should update this based on a fixed time accumulator, not just dt directly.
            # Let's add an accumulator for the faded throttle update timer
            # If we were directly porting C++: self._current_throttle_faded_500 += 16 if accelerating, -= 16 if decelerating
            # But this happens every 500us (0.5ms). Our physics update is 10ms.
            # So in a 10ms interval, the C++ updates faded throttle 20 times (10ms / 0.5ms = 20)
            # The total change in 10ms in C++ is step * (dt / interval) = 16 * (0.010 / 0.0005) = 16 * 20 = 320
            # However, the C++ logic is simpler: it just adds/subtracts 16 every 0.5ms loop iteration
            # Let's try to replicate the *amount* of change that would happen in self.dt
            
            # Determine the fade step based on C++ logic's fixed step (16) and interval (0.5ms)
            cpp_fade_step = 16 # Fixed step in C++
            cpp_fade_interval_ms = 0.5 # Fixed interval in C++ (500us)
            
            # Calculate how many C++ intervals fit into our dt
            num_intervals = (self.dt * 1000.0) / cpp_fade_interval_ms
            
            # Calculate the total change to apply in this dt
            fade_amount = cpp_fade_step * num_intervals

            # Apply the fade amount towards the target throttle
            if self._current_throttle_faded_500 < target_throttle_500:
                self._current_throttle_faded_500 += fade_amount
                # Ensure we don't overshoot the target
                if self._current_throttle_faded_500 > target_throttle_500:
                     self._current_throttle_faded_500 = target_throttle_500
            elif self._current_throttle_faded_500 > target_throttle_500:
                self._current_throttle_faded_500 -= fade_amount
                # Ensure we don't overshoot the target
                if self._current_throttle_faded_500 < target_throttle_500:
                     self._current_throttle_faded_500 = target_throttle_500

            # Ensure faded throttle stays within 0-500 range
            self._current_throttle_faded_500 = constrain(self._current_throttle_faded_500, 0.0, 500.0)

            # Ensure minimum target RPM is idle RPM if above 0 throttle
            idle_rpm_target = getattr(self.current_vehicle, 'idle_rpm_target')
            # Match C++ dead zone behavior from engineMassSimulation
            target_rpm = target_throttle_500
            if self._current_throttle_faded_500 > 0:
                target_rpm = max(idle_rpm_target, target_throttle_500)
            else:
                target_rpm = idle_rpm_target

            # Engine RPM Acceleration/Deceleration using vehicle config values directly
            rpm_diff = target_rpm - self.current_rpm

            if rpm_diff > 0:
                # Accelerate engine
                self.current_rpm = min(self.current_rpm + self.acc_cfg, target_rpm)
            elif rpm_diff < 0:
                # Decelerate engine
                self.current_rpm = max(self.current_rpm - self.dec_cfg, target_rpm)

            # Ensure minimum idle RPM when engine is running
            # This check is important if dec makes RPM drop below idle_rpm_target even if target is > idle_rpm_target
            # Match C++ minimum RPM check using faded throttle
            if self.current_rpm < idle_rpm_target and self.throttle_input > 0:
                self.current_rpm = idle_rpm_target
            # If throttle is 0 and RPM drops, let it go to the true idle target

            # Debug print: Show target RPM and updated current RPM
            # print(f"[Physics] Target RPM: {target_rpm}, Updated Current RPM: {self.current_rpm}")

            self._debug_target_rpm = target_rpm

        elif self.engine_state == EngineState.STOPPING:
             # Decelerate to 0 RPM when stopping
            self.current_rpm = max(self.current_rpm - self.dec_cfg, 0)
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

        self._last_throttle_input_500 = target_throttle_500

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
        throttle_500 = int(self.throttle_input * 500) # Scale throttle 0-1 to 0-500 range
        throttle_500 = max(0, min(500, throttle_500)) # Ensure bounds

        # Recalculate target_throttle_500 for use in this function
        target_throttle_500 = int(self.throttle_input * 500)

        # Get volume percentages from vehicle config
        engine_idle_vol_perc = getattr(self.current_vehicle, 'engine_idle_volume', 100)
        full_throttle_vol_perc = getattr(self.current_vehicle, 'full_throttle_volume', 100)
        engine_rev_vol_perc = getattr(self.current_vehicle, 'engine_rev_volume', 100)
        idle_base_vol_perc = getattr(self.current_vehicle, 'idle_volume', 100)
        rev_base_vol_perc = getattr(self.current_vehicle, 'rev_volume', 100)

        # Interpolate throttle-dependent engine volumes (0-100 scale)
        # ESP32 maps throttle 0-500 to engineIdleVolumePercentage-fullThrottleVolumePercentage
        calc_engine_idle_vol = np.interp(throttle_500, [0, 500], [engine_idle_vol_perc, full_throttle_vol_perc])
        calc_engine_idle_vol = constrain(calc_engine_idle_vol, 0, 100)
        calc_engine_rev_vol = np.interp(throttle_500, [0, 500], [engine_rev_vol_perc, full_throttle_vol_perc])
        calc_engine_rev_vol = constrain(calc_engine_rev_vol, 0, 100)

        # Determine the blend ratio between idle and rev sounds based on RPM
        # ESP32 uses revSwitchPoint and idleEndPoint (get from vehicle config, use defaults if not present)
        # Load blending configuration from vehicle config
        rev_switch_point_cfg = getattr(self.current_vehicle, 'rev_switch_point', 280)
        idle_end_point_cfg = getattr(self.current_vehicle, 'idle_end_point', 300)
        idle_vol_proportion_perc_cfg = getattr(self.current_vehicle, 'idle_volume_proportion_percentage', 90)

        # Load base volume percentages from vehicle config
        idle_base_vol_perc = getattr(self.current_vehicle, 'idle_volume_percentage', 100)
        rev_base_vol_perc = getattr(self.current_vehicle, 'rev_volume_percentage', 100)

        # Calculate a1Multi based on C++ logic for blending idle and rev sounds
        # a1Multi determines the proportion of idle sound (0-100)
        a1_multi_perc = 0.0
        # Blending will now be based on FADED THROTTLE (0-500 scale), not RPM
        blending_input_500 = self._current_throttle_faded_500

        # Replicating C++ a1Multi blending logic precisely (src/src.ino lines ~555-560)
        # Using config points (originally for RPM) as throttle points for blending

        if blending_input_500 > rev_switch_point_cfg:
            # Map blending_input_500 from [rev_switch_point_cfg, idle_end_point_cfg] to [idle_vol_proportion_perc_cfg, 0.0]
            # This replicates the C++ map behavior where RPM > revSwitchPoint
            a1_multi_perc = np.interp(
                blending_input_500,
                [rev_switch_point_cfg, idle_end_point_cfg],
                [idle_vol_proportion_perc_cfg, 0.0] # Output goes from idle_vol_proportion_perc_cfg down to 0
            )
        else:
            # If input is at or below revSwitchPoint, a1Multi is idleVolumeProportionPercentage
            a1_multi_perc = idle_vol_proportion_perc_cfg

        # Apply the overriding check: if input > idleEndPoint, a1Multi = 0
        if blending_input_500 > idle_end_point_cfg:
             a1_multi_perc = 0.0

        # Ensure a1_multi_perc is within bounds (0-100)
        a1_multi_perc = constrain(a1_multi_perc, 0.0, 100.0)

        # Debug print for a1Multi - will be logged by debug thread
        self._debug_a1_multi_perc = a1_multi_perc

        # --- Calculate Final Channel Volumes ---
        # Each channel's volume is determined by its calculated throttle-dependent volume,
        # the blending factor, its base volume percentage from config, and the master volume.

        final_idle_volume_pygame_scale = (
            (calc_engine_idle_vol / 100.0) * # Throttle-dependent idle vol (0-1 scale)
            (a1_multi_perc / 100.0) * # a1Multi blending factor for idle
            (idle_base_vol_perc / 100.0) * # Base idle volume percentage from config (corrected variable name)
            (self.master_volume / 100.0) # Master volume
        )

        final_rev_volume_pygame_scale = (
            (calc_engine_rev_vol / 100.0) * # Throttle-dependent rev vol (0-1 scale)
            ((100.0 - a1_multi_perc) / 100.0) * # a1Multi blending factor for rev
            (rev_base_vol_perc / 100.0) * # Base rev volume percentage from config (corrected variable name)
            (self.master_volume / 100.0) # Master volume
        )

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

        # --- Update Pitch ---
        # Apply pitch scaling to both channels based on FADED THROTTLE (0-500 scale)
        pitch_input_500 = self._current_throttle_faded_500

        # Ensure pitch range is valid before mapping
        if self.max_pitch > self.min_pitch:
             # Map faded throttle (0-500) to pitch range (min_pitch to max_pitch)
             pitch_factor = np.interp(
                 pitch_input_500, [0.0, 500.0],
                 [self.min_pitch, self.max_pitch]
             )
             # Constrain pitch to the defined range
             pitch_factor = constrain(pitch_factor, self.min_pitch, self.max_pitch)
             
             # Apply pitch to channels
             self.idle_channel.set_pitch(pitch_factor)
             self.rev_channel.set_pitch(pitch_factor)
        else:
             # Default pitch if range is invalid or min/max are the same
             self.idle_channel.set_pitch(1.0)
             self.rev_channel.set_pitch(1.0)

        # Debug printing (optional)
        # print(f"[AudioUpdate] RPM: {self.current_rpm}, Thr: {self.throttle:.2f}, IdleBlend: {idle_blend:.2f}, RevBlend: {rev_blend:.2f}, IdleVol: {final_idle_volume_pygame_scale:.2f}, RevVol: {final_rev_volume_pygame_scale:.2f}")

        vc = self.current_vehicle
        fade_rate = self.acc_cfg if self._current_throttle_faded_500 < target_throttle_500 else self.dec_cfg
        current_rpm_500 = self.current_rpm

        # --- Volume calculation based on C++ mapThrottle and ISR logic ---
        
        # 1. Calculate throttle_map_output_perc (like C++ throttleDependentVolume)
        # These are from vehicle config (e.g., Beetle: eng_idle=120, full_thr=200)
        engine_idle_vol_perc_cfg = getattr(vc, 'engine_idle_volume_percentage', 100)
        full_throttle_vol_perc_cfg = getattr(vc, 'full_throttle_volume_percentage', 100)
        
        throttle_map_output_perc = np.interp(
            self._current_throttle_faded_500, [0, 500],
            [engine_idle_vol_perc_cfg, full_throttle_vol_perc_cfg]
        )
        print(f"[DEBUG VolCalc] FadThr500: {self._current_throttle_faded_500:.1f}, EngIdleVol%: {engine_idle_vol_perc_cfg}, FullThrVol%: {full_throttle_vol_perc_cfg}, ThrottleMapOut%: {throttle_map_output_perc:.1f}")

        # Get base volume scalers (C++ globals, e.g., Beetle: idle_vol_perc=120)
        base_idle_scaler_cfg_perc = getattr(vc, 'idle_volume_percentage', 100)

        # Store debug values
        self._debug_idle_channel_vol = self.idle_channel.get_volume()
        self._debug_rev_channel_vol = self.rev_channel.get_volume()

        # Debug volume calculation details
        self._debug_faded_throttle_500 = self._current_throttle_faded_500
        self._debug_engine_idle_vol_perc_cfg = engine_idle_vol_perc_cfg
        self._debug_full_throttle_vol_perc_cfg = full_throttle_vol_perc_cfg
        self._debug_throttle_map_output_perc = throttle_map_output_perc
        if getattr(vc, 'has_rev_sound', False):
            self._debug_engine_rev_vol_perc_cfg = engine_rev_vol_perc
            self._debug_base_idle_scaler_cfg_perc = base_idle_scaler_cfg_perc
            self._debug_base_rev_scaler_cfg_perc = base_rev_scaler_cfg_perc
            self._debug_rev_switch_point_cfg = rev_switch_point_cfg
            self._debug_idle_end_point_cfg = idle_end_point_cfg
            self._debug_idle_vol_proportion_perc_cfg = idle_vol_proportion_perc_cfg
            self._debug_throttle_map_output_for_rev_perc = throttle_map_output_for_rev_perc
        else:
            self._debug_engine_rev_vol_perc_cfg = 0 # N/A
            self._debug_base_idle_scaler_cfg_perc = base_idle_scaler_cfg_perc # Still relevant for non-rev sound calc
            self._debug_base_rev_scaler_cfg_perc = 0 # N/A
            self._debug_rev_switch_point_cfg = 0 # N/A
            self._debug_idle_end_point_cfg = 0 # N/A
            self._debug_idle_vol_proportion_perc_cfg = 0 # N/A
            self._debug_throttle_map_output_for_rev_perc = 0 # N/A

        # Ensure debug a1_multi_perc is set even if has_rev_sound is False (will be 100)
        if not getattr(vc, 'has_rev_sound', False):
             self._debug_a1_multi_perc = 100.0 # Hardcoded when no rev sound
        # Debug blending config values regardless of has_rev_sound
        self._debug_rev_switch = rev_switch_point_cfg
        self._debug_idle_end = idle_end_point_cfg
        self._debug_idle_blend = a1_multi_perc # Store the calculated a1Multi
        self._debug_rev_blend = (100.0 - a1_multi_perc) # Store the calculated rev blend
        self._debug_engine_idle_vol_perc_cfg = engine_idle_vol_perc_cfg
        self._debug_full_throttle_vol_perc_cfg = full_throttle_vol_perc_cfg
        self._debug_throttle_map_output_perc = throttle_map_output_perc
        self._debug_base_idle_scaler_cfg_perc = idle_base_vol_perc # Use the loaded base volume
        self._debug_idle_vol_proportion_perc_cfg = idle_vol_proportion_perc_cfg
        
        if getattr(vc, 'has_rev_sound', False):
            self._debug_engine_rev_vol_perc_cfg = engine_rev_vol_perc
            self._debug_base_rev_scaler_cfg_perc = rev_base_vol_perc
            # Need to calculate throttle_map_output_for_rev_perc for debug if rev sound is present
            throttle_map_output_for_rev_perc = np.interp(
                self._current_throttle_faded_500, [0, 500],
                [engine_rev_vol_perc, full_throttle_vol_perc]
            )
            self._debug_throttle_map_output_for_rev_perc = throttle_map_output_for_rev_perc
        else:
            # Set rev-sound specific debug values to 0 or N/A if no rev sound
            self._debug_engine_rev_vol_perc_cfg = 0
            self._debug_base_rev_scaler_cfg_perc = 0
            self._debug_throttle_map_output_for_rev_perc = 0

    def debug_printer_thread_func(self):
        """Prints debug information at a controlled rate (2Hz)."""
        log_file_path = "log.txt"
        try:
            with open(log_file_path, 'w') as log_file:
                log_file.write("RC Engine Sound Simulator Debug Log\n")
                log_file.write("----------------------------------\n")
                # Write header
                header = ("[DBG] Time(ms), RPM, ThrIn, FadThr500, TgtRPM, "
                          "EngIdleVol%, FullThrVol%, ThrottleMapOut%, "
                          "BaseIdleScaler%, BaseRevScaler%, "
                          "RevSwitch, IdleEnd, IdleVolProp%, "
                          "ThrMapOutRev%, a1Multi%, IdleVol, RevVol\n")
                log_file.write(header)
                log_file.flush()

                while self.running:
                    # Only log when engine is running or stopping and vehicle is selected
                    if self.current_vehicle and (self.engine_state == EngineState.RUNNING or self.engine_state == EngineState.STOPPING):
                        current_time_ms = int(time.time() * 1000)
                        log_entry = (
                            f"[DBG] {current_time_ms}, "
                            f"{self.current_rpm:.0f}, {self.throttle_input:.2f}, {self._debug_faded_throttle_500:.1f}, {self._debug_target_rpm:.0f}, "
                            f"{self._debug_engine_idle_vol_perc_cfg:.1f}, {self._debug_full_throttle_vol_perc_cfg:.1f}, {self._debug_throttle_map_output_perc:.1f}, "
                            f"{self._debug_base_idle_scaler_cfg_perc:.1f}, {self._debug_base_rev_scaler_cfg_perc:.1f}, "
                            f"{self._debug_rev_switch:.0f}, {self._debug_idle_end:.0f}, {self._debug_idle_blend:.1f}, {self._debug_rev_blend:.1f}, {self._debug_a1_multi_perc:.1f}, {self._debug_idle_channel_vol:.3f}, {self._debug_rev_channel_vol:.3f}\n"
                        )
                        log_file.write(log_entry)
                        log_file.flush()

                    # Control log rate
                    time.sleep(1.0 / self.debug_log_rate_hz)

                log_file.write("[DEBUG] Debug printer thread stopping.\n")
        except Exception as e:
            print(f"Error writing to log.txt: {e}")

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
        self.throttle_input = max(0.0, min(1.0, value))
        # print(f"Throttle set to {self.throttle_input*100:.0f}%") # Keep this print in main loop command handling

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