import re
import os

class VehicleConfig:
    def __init__(self, name):
        self.name = name
        # Sound files
        self.start_sound = None
        self.idle_sound = None
        self.rev_sound = None
        self.knock_sound = None
        self.horn_sound = None
        self.siren_sound = None
        # Volume settings
        self.start_volume = 100
        self.idle_volume = 100
        self.engine_idle_volume = 100
        self.full_throttle_volume = 100
        self.rev_volume = 100
        self.engine_rev_volume = 100
        self.knock_volume = 100
        self.horn_volume = 100
        self.siren_volume = 100
        self.turbo_volume = 0
        self.turbo_idle_volume = 0
        self.charger_volume = 0
        self.charger_idle_volume = 0
        self.charger_start_point = 0
        self.wastegate_volume = 0
        self.wastegate_idle_volume = 0
        self.fan_volume = 0
        self.fan_idle_volume = 0
        self.fan_start_point = 0
        self.brake_volume = 100
        self.parking_brake_volume = 100
        self.shifting_volume = 100
        self.sound1_volume = 100
        self.reversing_volume = 0
        self.indicator_volume = 100
        self.coupling_volume = 100
        # Clutch/Gearbox/Engine
        self.clutch_engaging_point = 0
        self.automatic = False
        self.double_clutch = False
        self.shifting_auto_throttle = False
        self.number_of_automatic_gears = 3
        self.acc = 2
        self.dec = 1
        self.max_rpm_percentage = 500
        # Other
        self.idle_rpm_target = 800  # Default value from EngineParameters
        self.indicator_on = 300
        self.indicator_dir = True
        self.double_flash_blue_light = False
        self.esc_ramp_time_first_gear = 20
        self.esc_ramp_time_second_gear = 50
        self.esc_ramp_time_third_gear = 75
        self.esc_brake_steps = 30
        self.esc_acceleration_steps = 3
        self.flags = set()

# Helper to parse #define and #undef flags
flag_pattern = re.compile(r'#define\s+(\w+)')
ifdef_pattern = re.compile(r'#ifdef\s+(\w+)')
ifndef_pattern = re.compile(r'#ifndef\s+(\w+)')
ifdefined_pattern = re.compile(r'#if\s+defined\s+(\w+)')

# Helper to find all enabled flags

def extract_flags(content):
    flags = set()
    for match in flag_pattern.finditer(content):
        flags.add(match.group(1))
    for match in ifdef_pattern.finditer(content):
        flags.add(match.group(1))
    for match in ifdefined_pattern.finditer(content):
        flags.add(match.group(1))
    # Note: #ifndef disables, but for now just collect enables
    return flags

def parse_vehicle_file(file_path):
    """Parse a vehicle configuration file and extract sound settings and all relevant parameters."""
    with open(file_path, 'r') as f:
        content = f.read()
    
    config = VehicleConfig(os.path.basename(file_path)[:-2])  # Remove .h extension
    
    # Extract all enabled feature flags
    config.flags = extract_flags(content)
    
    # Extract sound file includes (in order)
    sound_pattern = r'#include\s+"sounds/([^"]+)"'
    sound_includes = re.findall(sound_pattern, content)
    
    # Extract volume and parameter settings
    param_patterns = [
        (r'volatile\s+int\s+(\w+)VolumePercentage\s*=\s*(\d+)', '_volume'),
        (r'volatile\s+int\s+turboIdleVolumePercentage\s*=\s*(\d+)', 'turbo_idle_volume'),
        (r'volatile\s+int\s+chargerIdleVolumePercentage\s*=\s*(\d+)', 'charger_idle_volume'),
        (r'volatile\s+int\s+chargerStartPoint\s*=\s*(\d+)', 'charger_start_point'),
        (r'volatile\s+int\s+wastegateIdleVolumePercentage\s*=\s*(\d+)', 'wastegate_idle_volume'),
        (r'volatile\s+int\s+fanIdleVolumePercentage\s*=\s*(\d+)', 'fan_idle_volume'),
        (r'volatile\s+int\s+fanStartPoint\s*=\s*(\d+)', 'fan_start_point'),
        (r'volatile\s+int\s+reversingVolumePercentage\s*=\s*(\d+)', 'reversing_volume'),
        (r'volatile\s+int\s+indicatorVolumePercentage\s*=\s*(\d+)', 'indicator_volume'),
        (r'volatile\s+int\s+couplingVolumePercentage\s*=\s*(\d+)', 'coupling_volume'),
        (r'uint16_t\s+clutchEngagingPoint\s*=\s*(\d+)', 'clutch_engaging_point'),
        (r'const\s+boolean\s+automatic\s*=\s*(true|false)', 'automatic'),
        (r'const\s+boolean\s+doubleClutch\s*=\s*(true|false)', 'double_clutch'),
        (r'const\s+boolean\s+shiftingAutoThrottle\s*=\s*(true|false)', 'shifting_auto_throttle'),
        (r'#define\s+NumberOfAutomaticGears\s+(\d+)', 'number_of_automatic_gears'),
        (r'const\s+int8_t\s+acc\s*=\s*(-?\d+)', 'acc'),
        (r'const\s+int8_t\s+dec\s*=\s*(-?\d+)', 'dec'),
        (r'uint32_t\s+MAX_RPM_PERCENTAGE\s*=\s*(\d+)', 'max_rpm_percentage'),
        (r'const\s+uint16_t\s+indicatorOn\s*=\s*(\d+)', 'indicator_on'),
        (r'const\s+boolean\s+INDICATOR_DIR\s*=\s*(true|false)', 'indicator_dir'),
        (r'const\s+boolean\s+doubleFlashBlueLight\s*=\s*(true|false)', 'double_flash_blue_light'),
        (r'const\s+uint8_t\s+escRampTimeFirstGear\s*=\s*(\d+)', 'esc_ramp_time_first_gear'),
        (r'const\s+uint8_t\s+escRampTimeSecondGear\s*=\s*(\d+)', 'esc_ramp_time_second_gear'),
        (r'const\s+uint8_t\s+escRampTimeThirdGear\s*=\s*(\d+)', 'esc_ramp_time_third_gear'),
        (r'const\s+uint8_t\s+escBrakeSteps\s*=\s*(\d+)', 'esc_brake_steps'),
        (r'const\s+uint8_t\s+escAccelerationSteps\s*=\s*(\d+)', 'esc_acceleration_steps'),
        # Add patterns for revSwitchPoint and idleEndPoint
        (r'volatile\s+const\s+uint16_t\s+revSwitchPoint\s*=\s*(\d+)', 'revSwitchPoint'),
        (r'volatile\s+const\s+uint16_t\s+idleEndPoint\s*=\s*(\d+)', 'idleEndPoint'),
    ]
    for pattern, attr in param_patterns:
        for match in re.findall(pattern, content):
            value = match if isinstance(match, str) else match[-1]
            if attr.endswith('_volume') or attr in ['acc', 'dec', 'max_rpm_percentage', 'clutch_engaging_point', 'number_of_automatic_gears', 'indicator_on', 'esc_ramp_time_first_gear', 'esc_ramp_time_second_gear', 'esc_ramp_time_third_gear', 'esc_brake_steps', 'esc_acceleration_steps', 'fan_start_point', 'charger_start_point', 'wastegate_idle_volume', 'fan_idle_volume', 'reversing_volume', 'indicator_volume', 'coupling_volume', 'revSwitchPoint', 'idleEndPoint']:
                setattr(config, attr, int(value))
            elif attr in ['automatic', 'double_clutch', 'shifting_auto_throttle', 'indicator_dir', 'double_flash_blue_light']:
                setattr(config, attr, value == 'true')
    # Assign sound files based on section and enabled flags
    # We'll use a state machine approach to track which section we're in
    section = None
    lines = content.splitlines()
    for i, line in enumerate(lines):
        l = line.strip()
        # Section detection
        if 'Choose the start sound' in l:
            section = 'start'
        elif 'Choose the motor idle sound' in l:
            section = 'idle'
        elif 'Choose the motor revving sound' in l:
            section = 'rev'
        elif 'Choose the jake brake sound' in l:
            section = 'jake_brake'
        elif 'Choose the Diesel' in l and 'knock' in l:
            section = 'knock'
        elif 'Choose the horn sound' in l:
            section = 'horn'
        elif 'Choose the siren' in l:
            section = 'siren'
        # Add more as needed
        # Only assign if #include in this line
        if l.startswith('#include') and 'sounds/' in l:
            sound_file = re.search(r'sounds/([^"]+)', l)
            if sound_file:
                sound_file = sound_file.group(1).replace('.h', '.wav')
                # Only assign if feature is enabled (for rev, jake_brake, etc.)
                if section == 'start' and config.start_sound is None:
                    config.start_sound = sound_file
                elif section == 'idle' and config.idle_sound is None:
                    config.idle_sound = sound_file
                elif section == 'rev' and 'REV_SOUND' in config.flags and config.rev_sound is None:
                    config.rev_sound = sound_file
                elif section == 'knock' and config.knock_sound is None:
                    config.knock_sound = sound_file
                elif section == 'horn' and config.horn_sound is None:
                    config.horn_sound = sound_file
                elif section == 'siren' and config.siren_sound is None:
                    config.siren_sound = sound_file
                # Add more as needed
    return config

def load_all_vehicles(vehicles_dir):
    """Load all vehicle configurations from the vehicles directory."""
    vehicles = {}
    
    for filename in os.listdir(vehicles_dir):
        if filename.endswith('.h') and filename != '00_Master.h':
            file_path = os.path.join(vehicles_dir, filename)
            try:
                config = parse_vehicle_file(file_path)
                vehicles[config.name] = config
            except Exception as e:
                print(f"Error loading {filename}: {str(e)}")
    
    return vehicles

if __name__ == "__main__":
    # Test loading vehicles
    vehicles_dir = "../src/vehicles"
    vehicles = load_all_vehicles(vehicles_dir)
    
    print("Loaded vehicles:")
    for name, config in vehicles.items():
        print(f"\n{name}:")
        print(f"  Start: {config.start_sound} ({config.start_volume}%)")
        print(f"  Idle: {config.idle_sound} ({config.idle_volume}%)")
        print(f"  Rev: {config.rev_sound} ({config.rev_volume}%)")
        print(f"  Knock: {config.knock_sound} ({config.knock_volume}%)")
        print(f"  Horn: {config.horn_sound} ({config.horn_volume}%)")
        print(f"  Siren: {config.siren_sound} ({config.siren_volume}%)")
        print(f"  Flags: {sorted(list(config.flags))}")