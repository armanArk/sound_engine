import os
import numpy as np
from scipy.io import wavfile

def convert_sound_file(input_file, output_file):
    """Convert a .h sound file to .wav format."""
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Extract the sound data array
    start_idx = content.find('{')
    end_idx = content.rfind('}')
    if start_idx == -1 or end_idx == -1:
        print(f"Could not find sound data in {input_file}")
        return False
    
    data_str = content[start_idx + 1:end_idx]
    # Convert string array to numpy array
    try:
        data = np.array([int(x.strip()) for x in data_str.split(',') if x.strip()], dtype=np.int16)
        # Normalize to 16-bit range
        data = np.clip(data, -32768, 32767)
        # Save as WAV file
        wavfile.write(output_file, 22050, data)  # Using 22050 Hz sample rate
        return True
    except Exception as e:
        print(f"Error converting {input_file}: {str(e)}")
        return False

def convert_all_sounds(input_dir, output_dir):
    """Convert all .h sound files in input_dir to .wav files in output_dir."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    converted = 0
    failed = 0
    
    for filename in os.listdir(input_dir):
        if filename.endswith('.h'):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, filename[:-2] + '.wav')
            
            if convert_sound_file(input_path, output_path):
                converted += 1
                print(f"Converted: {filename}")
            else:
                failed += 1
                print(f"Failed to convert: {filename}")
    
    print(f"\nConversion complete: {converted} files converted, {failed} files failed")

if __name__ == "__main__":
    input_dir = "../src/vehicles/sounds"
    output_dir = "sounds"
    convert_all_sounds(input_dir, output_dir) 