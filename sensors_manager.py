import time
import threading
import random
import math

# Try to import Raspberry Pi specific libraries
try:
    import board
    import busio
    import adafruit_mpu6050
    SENSORS_AVAILABLE = True
except ImportError:
    SENSORS_AVAILABLE = False
    print("Sensor libraries not available - using simulated data")

class SensorsManager:
    def __init__(self):
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 100.0
        self.temperature = 22.0
        self.pressure = 1013.0
        self.speed = 0.0
        
        # Variables for MPU6050 heading calculation
        self.mag_declination = 0.0  # Magnetic declination for your location
        self.last_update_time = time.time()
        self.gyro_heading = 0.0
        self.accel_heading = 0.0
        self.complementary_heading = 0.0
        self.gyro_weight = 0.98  # Weight for complementary filter
        
        self._running = False
        self._sensors_thread = None
        self._lock = threading.Lock()
        self._simulation_mode = not SENSORS_AVAILABLE
        
        # Initialize sensors or simulation
        self._init_sensors()
        
    def _init_sensors(self):
        """Initialize hardware sensors or simulation mode"""
        if not self._simulation_mode:
            try:
                # Set up I2C interface
                self.i2c = busio.I2C(board.SCL, board.SDA)
                
                # Initialize IMU for gyro/accelerometer (MPU6050)
                try:
                    self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
                    
                    # Configure MPU6050 settings
                    self.mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_8_G
                    self.mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_500_DPS
                    self.mpu.filter_bandwidth = adafruit_mpu6050.Bandwidth.BAND_21_HZ
                    print("MPU6050 sensor initialized")
                    
                    # Calibrate the sensor
                    self._calibrate_mpu6050()
                except Exception as e:
                    print(f"Failed to initialize MPU6050: {e}")
                    self.mpu = None
                    self._simulation_mode = True
                
            except Exception as e:
                print(f"Error initializing sensors: {e}")
                self._simulation_mode = True
        
        if self._simulation_mode:
            print("Running in simulation mode - no MPU6050 detected")
            
    def _calibrate_mpu6050(self):
        """Calibrate the MPU6050 sensor to find baseline offsets"""
        print("Calibrating MPU6050. Keep the device still...")
        
        # Initialize offset variables
        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0
        
        # Number of samples to take for calibration
        samples = 100
        
        # Collect multiple readings and average them
        for _ in range(samples):
            gx, gy, gz = self.mpu.gyro
            self.gyro_offset_x += gx
            self.gyro_offset_y += gy
            self.gyro_offset_z += gz
            time.sleep(0.01)
            
        # Calculate average offset
        self.gyro_offset_x /= samples
        self.gyro_offset_y /= samples
        self.gyro_offset_z /= samples
        
        print(f"Gyro offsets: X={self.gyro_offset_x:.4f}, Y={self.gyro_offset_y:.4f}, Z={self.gyro_offset_z:.4f}")
        print("Calibration complete!")
    
    def start(self):
        """Start the sensor reading thread"""
        self._running = True
        self._sensors_thread = threading.Thread(target=self._sensors_loop)
        self._sensors_thread.daemon = True
        self._sensors_thread.start()
        print("Sensor manager started")
    
    def stop(self):
        """Stop the sensor reading thread"""
        self._running = False
        if self._sensors_thread:
            self._sensors_thread.join(timeout=1.0)
        print("Sensor manager stopped")
    
    def _sensors_loop(self):
        """Main loop for reading sensors or generating simulated data"""
        last_update = 0
        update_interval = 0.01  # 100 Hz update rate for smooth heading
        
        while self._running:
            current_time = time.time()
            
            # Limit update rate
            if current_time - last_update >= update_interval:
                last_update = current_time
                
                if not self._simulation_mode and hasattr(self, 'mpu') and self.mpu:
                    self._read_mpu6050()
                else:
                    self._simulate_sensors()
                    
            # Don't hog the CPU
            time.sleep(0.001)  # 1ms sleep
            
    def _read_mpu6050(self):
        """Read data from MPU6050 sensor and calculate heading"""
        try:
            # Get current time for delta-time calculation
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # Read accelerometer and gyroscope values
            accel_x, accel_y, accel_z = self.mpu.acceleration
            gyro_x, gyro_y, gyro_z = self.mpu.gyro
            
            # Apply calibration offsets
            gyro_x -= self.gyro_offset_x
            gyro_y -= self.gyro_offset_y
            gyro_z -= self.gyro_offset_z
            
            # Compute pitch and roll from accelerometer (in degrees)
            accel_roll = math.atan2(accel_y, accel_z) * 180.0 / math.pi
            accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180.0 / math.pi
            
            # Calculate heading from gyroscope
            # Note: We're using gyro_z for heading/yaw changes
            self.gyro_heading += gyro_z * dt
            
            # Keep heading in 0-360 range
            self.gyro_heading %= 360
            
            # Calculate heading from accelerometer tilt compensated
            # This is a simplified approach using accelerometer
            # For a more accurate heading, you would need a magnetometer (compass)
            if accel_z != 0:
                accel_heading = math.degrees(math.atan2(accel_y, accel_x)) + 180
            else:
                accel_heading = 0
                
            # Apply complementary filter to combine both headings
            # This reduces drift from the gyro and noise from the accelerometer
            self.complementary_heading = (self.gyro_weight * (self.complementary_heading + gyro_z * dt) + 
                                         (1.0 - self.gyro_weight) * accel_heading)
            
            # Keep complementary heading in 0-360 range
            self.complementary_heading %= 360
            
            # Add magnetic declination to get true heading
            true_heading = (self.complementary_heading + self.mag_declination) % 360
            
            # Update sensor values with lock to avoid race conditions
            with self._lock:
                self.heading = true_heading
                self.roll = accel_roll
                self.pitch = accel_pitch
                self.yaw = true_heading  # Yaw is essentially heading
                self.temperature = self.mpu.temperature
                
                # Calculate "speed" as movement intensity (total acceleration magnitude)
                accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2) - 9.8  # Subtract gravity
                self.speed = max(0, abs(accel_magnitude) * 10)  # Scale for display
                
        except Exception as e:
            print(f"Error reading MPU6050: {e}")
            self._simulate_sensors()  # Fall back to simulation if reading fails
    
    def _simulate_sensors(self):
        """Generate simulated sensor data"""
        with self._lock:
            # Simulate slowly changing heading (compass)
            self.heading = (self.heading + 0.5) % 360
            
            # Simulate slight pitch and roll oscillations
            t = time.time() * 0.5
            self.pitch = 5 * math.sin(t)
            self.roll = 7 * math.cos(t * 0.7)
            self.yaw = self.heading
            
            # Simulate altitude variations
            self.altitude = 100 + 10 * math.sin(t * 0.2)
            
            # Simulate speed changes
            self.speed = 85 + 5 * math.sin(t * 0.3)
    
    def get_sensor_data(self):
        """Get current sensor readings as a dictionary"""
        with self._lock:
            return {
                'heading': self.heading,
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
                'altitude': self.altitude,
                'temperature': self.temperature,
                'pressure': self.pressure,
                'speed': self.speed
            }
    
    def set_declination(self, declination):
        """Set the magnetic declination for your location"""
        self.mag_declination = declination
        
    def reset_heading(self):
        """Reset the heading to zero (recalibrate)"""
        with self._lock:
            self.gyro_heading = 0
            self.complementary_heading = 0
            self.heading = 0
            self.yaw = 0

# Import this to other files and integrate with HUD
if __name__ == "__main__":
    sensors = SensorsManager()
    sensors.start()
    
    try:
        while True:
            data = sensors.get_sensor_data()
            print(f"Heading: {data['heading']:.1f}°, "
                  f"Pitch: {data['pitch']:.1f}°, "
                  f"Roll: {data['roll']:.1f}°, "
                  f"Alt: {data['altitude']:.1f}m, "
                  f"Temp: {data['temperature']:.1f}°C")
            time.sleep(0.5)
    except KeyboardInterrupt:
        sensors.stop()
        print("Test completed")