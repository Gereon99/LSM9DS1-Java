### LSM9DS1 for Java

This repository provides you with an easy to use driver for the LSM9DS1 Inertial Measurement Unit (IMU) written in Java,
focused to be used on a Raspberry Pi.
The sensor includes an accelerometer, gyroscope, magnetometer and a temperature sensor.

The driver allows you to read data from the IMU, but also supplies more complex methods for calculating the
Euler angles (pitch, roll and yaw) while using low-pass and complementary filters.

### 1. Preparation

Make sure you have wiringpi installed on your Pi. If not, install it with ```apt-get install wiringpi```.
Add the Pi4J library to your projects build path. (You might have to change a few imports)

### 2. Example

To use this driver, you just have to create a new instance of the LSM9DS1 class. Afterwards you can start reading data.
```
    // Create an instance
    LSM9DS1 imu = new LSM9DS1();

    // Read and save current data (also calculates Euler angles)
    imu.read();

    // Get acceleration data
    float accelerometerX = imu.getAccelerometerData()[0];
    float accelerometerY = imu.getAccelerometerData()[1];
    float accelerometerZ = imu.getAccelerometerData()[2];

    // Get gyroscope data
    float gyroscopeX = imu.getGyroscopeData()[0];
    float gyroscopeY = imu.getGyroscopeData()[1];
    float gyroscopeZ = imu.getGyroscopeData()[2];

    // Get magnetometer data
    float magnetometerX = imu.getMagnetometerData()[0];
    float magnetometerY = imu.getMagnetometerData()[1];
    float magnetometerZ = imu.getMagnetometerData()[2];

    // Get temperature
    int temperature = imu.getTemperature();

    // Calculating and obtaining the Euler angles
    double pitch = imu.getPitch();
    double roll = imu.getRoll();
    double yaw = imu.getYaw();
```

### 3. Other

This driver was more or less a conversion from the driver written in C by Adafruit (https://github.com/adafruit/Adafruit_LSM9DS1)
This driver will most likely be updated in the near future. (Primarily implementing yaw calculation)