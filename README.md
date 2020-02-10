### LSM9DS1 for Java

This repository provides you with an easy to use driver for the LSM9DS1 Inertial Measurement Unit (IMU) written in Java,
focused to be used on a Raspberry Pi.
The sensor includes an accelerometer, gyroscope, magnetometer and a temperature sensor.

The driver allows you to read data from the IMU, but also supplies more complex methods for calculating the
Euler angles (pitch, roll and yaw) while using low-pass and complementary filters.

1. Preparation
    Make sure you have wiringpi installed on your Pi. If not, install it with 'apt-get install wiringpi'.
    Add the Pi4J library to your projects build path. (You might have to change a few imports)

2. Example
    To use this driver, you just have to create a new instance of the LSM9DS1 class. Afterwards you can start reading data.
    ```
    // Create an instance
    LSM9DS1 imu = new LSM9DS1();

    // Read and save current data
    imu.read();

    // Get acceleration data
    float accelerometerX = imu.getAccelerationData().getX();
    float accelerometerY = imu.getAccelerationData().getY();
    float accelerometerZ = imu.getAccelerationData().getZ();

    // Get gyroscope data
    float gyroscopeX = imu.getGyroscopeData().getX();
    float gyroscopeY = imu.getGyroscopeData().getY();
    float gyroscopeZ = imu.getGyroscopeData().getZ();

    // Get magnetometer data
    float magnetometerX = imu.getMagnetometerData().getX();
    float magnetometerY = imu.getMagnetometerData().getY();
    float magnetometerZ = imu.getMagnetometerData().getZ();

    // Get temperature
    int temperature = imu.getTemperature();

    // Calculating and obtaining the Euler angles as a formatted string
    String eulerAngles = imu.getAngles();
    ```

3. Other
    This driver was more or less a conversion from the driver written in C by Adafruit (https://github.com/adafruit/Adafruit_LSM9DS1)
    This driver will most likely be updated in the near future. (Primarily implementing yaw calculation)