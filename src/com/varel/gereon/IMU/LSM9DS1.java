package com.varel.gereon.components.IMU;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.varel.gereon.components.IMU.Constants.LSM9DS1_Accelerometer_Range;
import com.varel.gereon.components.IMU.Constants.LSM9DS1_Gyroscope_Scale;
import com.varel.gereon.components.IMU.Constants.LSM9DS1_Magnetometer_Gain;

import java.io.IOException;
import java.text.DecimalFormat;

import static com.varel.gereon.components.IMU.Constants.LSM9DS1_Constants.*;

public class LSM9DS1 {

    private int temperature = 0;
    private long deltaTime = 0;
    private double rollXLFilteredOld = 0.0d;
    private double pitchXLFilteredOld = 0.0d;
    private double roll = 0.0d;
    private double pitch = 0.0d;
    private double yaw = 0.0d;
    private I2CBus bus;
    private I2CDevice magnetometer;
    private I2CDevice accelerometerGyro;
    private AccelerometerData accelerometerData;
    private GyroscopeData gyroscopeData;
    private MagnetometerData magnetometerData;

    /**
     * Creates an instance using the default settings. (2G, 245DPS, 4GAUSS)
     */
    public LSM9DS1() {
        this.setup(LSM9DS1_Accelerometer_Range.LSM9DS1_ACCELEROMETER_2G, LSM9DS1_Gyroscope_Scale.LSM9DS1_GYROSCOPE_245DPS, LSM9DS1_Magnetometer_Gain.LSM9DS1_MAGNETOMETER_4GAUSS);
    }

    /**
     * Creates an instance using the specified settings.
     *
     * @param accelerometerRange AccelerometerRange that will be used.
     * @param gyroscopeScale GyroscopeScale that will be used.
     * @param magnetometerRange MagnetometerRange that will be used.
     */
    public LSM9DS1(LSM9DS1_Accelerometer_Range accelerometerRange, LSM9DS1_Gyroscope_Scale gyroscopeScale, LSM9DS1_Magnetometer_Gain magnetometerRange) {
        this.setup(accelerometerRange, gyroscopeScale, magnetometerRange);
    }

    private void setup(LSM9DS1_Accelerometer_Range accelerometerRange, LSM9DS1_Gyroscope_Scale gyroscopeRange, LSM9DS1_Magnetometer_Gain magnetometerRange) {
        try {
            this.bus = I2CFactory.getInstance(I2CBus.BUS_1);
            this.magnetometer = this.bus.getDevice(LSM9DS1_M_ADDR);
            this.accelerometerGyro = this.bus.getDevice(LSM9DS1_XG_ADDR);
            this.accelerometerData = new AccelerometerData();
            this.gyroscopeData = new GyroscopeData();
            this.magnetometerData = new MagnetometerData();
            this.begin(accelerometerRange, gyroscopeRange, magnetometerRange);
        } catch (I2CFactory.UnsupportedBusNumberException | IOException ex) {
            ex.printStackTrace();
        }
    }

    private void begin(LSM9DS1_Accelerometer_Range accelerometerRange, LSM9DS1_Gyroscope_Scale gyroscopeRange, LSM9DS1_Magnetometer_Gain magnetometerRange) {
        // soft reset & reboot accelerometer/gyroscope
        this.writeRegister(true, LSM9DS1_REGISTER_CTRL_REG8, (byte) 0x05);
        try { // TODO: might be unnecessary
            Thread.sleep(10);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
        int id = readRegister(true, LSM9DS1_REGISTER_WHO_AM_I_XG);
        if (id != 104) {
            System.out.println("Error identifying the device!");
            return;
        }
        // TODO: enable temperature sensor
        // this.write8(true, LSM9DS1_REGISTER_CTRL_REG10, (byte)0xF4);

        // enable gyro continuous
        this.writeRegister(true, LSM9DS1_REGISTER_CTRL_REG4, (byte) 0x38);

        // enable the accelerometer continuously
        this.writeRegister(true, LSM9DS1_REGISTER_CTRL_REG5_XL, (byte) 0x38);
        this.writeRegister(true, LSM9DS1_REGISTER_CTRL_REG6_XL, (byte) 0xC0);
        // activate accelerometer high resolution mode
        this.writeRegister(true, LSM9DS1_REGISTER_CTRL_REG7_XL, (byte) 0x80);

        // set default ranges for the various sensors
        this.setupAccelerometer(accelerometerRange);
        this.setupGyroscope(gyroscopeRange);
        this.setupMagnetometer(magnetometerRange);
    }

    private void setupAccelerometer(LSM9DS1_Accelerometer_Range range) {
        int reg = readRegister(true, LSM9DS1_REGISTER_CTRL_REG6_XL);
        reg &= ~(0b00011000);
        byte rangeData = 0b00;
        if (range == LSM9DS1_Accelerometer_Range.LSM9DS1_ACCELEROMETER_2G) {
            this.accelerometerData.setAccelerometerMgLsb(0.061f);
        } else if (range == LSM9DS1_Accelerometer_Range.LSM9DS1_ACCELEROMETER_4G) {
            rangeData = (0b10 << 3);
            this.accelerometerData.setAccelerometerMgLsb(0.122f);
        } else if (range == LSM9DS1_Accelerometer_Range.LSM9DS1_ACCELEROMETER_8G) {
            rangeData = (0b11 << 3);
            this.accelerometerData.setAccelerometerMgLsb(0.244f);
        } else if (range == LSM9DS1_Accelerometer_Range.LSM9DS1_ACCELEROMETER_16G) {
            rangeData = (0b01 << 3);
            this.accelerometerData.setAccelerometerMgLsb(0.732f);
        }
        reg |= rangeData;
        writeRegister(true, LSM9DS1_REGISTER_CTRL_REG6_XL, (byte)reg);
    }

    private void setupGyroscope(LSM9DS1_Gyroscope_Scale range) {
        int reg = readRegister(true, LSM9DS1_REGISTER_CTRL_REG1_G);
        reg &= ~(0b00110000);
        byte rangeData = 0b00;
        if (range == LSM9DS1_Gyroscope_Scale.LSM9DS1_GYROSCOPE_245DPS) {
            this.gyroscopeData.setGyroDpsDigit(0.00875f);
        } else if (range == LSM9DS1_Gyroscope_Scale.LSM9DS1_GYROSCOPE_500DPS) {
            rangeData = (0b01 << 4);
            this.gyroscopeData.setGyroDpsDigit(0.01750f);
        } else if (range == LSM9DS1_Gyroscope_Scale.LSM9DS1_GYROSCOPE_2000DPS) {
            rangeData = (0b11 << 4);
            this.gyroscopeData.setGyroDpsDigit(0.07000f);
        }
        reg |= rangeData;
        writeRegister(true, LSM9DS1_REGISTER_CTRL_REG1_G, (byte)reg);
    }

    private void setupMagnetometer(LSM9DS1_Magnetometer_Gain range) {
        // TODO
    }

    private void readAccelerometerOrGyroscope(boolean readAccelerometer) {
        byte reg;
        if (readAccelerometer) {
            reg = (byte)(0x80 | LSM9DS1_REGISTER_OUT_X_L_XL);
        } else {
            reg = (byte)(0x80 | LSM9DS1_REGISTER_OUT_X_L_G);
        }
        byte[] buffer = this.readBuffer(true, reg, (byte)6);

        int xAxis = (((int) buffer[1]) << 8) | (buffer[0] & 0xFF);
        int yAxis = (((int) buffer[3]) << 8) | (buffer[2] & 0xFF);
        int zAxis = (((int) buffer[5]) << 8) | (buffer[4] & 0xFF);

        if (readAccelerometer) {
            this.accelerometerData.x = xAxis;
            this.accelerometerData.y = yAxis;
            this.accelerometerData.z = zAxis;
        } else {
            this.gyroscopeData.x = xAxis;
            this.gyroscopeData.y = yAxis;
            this.gyroscopeData.z = zAxis;
        }
    }

    private void readMagnetometer() {
        // TODO
    }

    private void readTemperature() {
        byte reg = (byte)(0x80 | LSM9DS1_REGISTER_TEMP_OUT_L);
        byte[] buffer = this.readBuffer(true, reg, (byte)2);
        this.temperature = (((int) buffer[1] << 8) | (buffer[0] & 0xFF));
    }

    private void writeRegister(boolean writerAccelerometerGyroscope, byte register, byte value) {
        try {
            if (writerAccelerometerGyroscope) {
                this.accelerometerGyro.write(register, value);
            } else {
                this.magnetometer.write(register, value);
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    private byte readRegister(boolean readAccelerometerGyroscope, byte register) {
        byte[] value = this.readBuffer(readAccelerometerGyroscope, register, (byte)1);
        return value[0];
    }

    private byte[] readBuffer(boolean readAccelerometerGyro, byte register, byte length) {
        byte[] buffer = new byte[length];
        try {
            if (readAccelerometerGyro) {
                this.accelerometerGyro.write(register);
                this.accelerometerGyro.read(buffer, 0, length);
            } else {
                // TODO Magnetometer read
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return buffer;
    }

    /**
     * Reads and stores all data from the IMU.
     */
    public void read() {
        this.readAccelerometerOrGyroscope(true);
        this.readAccelerometerOrGyroscope(false);
        this.readMagnetometer();
        // this.readTemperature();
    }

    /**
     * Calculates Euler angles and returns them.
     *
     * @return Returns a formatted string displaying Roll, Pitch and Yaw.
     */
    public String getEulerAngles() {
        DecimalFormat formatter = new DecimalFormat("00.00");
        // Measured angle by the accelerometer
        double rollXLMeasured = Math.atan2(this.accelerometerData.getX() / EARTH_GRAVITY, this.accelerometerData.getZ() / EARTH_GRAVITY) / 2 / Math.PI * 360;
        double pitchXLMeasured = Math.atan2(this.accelerometerData.getY() / EARTH_GRAVITY, this.accelerometerData.getZ() / EARTH_GRAVITY) / 2 / Math.PI * 360;

        // Adding a low pass filter
        double rollXLFiltered = 0.9f * rollXLFilteredOld + 0.1f * rollXLMeasured;
        double pitchXLFiltered = 0.9f * pitchXLFilteredOld + 0.1f * pitchXLMeasured;
        this.rollXLFilteredOld = rollXLFiltered;
        this.pitchXLFilteredOld = pitchXLFiltered;

        // Calculating deltaTime
        long time = System.nanoTime();
        int difference = (int) ((time - this.deltaTime) / 1000000000);
        this.deltaTime = time;

        // Adding a complementary filter
        this.roll = 0.95f * (this.roll + this.gyroscopeData.getY() * difference) + 0.05f * rollXLMeasured;
        this.pitch = 0.95f * (this.pitch - this.gyroscopeData.getX() * difference) + 0.05f * pitchXLMeasured;

        return "Roll: \t" + formatter.format(roll) +
               ",\tPitch: \t" + formatter.format(pitch) +
               ",\tYaw: \t" + formatter.format(yaw) + "\n";
    }

    /**
     * @return Retuns the last saved data from the accelerometer.
     */
    public AccelerometerData getAccelerometerData() {
        return this.accelerometerData;
    }

    /**
     * @return Retuns the last saved data from the gyroscope.
     */
    public GyroscopeData getGyroscopeData() {
        return this.gyroscopeData;
    }

    /**
     * @return Retuns the last saved data from the magnetometer.
     */
    public MagnetometerData getMagnetometerData() {
        return this.magnetometerData;
    }

    /**
     * @return Retuns the last saved raw data from the temperature sensor.
     */
    public int getTemperature() {
        return this.temperature;
    }

    /**
     * Closes the currently used I2C bus.
     *
     * @throws IOException Throws when the I2C bus could not be closed.
     */
    public void close() throws IOException {
        this.bus.close();
    }

    /**
     * Obtains all the data from the IMU and returns a formatted string.
     *
     * @return Returns a formatted string displaying all of the available data from the IMU.
     */
    @Override
    public String toString() {
        String temp = "";
        DecimalFormat formatter = new DecimalFormat("00.00");
        if (this.accelerometerData != null) {
            temp += "Accelerometer: \tX->" + formatter.format(this.accelerometerData.getX()) + " \tY->" + formatter.format(this.accelerometerData.getY()) + " \tZ->" + formatter.format(this.accelerometerData.getZ()) + " \t(m/s^2)\n";
        }
        if (this.gyroscopeData != null) {
            temp += "Gyroscope: \tX->" + formatter.format(this.gyroscopeData.getX()) + " \tY->" + formatter.format(this.gyroscopeData.getY()) + " \tZ->" + formatter.format(this.gyroscopeData.getZ()) + " \t(rad/s)\n";
        }
        if (this.magnetometerData != null) {
            // TODO
        }
        // temp += "Temperature: \t" + ((float)(this.temperature / 8) + 21.0f) + " (°C)\n";

        return temp;
    }

    private static class AccelerometerData {
        private float x = 0.0f;
        private float y = 0.0f;
        private float z = 0.0f;
        private float accelerometerMgLsb = 0.0f;

        public void setAccelerometerMgLsb(float accelerometerMgLsb) {
            this.accelerometerMgLsb = accelerometerMgLsb;
        }

        public float getX() {
            return (x * accelerometerMgLsb / 1000 * EARTH_GRAVITY);
        }

        public float getY() {
            return (y * accelerometerMgLsb / 1000 * EARTH_GRAVITY);
        }

        public float getZ() {
            return (z * accelerometerMgLsb / 1000 * EARTH_GRAVITY);
        }

    }

    private static class GyroscopeData {
        private float x = 0.0f;
        private float y = 0.0f;
        private float z = 0.0f;
        private float gyroDpsDigit = 0.0f;


        public void setGyroDpsDigit(float gyroDpsDigit) {
            this.gyroDpsDigit = gyroDpsDigit;
        }

        public float getX() {
            return (x * gyroDpsDigit * DPS_TO_RADS);
        }

        public float getY() {
            return (y * gyroDpsDigit * DPS_TO_RADS);
        }

        public float getZ() {
            return (z * gyroDpsDigit * DPS_TO_RADS);
        }

    }

    private static class MagnetometerData {
        private float x = 0.0f;
        private float y = 0.0f;
        private float z = 0.0f;

        public float getX() {
            return x;
        }

        public float getY() {
            return y;
        }

        public float getZ() {
            return z;
        }

    }
}
