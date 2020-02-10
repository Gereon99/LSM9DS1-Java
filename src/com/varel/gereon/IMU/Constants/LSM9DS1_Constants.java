package com.varel.gereon.components.IMU.Constants;

public final class LSM9DS1_Constants {

    public static final float EARTH_GRAVITY = 9.80665f;
    public static final float DPS_TO_RADS = 0.017453293f;

    public static final byte LSM9DS1_M_ADDR = (byte)0x1E;
    public static final byte LSM9DS1_XG_ADDR = (byte)0x6B;

    public static final byte LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F;
    public static final byte LSM9DS1_REGISTER_CTRL_REG1_G = 0x10;
    public static final byte LSM9DS1_REGISTER_CTRL_REG2_G = 0x11;
    public static final byte LSM9DS1_REGISTER_CTRL_REG3_G = 0x12;
    public static final byte LSM9DS1_REGISTER_STATUS_REG = 0x17;
    public static final byte LSM9DS1_REGISTER_OUT_X_L_G = 0x18;
    public static final byte LSM9DS1_REGISTER_OUT_X_H_G = 0x19;
    public static final byte LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A;
    public static final byte LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B;
    public static final byte LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C;
    public static final byte LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D;
    public static final byte LSM9DS1_REGISTER_CTRL_REG4 = 0x1E;
    public static final byte LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F;
    public static final byte LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20;
    public static final byte LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21;
    public static final byte LSM9DS1_REGISTER_CTRL_REG8 = 0x22;
    public static final byte LSM9DS1_REGISTER_CTRL_REG9 = 0x23;
    public static final byte LSM9DS1_REGISTER_CTRL_REG10 = 0x24;

    public static final byte LSM9DS1_REGISTER_TEMP_OUT_L = 0x05;
    public static final byte LSM9DS1_REGISTER_TEMP_OUT_H = 0x06;
    public static final byte LSM9DS1_REGISTER_OUT_X_L_XL = 0x28;
    public static final byte LSM9DS1_REGISTER_OUT_X_H_XL = 0x29;
    public static final byte LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A;
    public static final byte LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B;
    public static final byte LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C;
    public static final byte LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D;

    private LSM9DS1_Constants() { }

}
