package org.firstinspires.ftc.teamcode.util.fields;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PositionFields {
    // Slider Positions
    public static int LOW = 200;
    public static int MIDDLE = 782;
    public static int TOP = 2000;
    public static int sliderCapstoneIntake = 700;

    // Intake Bar Positions
    public static double BUCKET_OVER = 0;
    public static double BUCKET_NOT_OVER = .3;

    // Bucket Positions
    public static double BUCKET_INTAKE = 0;
    public static double BUCKET_HOLDING = .4;
    public static double BUCKET_OUTTAKE = 1;


    // Capstone Positions
    public static double CAPSTONE_REST = .9;
    public static double CAPSTONE_INTAKE = .24;
    public static double CAPSTONE_CAPPING = .6;

    // Speed Values
    public static double MAX_CAROUSEL_SPEED = .8;
    public static double CAROUSEL_SPEED_AFTER = 1;
    public static double STOP = 1.5;
    public static double GO = 1.5;

    // Intake Timed Cycle Variable
    public static double INTAKE_IN = 2;
    public static double INTAKE_OUT = 1;

    // Modify the turning
    //public static double TURN_MOD = 4.25;

    // For Encoder Functions
    public static double COUNTS_PER_MOTOR_REV = 537.6;
    public static double MOTOR_RPM = 312;
    public static double turnMod = 3.83;
    public static double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    public static double WHEEL_DIAMETER_INCHES = 4; // For figuring out circumference
    public static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;
    public static double DRIVE_SPEED_MODIFIER = 1.0;
    public static double TURN_SPEED_MODIFIER = .8;
}
