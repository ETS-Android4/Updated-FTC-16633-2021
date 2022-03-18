package org.firstinspires.ftc.teamcode.util.fields;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class PIDFields {
    // todo; IMPORTANT: CHANGES IN FTCDASH DO NOT PROPAGATE!!!
    // todo; PLEASE COPY UPDATED VALUES FROM FTCDASH INTO ANDROID STUDIO ONCE FINISHED TUNING

    // PID Coefficients for Autonomous Turning Using the IMU
    public static PIDCoefficients IMU_TURN_PID = new PIDCoefficients(0.013, 0, 0.01);
    // PID (and possibly f soon) Coefficients for Autonomous Forward and Backward Movement using Encoder Readings
    // TODO: tune
    public static PIDCoefficients MOVE_PID = new PIDCoefficients(0.1, 0.1, 0.1);
    public static double imuModifier = .7;
    public static double marginOfError = .5;//just change this back from 1
    public static double rightImuTurnMod = 1;
    public static double leftImuTurnMod = .91;
    public static double AUTONOMOUSTURNMOD = 1.16489;

}
