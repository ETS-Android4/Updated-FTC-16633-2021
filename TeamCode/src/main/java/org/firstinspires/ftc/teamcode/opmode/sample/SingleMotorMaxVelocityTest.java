package org.firstinspires.ftc.teamcode.opmode.sample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.storage.SingleMotorMaxStaticVelocity;

@TeleOp
// TODO: delete if using
@Disabled
public class SingleMotorMaxVelocityTest extends LinearOpMode {
    DcMotorEx flywheelMotor;
    double currentVelocity = 0.0;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: add motor that is being tested
        flywheelMotor = null;
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Important: ", "Make sure to have a full battery!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            flywheelMotor.setPower(1.0);
            currentVelocity = flywheelMotor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.addData("static variable velocity", SingleMotorMaxStaticVelocity.maxStaticVelocity);
            telemetry.update();

            SingleMotorMaxStaticVelocity.maxStaticVelocity = maxVelocity;
        }

    }
}
