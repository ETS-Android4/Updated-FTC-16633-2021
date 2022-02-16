package org.firstinspires.ftc.teamcode.opmode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.control.Pipeline_Target_Detect;
import org.firstinspires.ftc.teamcode.util.BaseRobot;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class CameraTester extends OpMode {
    BaseRobot robot = new BaseRobot();
    WebcamName webcamName;
    OpenCvCamera camera;
    Pipeline_Target_Detect myPipeline;
    int zone = 3;
    double xPos = -1;
    int width = 64;
    int height = 64;
    int callTimes = 0;
    int callTimes2 = 0;
    boolean preValueLBumper = false;
    boolean preValueRBumper = false;
    boolean preValueX = false;
    boolean preValueB = false;
    boolean preValueA = false;
    boolean cameraOpen = false;

    public void init() {
    }

    public void loop() {
        if (gamepad1.left_bumper && gamepad1.left_bumper != preValueLBumper) {
            width -= 5;
            closeCamera();
            openCamera();
        }
        preValueLBumper = gamepad1.left_bumper;

        if (gamepad1.right_bumper && gamepad1.right_bumper != preValueRBumper) {
            width += 5;
            closeCamera();
            openCamera();
        }
        preValueRBumper = gamepad1.right_bumper;
        if (gamepad1.x && gamepad1.x != preValueX) {
            height -= 5;
            resetCamera();
        }
        preValueLBumper = gamepad1.x;
        if (gamepad1.b && gamepad1.b != preValueB) {
            height += 5;
            resetCamera();
        }
        preValueB= gamepad1.b;
        if(Math.abs( gamepad1.left_stick_x) > 0)
        {

                width += Math.floor(gamepad1.left_stick_x * 2.9);//if pushed all the way to the side will decrease or increase by two if middle by 1 if
                resetCamera();

        }
        if(Math.abs( gamepad1.left_stick_y) > 0)
        {

                height += Math.floor(gamepad1.left_stick_y * 2.9);//if pushed all the way to the side will decrease or increase by two if middle by 1 if
                resetCamera();

        }



        if(cameraOpen)
        {
            xPos = myPipeline.getXPos();
            if (xPos < 100) {
                zone = 1;
            } else if (xPos > 200) {
                zone = 3;
            } else {
                zone = 2;
            }
            telemetry.addLine("XPOs: " + xPos);
            telemetry.addLine("Zone: " + zone);
        }

        if (gamepad1.a && gamepad1.a != preValueA) {

            if (!cameraOpen) {//if the camera isnt open lets open it
                cameraOpen = true;
                openCamera();
            }
            else if (cameraOpen) {
                cameraOpen = false;
                closeCamera();

            }
        }
        preValueA = gamepad1.a;

        telemetry.addLine("Width: " + width);
        telemetry.addLine("Height: " + height);


        telemetry.addLine("Camera Open: " + cameraOpen);
        telemetry.update();
    }
    private void resetCamera()
    {
        if(cameraOpen)
        {
            closeCamera();
            openCamera();
        }
        else
        {
            openCamera();
            cameraOpen = true;
        }
    }
    private void openCamera()
    {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new Pipeline_Target_Detect(width, height);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(myPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
    private void closeCamera()
    {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }


}




