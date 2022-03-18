package org.firstinspires.ftc.teamcode.opmode.testers;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.control.Pipeline_Target_Detect;
import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
//@Disabled
public class CameraTesterAS extends OpMode {
BaseRobot robot = new BaseRobot();


    WebcamName webcamName;
    OpenCvCamera camera;

    Pipeline_Target_Detect myPipeline;
    int zone=3;
    double xPos = -1;
    public boolean opModeIsActive = false;
    @Override
    public void init() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);



        // Send telemetry message to signify robot waiting;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new Pipeline_Target_Detect(20, 40);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                camera.setPipeline(myPipeline);
            }
            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    //delay(7);
    public void init_loop()
    {
        if(xPos < 100)
        {
            zone = 1;
        }
        else if(xPos > 200)
        {
            zone = 3;
        }
        else
        {
            zone = 2;
        }
        telemetry.addData("XPos: ", xPos);
        telemetry.addData("Zone: ", zone);
        telemetry.update();







    }
    //@Override
    public void start()
    {
        opModeIsActive = true;
        camera.stopStreaming();
        camera.closeCameraDevice();
        /*delay(1);
        forward(12);
        delay(1);
        right(90);
        delay(1);
        forward(22);
        delay(1);
        right(90);
        delay(1);
        forward(-6);
        delay(1);
        if (zone == 1) {
            robot.slider.setTargetPosition(bottom);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);
        } else if (zone == 2) {
            robot.slider.setTargetPosition(middle);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);
        } else {
            robot.slider.setTargetPosition(top);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);        }

        // RUN CODE HERE
        robot.bucket.setPosition(.8);
        delay(2);
        robot.bucket.setPosition(.08);
        delay(1);
        robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(1);

        telemetry.addData("XPos: ", xPos);
        telemetry.addData("Zone: ", zone);
        telemetry.update();
        delay(3);
        forward(6);
        delay(1);
        left(90);
        delay(1);
        velocity = 1;
        forward(30);*/

        telemetry.addData("XPos: ", xPos);
        telemetry.addData("Zone: ", zone);
        telemetry.update();



        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
    }
    @Override
    public void loop()
    {

    }



}
