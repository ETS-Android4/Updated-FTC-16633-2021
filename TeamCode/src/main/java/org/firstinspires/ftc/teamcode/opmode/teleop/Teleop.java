package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.fields.PositionFields;

//things I still want to make
/*
1 - figure out the math and code to keep track of the robots orientation and make a button to orient robot maybe expiriment with rewrting turn code
2 - make intake spin in for a seconds and then out to automatically fix blocks that are stuck
3- player2 right stick button control linear slider position maybe???
4- more capstone arm automation, maybe one button for standing up and another for on the ground
5- quality of life fixes- when bucket is put into holding position intake is stopped, intake bar moves if in holding and linear slider is pressed up and doesnt break :)
 */

@TeleOp
public class Teleop extends OpMode {
    //GamePad variables
    Gamepad player1 = null;
    Gamepad player2 = null;
    Gamepad unecessaryFunctionsGamepad = null;
    Gamepad necessaryFunctionsGamepad = null;
    boolean dualControls = true;
    /*There are basically 4 types of controlled robot functions
    1.) the functions that player 1 always has control over
    2.) the functions player2 always ahs control over
    3.) the functions that are unecessary to the robots functionality but are controlled by player 1
     4.) the functions that are necessary to the robot's functionality but are controlled by player 2
    the player1 gamepad object controls all of the first
    the player2 gamepad object controls all of the second
    the unecessaryFunctionsGamepad controls all of the third
    the necessaryFunctionsGamepad  controls all of the fourth*/

    /*Gamepad button Variables*/

    //LETTER BUTTONS
    boolean preValueA = false;
    boolean preValueA2 = false;
    boolean preValueB = false;
    boolean preValueB2 = false;
    boolean preValueX = false;
    boolean preValueX2 = false;
    boolean preValueY = false;
    boolean preValueY2 = false;

    //DPAD BUTTONS
    boolean preValueDRight = false;
    boolean preValueDRight2 = false;
    boolean preValueDLeft = false;
    boolean preValueDLeft2 = false;
    boolean preValueDUp = false;
    boolean preValueDUp2 = false;
    boolean preValueDDown = false;
    boolean preValueDDown2 = false;
    //TRIGGER AND BUMPER BUTTONS
    boolean leftStickButton = false;
    boolean leftStickButton2 = false;
    boolean rightStickButton = false;
    boolean rightStickButton2 = false;
    boolean preValueRBumper = false;
    boolean preValueRBumper2 = false;
    boolean preValueLBumper = false;
    boolean preValueLBumper2 = false;
    //OTHER BUTTONS
    boolean preValueBack = false;
    boolean preValueBack2 = false;
    boolean preValueStart = false;
    boolean preValueStart2 = false;
    boolean preValueGuide = false;
    boolean preValueGuide2 = false;

    //MOTOR STATE TRACKING VARIABLES\\

    //DRIVE VARIABLES
    double speed = 1; //tracks speed of the drive
    double curspeed = 1;//tracks the current speed of the motors and is used to remember the past speed when activating a speed boost
    private boolean speedCap = false;//is ture when linear slider is extended

    //INTAKE VARIABLES
    double intake = 0; //if 0 intake is off, if 1 intake is on -1 is intake is in reverse
    //LINEAR SLIDER
    int target = 0;//the position the slider is told to run to
    int sliderState = 0; //tracks the position of the slider in regard to the dpad shortcuts;
    //0 is rest; 1 is shared shipping hub; 2 is low goal; 3 is middlegoal; 4 is highgoal; 5 is capping position
    double sliderPower = 0;// Variable for linear slider manual power
    //BUCKET VARIABLES
    int bucketState = 0; //bucketState of a toggle for bucket 0 if at intake 1 if at holding 2 if at outtake
    double bucketTarget = 0;

    //Intake Bar variables
    int intakeBar = 0; // 0 if not over 1 if over
    //CAPSTONE
    double capstoneTarget = 0;// the postion the capstone arm is told to run to
    double capstonePower = 0;//variable for detecting if triggers are pressed and how much they are pressed
    int capstoneState = 0; // 0 if at rest 1 if picking up and 2 if capping

    //CAROUSEL
    double carouselmodifier = 0;
    double maxCarouselSpeed = .8;
    boolean redCarouselActive = false;
    boolean blueCarouselActive = false;
    ElapsedTime carouselTimer = null;
    //Gamemode Tracking Variables
    ElapsedTime runtime;
    boolean matchActive = true;
    int gameMode = 0; //0 == auto 1 == driver control 2== 15 before endgame 3 == endgame
    String currentGameMode = "";
    //OTHER
    boolean practiceMode = false; //if practice mode is true than the dual controls are kept but the timer is ignored
    static boolean onRedAlliance = false; //tracks what color alliance the robot is on
    BaseRobot robot = new BaseRobot();

    @Override
    public void init() {
        // Init hardware variables
        robot.init(hardwareMap, false);

        // Init telemetry


        player1 = gamepad1;
        player2 = gamepad2;
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status: ", "Robot Ready");
        telemetry.addLine("");
        telemetry.addData("Warning: ", "Servo Moves on Initalization");

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset the value of the encoder of the linear slider
        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = robot.slider.getCurrentPosition();
        capstoneTarget = PositionFields.CAPSTONE_REST;


        // Reset the position of the servo for intaking
        /*robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
        robot.slider.setPower(1);*/
        robot.bucket.setPosition(PositionFields.BUCKET_INTAKE);
        robot.intakeBar.setPosition(PositionFields.BUCKET_NOT_OVER);
        robot.capstoneArm.setPosition(PositionFields.CAPSTONE_REST);

    }

    @Override
    public void loop() {
        if (matchActive) {
            checkDualControl();
            checkGamemode();
            checkReset();
            checkPracticeMode();
            checkBucket();
            checkCapstoneArm();
            checkCarousel();
            checkDrive();
            checkSlider();
            checkIntake();
            printTelemetry();
        }
    }



    //Voids for checking motor and servos

    //BUCKET- A button on necessary gamepad for automatic  Manual Control is player 2 dpad left and right
    public void checkBucket() {
        if(necessaryFunctionsGamepad.a && necessaryFunctionsGamepad.a != preValueA) {
            if(bucketState == 0)
            {
                bucketState = 1;
                intakeBar = 1;
                bucket(PositionFields.BUCKET_HOLDING);
                robot.intakeBar.setPosition(PositionFields.BUCKET_OVER);
            }

            else if(bucketState == 1)
            {
                if(intakeBar == 1)
                {
                    intakeBar = 0;
                    robot.intakeBar.setPosition(PositionFields.BUCKET_NOT_OVER);
                }
                else
                {
                    bucketState = 2;
                    bucket(PositionFields.BUCKET_OUTTAKE);
                }
            }
            else if(bucketState == 2)
            {
                bucketState = 0;
                bucket(PositionFields.BUCKET_INTAKE);
                }
        }
        preValueA = necessaryFunctionsGamepad.a;

        //MAUNAL CONTROL
        if(player2.dpad_left)
        {
            bucketTarget -= 1.0 / 500;
        }
        else if(player2.dpad_right){
            bucketTarget += 1.0 / 500;
        }
        bucket(bucketTarget);
    }
    //CAROUSEL: uses player 1 and player2 b and x adn player 2's right and left bumeprs  and player 1 and 3 left stick buttons
    public void checkCarousel() {
        if (player1.b && player1.b != preValueB) {
            carouselTimer = null;
            if (!redCarouselActive && !blueCarouselActive) {
                robot.carousel.setPower(-1 * (maxCarouselSpeed + carouselmodifier));
                redCarouselActive = true;
            } else {
                robot.carousel.setPower(0);
                redCarouselActive = false;
                blueCarouselActive = false;
            }
        }
        preValueB = player1.b;
        if (player1.x && player1.x != preValueX) {
           carouselTimer = null;
            if (!blueCarouselActive && !redCarouselActive) {
                robot.carousel.setPower(maxCarouselSpeed + carouselmodifier);
                blueCarouselActive = true;
            } else {
                robot.carousel.setPower(0);
                blueCarouselActive = false;
                redCarouselActive = false;
            }

        }
        preValueX = player1.x;
        if(player1.left_stick_button || player2.left_stick_button)
        {
            if(redCarouselActive) {
                robot.carousel.setPower(-1);
                redCarouselActive = false;
            }
            else if(blueCarouselActive)
            {
                robot.carousel.setPower(1);
                blueCarouselActive = false;
            }
        }
        else if(!redCarouselActive && !blueCarouselActive) robot.carousel.setPower(0);

        if (player2.b && player2.b != preValueB2 || (carouselTimer != null && carouselTimer.seconds() >= PositionFields.CAROUSEL_SPEED_AFTER && !blueCarouselActive)) {
            if (!redCarouselActive && !blueCarouselActive) {
                robot.carousel.setPower(-1 * (maxCarouselSpeed + carouselmodifier));
                redCarouselActive = true;
                carouselTimer = new ElapsedTime();
            }
            else if(carouselTimer != null && carouselTimer.seconds() >= PositionFields.CAROUSEL_SPEED_AFTER)
            {
                robot.carousel.setPower(-1);
                carouselTimer = null;
            }
            else {
                robot.carousel.setPower(0);
                redCarouselActive = false;
                blueCarouselActive = false;
            }
        }
        preValueB2 = player2.b;
        if (player2.x && player2.x != preValueX2 || (carouselTimer != null && carouselTimer.seconds() >= PositionFields.CAROUSEL_SPEED_AFTER && !redCarouselActive)) {
            if (!redCarouselActive && !blueCarouselActive) {
                robot.carousel.setPower(1 * (maxCarouselSpeed + carouselmodifier));
                blueCarouselActive = true;
                carouselTimer = new ElapsedTime();
            }
            else if(carouselTimer != null && carouselTimer.seconds() >= PositionFields.CAROUSEL_SPEED_AFTER)
            {
                robot.carousel.setPower(-1);
                carouselTimer = null;
            }
            else {
                robot.carousel.setPower(0);
                redCarouselActive = false;
                blueCarouselActive = false;
            }
        }
        preValueX2 = player2.x;









        //Carosuel Speed modification
        if (player2.right_bumper && player2.right_bumper != preValueRBumper2) {
            if ((carouselmodifier + maxCarouselSpeed) >= 1) {
                carouselmodifier = .1;
            } else {
                carouselmodifier += 0.1;
            }

        }
        preValueRBumper2 = player2.right_bumper;

        if (player2.left_bumper && player2.left_bumper != preValueLBumper2) {
            if ((maxCarouselSpeed + carouselmodifier) <= 0) {
                carouselmodifier = -.9;
            } else {
                carouselmodifier -= .1;
            }

        }
        preValueLBumper2 = player2.left_bumper;
    }
    //INTAKE: uses player 1 dpad left and right along with player 1's left stick x plane
    public void checkIntake() {
        if (player1.dpad_right && player1.dpad_right != preValueDRight) {
            if (intake == 0 || intake == -1) intake = 1;
            else intake = 0;
            robot.intake.setPower(intake);
        }
        preValueDRight = player1.dpad_right;
        if (player1.dpad_left && player1.dpad_left != preValueDLeft) {
            intake = -1;
            robot.intake.setPower(intake);
        }
        preValueDLeft = player1.dpad_left;
        if (intake == 0) {
            robot.intake.setPower(player1.left_stick_x);//right stick x controlls intake
        }
    }
    //Check Capstone - Automatic uses unecessary a button and manual uses player2 triggers
    public void checkCapstoneArm() {
        //Automactic COntrol
        if(unecessaryFunctionsGamepad.a && unecessaryFunctionsGamepad.a != preValueA2) {
            if (capstoneState == 0) {
                intakeBar = 0;
                capstoneState = 1;
                robot.intakeBar.setPosition(PositionFields.BUCKET_NOT_OVER);
                slider(PositionFields.sliderCapstoneIntake);
                capstone(PositionFields.CAPSTONE_INTAKE);
            }
            else if(capstoneState == 1)
            {
                capstoneState = 2;
                slider(2500);
                capstone(PositionFields.CAPSTONE_CAPPING);
            }
            else
            {
                resetRobot();
                capstoneState = 0;
            }
        }
        preValueA2 = unecessaryFunctionsGamepad.a;

        //Manual Control
        capstonePower = unecessaryFunctionsGamepad.right_trigger - unecessaryFunctionsGamepad.left_trigger;
        if (Math.abs(capstonePower) > 0) { //if the triggers are pressed and in endgame or not in matchmode
            capstoneTarget += capstonePower / 500;//adds or subtracts between .01 and 1 to the target
            if (capstoneTarget >= .9) {
                capstoneTarget = .9;//if the target is greater than  1 than the servo target = 1
            } else if (capstoneTarget > 100 && capstoneTarget > 0)// if the linear slider is above a certain position than the capstone arm can be moved back all the way
            {
            } else if (capstoneTarget <= .24) {
                capstoneTarget = .24;
            }

            robot.capstoneArm.setPosition(capstoneTarget);
        }

    }
    //SLIDER - dpad up and down on player 2 and right and left trigger on necessary controls
    public void checkSlider() {
        //Automatic COntrol
        if (player2.dpad_up && player2.dpad_up != preValueDUp) {
            if (intakeBar == 0) {
                if (sliderState == 0 || sliderState == 1 || sliderState == 2) {
                    sliderState = 3;
                    telemetry.addLine("in");
                    target = PositionFields.TOP;
                } else if (sliderState == 3) {
                    sliderState = 4;
                    target = 2500;
                    capstoneTarget = PositionFields.CAPSTONE_CAPPING;
                }
                slider(target);
                capstone(capstoneTarget);
            }
        }
        preValueDUp = player2.dpad_up;
        if (player2.dpad_down && player2.dpad_down != preValueDDown) {
            if (intakeBar == 0) {
                if (sliderState == 4) {
                    sliderState = 3;
                    target = PositionFields.TOP;
                } else if (sliderState == 3) {
                    sliderState = 2;
                    target = PositionFields.MIDDLE;
                } else if (sliderState == 2) {
                    sliderState = 1;
                    target = PositionFields .LOW;
                } else if (sliderState == 1) {
                    sliderState = 0;
                    target = 0;
                }
                slider(target);

            }
        }
        preValueDDown = player2.dpad_down;
        //Manual Control
        sliderPower = necessaryFunctionsGamepad.right_trigger - necessaryFunctionsGamepad.left_trigger;
        if (Math.abs(sliderPower) > 0 && intakeBar == 0) {
            target += sliderPower * 10;//adds 10 to the target
            if (target > 2500) {
                target = 2500;//if the target is greater than the maxcounts than the target = max counts
            } else if (target < 0) {
                target = 0;
            }
            slider(target);
        }
    }
    //DRIVE- uses player 1 back, and left stick y axis and right stick x axis
    public void checkDrive() {
        // Modifiable variables for current speed calculations
        double forward;
        double turn;
        double total1;
        double total2;

        if(robot.slider.getCurrentPosition() > 2300) speedCap = true;//speed cap enabled when capstone is up
        else speedCap = false;
        // Toggle speed cap for motors
        if (player1.back && player1.back != preValueBack) {
            if (speed == 1) {
                speed = 0.5;
                curspeed = 0.5;
            } else if (speed != 1) {
                speed = 1;
                curspeed = 1;
            }
        }
        preValueBack = player1.back;
        if (player1.left_stick_button) {
            if (speed != 1) {
                curspeed = speed;
            }
            speed = 1;
        } else {
            if(!speedCap) {
                speed = curspeed;
            }
        }
       /* if(player1.right_stick_button)
        {
            curspeed = speed;
            speed = .25;
        }*/
        // Adjusting the input by the speed cap
        forward = player1.left_stick_y * speed;
        turn = player1.right_stick_x * speed * PositionFields.TURN_SPEED_MODIFIER;
        // Kinematics
        total1 = forward - turn;
        total2 = forward + turn;
        // Setting motor powers
        robot.leftFront.setPower(total1);
        robot.rightFront.setPower(total2);
        robot.leftRear.setPower(total1);
        robot.rightRear.setPower(total2);
    }
    //Robot Reset- both players y buttons
    public void checkReset() {
        if ((player1.y && player1.y != preValueY) || (player2.y && player2.y != preValueY2)) {
            resetRobot();
        }
        preValueY = player1.y;
        preValueY2 = player2.y;
    }


    //Other voids for checking othre stuff
    public void checkDualControl() {


        if (gamepad1.guide && gamepad1.guide != preValueGuide) {
            if (dualControls) {
                dualControls = false;
                player1 = gamepad1;
                unecessaryFunctionsGamepad = gamepad2;
                player2 = gamepad2;
                necessaryFunctionsGamepad = gamepad1;
            } else if (!dualControls) {
                dualControls = true;
            }
        }
        preValueGuide = gamepad1.guide;

        // If the second player presses the guide button then all necessary controls go to it
        if (gamepad2.guide && gamepad2.guide != preValueGuide2) {
            if (dualControls) {
                dualControls = false;//setDualControls equal to false
                player1 = gamepad2;//make player one be controlled by the second player
                unecessaryFunctionsGamepad = gamepad1;//give control of unecessary function in temperoraryGamepad1
                player2 = gamepad1;//give control of normal player2 functions to gamepad1
                necessaryFunctionsGamepad = gamepad2;//give control of necessary functiosn to gamepad2
            } else if (!dualControls) {
                dualControls = true;
            }
        }
        preValueGuide2 = player2.guide;

        //if Dual controls is true then reset all controls back to original controllers
        if (dualControls) {
            player1 = gamepad1;
            unecessaryFunctionsGamepad = gamepad1;
            player2 = gamepad2;
            necessaryFunctionsGamepad = gamepad2;
        }
    }
    public void checkGamemode() {
        if (runtime == null) {
            runtime = new ElapsedTime();
        } else if (runtime.seconds() < 121 || practiceMode) { //it the match is active or it is practice mode


            if (runtime.seconds() < 76 && runtime.seconds() > 75) {//if 15 seconds before endgame
                rumble(1);
                gameMode = 2;
                currentGameMode = "Get Capstone Time!";
            } else if (runtime.seconds() > 90) {
                gameMode = 3;
                currentGameMode = "ENDGAME!!!";
            }// game is in Endgame

            else if (runtime.seconds() < 90) {
                gameMode = 1;
                currentGameMode = "DRIVER CONTROL";
            }// in drvier control
        } else
            matchActive = false; //once 121 seconds is up the match is over and the teleop shuts down
    }
    public void checkPracticeMode() {
        if(gamepad1.start && gamepad1.start != preValueStart || gamepad2.start && gamepad2.start != preValueStart2)
        {
            if(practiceMode) practiceMode = false;
            else practiceMode = true;
        }
        preValueStart = gamepad1.start;
        preValueStart2 = gamepad2.start;
    }
    public void printTelemetry() {
        if (practiceMode) {
            //OUTPUTTING bucketStateS OF VARIABLES
            telemetry.addLine("/////LinearSlider/////");
            String sliderStateStr;
            if (sliderState == 0) sliderStateStr = "Intake";
            else if (sliderState == 1) sliderStateStr = "Shared Shipping Hub level";
            else if (sliderState == 2) sliderStateStr = "Low Level";
            else if (sliderState == 3) sliderStateStr = "Middle Level";
            else if (sliderState == 4) sliderStateStr = "Top Level";
            else sliderStateStr = "Capping Level";
            telemetry.addLine("Slider Position is " + robot.slider.getCurrentPosition() + " and target of the slider is" + target);
            telemetry.addLine("Slider" + sliderStateStr);
            telemetry.addLine("");


            telemetry.addLine("/////Intake/////");
            telemetry.addLine("Intake: " + intake);
            telemetry.addLine("IntakeBar: " + intakeBar);

            telemetry.addLine("");

            telemetry.addLine("/////Bucket/////");
            telemetry.addLine("bucket: " + bucketState);
            telemetry.addLine();

            telemetry.addLine("/////Capstone/////");
            telemetry.addLine("Capstone:  " + capstoneTarget);
            telemetry.addLine();


            telemetry.addLine("/////Drive/////");
            telemetry.addLine("Drive Speed:" + speed);
            telemetry.addLine();
            String carouselTel = "";
            if(robot.carousel.getPower() == 0) carouselTel = "INACTIVE";
            else if(robot.carousel.getPower() != 1 && robot.carousel.getPower() != -1 ) carouselTel = "SPINNING SPEED: SLOW";
            if(robot.carousel.getPower() == 1 || robot.carousel.getPower() == -1) carouselTel = "SPINNING SPEED: FAST";


            telemetry.addLine("/////Carousel/////");
            telemetry.addLine("CAROUSEL: " + carouselTel);
            telemetry.addLine("Blue Carousel Active = " + blueCarouselActive);
            telemetry.addLine("Red Carousel Active = " + redCarouselActive);
            telemetry.addLine(" Carousel speed = " + maxCarouselSpeed + carouselmodifier);
            telemetry.addLine("Carousel speed modifier = " + carouselmodifier);
            telemetry.addLine();

            //TO BE ADDED

            telemetry.addLine("/////Orientation/////");
            /*telemetry.addLine("Orientation" + robot.getCurrentOrientation());
            telemetry.addLine("heading" + robot.getIntegratedHeading());*/
            telemetry.addLine();

            //OTHER DATA
            telemetry.addLine("//////OTHER DATA/////");
            telemetry.addLine("Time: " + runtime.seconds());
            telemetry.addLine("Game Portion: " + currentGameMode);
            telemetry.addLine("practiceMode:  " + practiceMode);
            telemetry.addLine("DualControlsActive:  " + dualControls);


        } else {
            telemetry.addLine("Time" + runtime.seconds());
            telemetry.addLine("Carousel speed = " + maxCarouselSpeed + carouselmodifier);
            telemetry.addLine("Drive Speed:" + speed);
            telemetry.addLine("Game Portion:" + currentGameMode);
            telemetry.addLine("DualControls: " + dualControls);
            telemetry.addLine("practiceMode: " + practiceMode);


        }
        telemetry.update();
    }
    public void rumble(double seconds) {
        //gamepad1.rumble((int) (1000 * seconds));
        gamepad2.rumble((int) (1000 * seconds));
    }
    public void resetRobot() {
        robot.reset();
        sliderState = 0;
        bucketState = 0;
        bucketTarget = PositionFields.BUCKET_INTAKE;
        intakeBar = 0;
        target = 0;
        capstoneState = 0;
        capstoneTarget = PositionFields.CAPSTONE_REST;
    }



    //voids for moving servos and motors
    public void bucket(double position) {
    bucketTarget = position;
    robot.bucket.setPosition(bucketTarget);
    }
    public void capstone(double position) {
        capstoneTarget = position;
        robot.capstoneArm.setPosition(position);
    }
    public void slider(int position) {
        target = position;
        robot.slider.setTargetPosition(position);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);//runs the motor to the target positions
        robot.slider.setPower(1);
    }




}