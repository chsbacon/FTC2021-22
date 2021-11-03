// FTC Team 7080 BACON
// Autonomous code 2020-2021

package org.firstinspires.ftc.teamcode;

// All imports here that are not commented out are likely reusable and helpful for this coming year

//These three lines I believe work with the app on the phone. Not sure if the color one is something we need to think about for this year...


//imports related to the opmode

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareBACONbot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;


@Autonomous(name = "BACON: Autonomous 2021", group = "Opmode")
//@Disabled


public class ARCHIVE_Auto2021 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    HardwareBACONbot robot = new HardwareBACONbot();

    //OpenCV stuff
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    // === DEFINE CONSTANTS HERE! ===

    /*double STRAFE_SPEED = 0.3;  // Motor power global variables
    double FAST_SPEED = 1.0;
    double SLOW_SPEED = 0.2;*/

    //Here is where ints will go:
    //For example:
    // int blueTape/redTape = color sensor value
    // int blue/red = 1 or 0 depending on situation
    // int left/right = 0 or 1 depending on situation
    // int madeUpVariable; to initialize a variable
    // int distance = value of distace sensor to desired location at point of initialization
    int FRONTDIST = 860;


    // ==============================
    public void runOpMode() {
        //OpenCV stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });


        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;

        int task = 0; //1 = drop&park  2 = fullRun
        int dropPark = 1;
        int fullRun = 2;

        int side = 0; // 1 = left side start 2 = right side start

        double meetDistance = 860; //Distance from wall to the rings (CM From Wall (BackSensor))

        double lastTime = runtime.milliseconds();

        // wobbleServo and wobbleMotor states
        float grabPos = 0;
        float freePos = 1;
        float upTilt = 0;
        float downTilt = 1;

        Orientation angles;
        Acceleration gravity;

        robot.init(hardwareMap);

        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();
        //Call component setup functions here: ex. openClaw, raiseLauncher, etc.

        //It will only assign color if the buttons are pressed
        while (!gamepad1.x && !gamepad1.b) {
        }

        //This sets the strips of lights to the team color
        if (gamepad1.x) {
            teamcolor = blue;

        }

        if (gamepad1.b) {
            teamcolor = red;

        }


        telemetry.addData("teamcolor ", teamcolor);
        telemetry.update();

        // Choosing task
        telemetry.addData("Press A for drop&park, Y for fullRun", "");
        telemetry.update();
        while (!gamepad1.a && !gamepad1.y) {
        }
        if (gamepad1.a) {
            task = dropPark;
        }
        if (gamepad1.y) {
            task = fullRun ;
        }
        telemetry.addData("task ", task);
        telemetry.update();

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) robot.backDistance;

        //Wobble grabber position
        robot.wobbleServo.setPosition(grabPos);
        //robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        //OpenCV stuff
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        double WSrestPos1 = 0;

        // Don't burn CPU cycles busy-looping in this sample
        //sleep(50);

        // run until the end of the match (when driver presses STOP)





        //fullRun -------------------------------------------------------------------------------------------------------------------------


        if ((task == fullRun) && (teamcolor == red)) {
            sleep(20000);
            double timeVar = getRuntime()*1000;
            while ((getRuntime()*1000) - timeVar < 5200){
                robot.backLeftMotor.setPower(-.5);
                robot.frontRightMotor.setPower(.5);
                robot.frontLeftMotor.setPower(-.5);
                robot.backRightMotor.setPower(.5);
                telemetry.addData("runTime:", (getRuntime()*1000)- timeVar);
                telemetry.update();
            }
            stopDriving(); 
        }
        if ((task == fullRun) && (teamcolor == blue)) {
            sleep(20000); 
            double timeVar = getRuntime()*1000;
            while ((getRuntime()*1000) - timeVar < 5200){
                robot.backLeftMotor.setPower(-.5);
                robot.frontRightMotor.setPower(.5);
                robot.frontLeftMotor.setPower(-.5);
                robot.backRightMotor.setPower(.5);
                telemetry.addData("runTime:", (getRuntime()*1000)- timeVar);
                telemetry.update();
            }
            stopDriving();
        }

        //drop&park------------------------------------------------------------------------------------------------
        if ((task == dropPark) && (teamcolor == red)) {
            sleep(20000);
            double timeVar = getRuntime()*1000;
            while ((getRuntime()*1000) - timeVar < 5200){
                robot.backLeftMotor.setPower(-.5);
                robot.frontRightMotor.setPower(.5);
                robot.frontLeftMotor.setPower(-.5);
                robot.backRightMotor.setPower(.5);
                telemetry.addData("runTime:", (getRuntime()*1000)- timeVar);
                telemetry.update();
            }
            stopDriving();

        }
        if ((task == dropPark) && (teamcolor == blue)) {
            sleep(20000);
            double timeVar = getRuntime()*1000;
            while ((getRuntime()*1000) - timeVar < 5200){
                robot.backLeftMotor.setPower(-.5);
                robot.frontRightMotor.setPower(.5);
                robot.frontLeftMotor.setPower(-.5);
                robot.backRightMotor.setPower(.5);
                telemetry.addData("runTime:", (getRuntime()*1000)- timeVar);
                telemetry.update();
            }
            stopDriving();
        }
        //  -----------------------------------------------------------------------------------------------------------------------



    }

    // Functions ----------------------------------------------------------------------------------------------------------------


    //Driving Functions_______________________________________________________________________________________________________________

    //Stop Driving - Kill power to all the motors
    void stopDriving() {

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

    }

    //Drive Backwards - Used for starting the game
    void driveBackwards() {
        robot.frontLeftMotor.setPower(-0.5);
        robot.frontRightMotor.setPower(0.5);
        robot.backLeftMotor.setPower(-0.5);
        robot.backRightMotor.setPower(0.5);
    }

    //Drive Backwards Slow - Used for starting the game
    void driveBackwardsSlow() {
        robot.frontLeftMotor.setPower(-0.3);
        robot.frontRightMotor.setPower(0.3);
        robot.backLeftMotor.setPower(-0.3);
        robot.backRightMotor.setPower(0.3);
    }


    //Drive Forwards - Towards where the Backsensor is facing
    void driveForward() {
        robot.frontLeftMotor.setPower(0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.backRightMotor.setPower(-0.5);
        robot.frontRightMotor.setPower(-0.5);
    }

    //Drive Forwards Slow- Towards where the Backsensor is facing
    void driveForwardSlow() {
        robot.frontLeftMotor.setPower(0.3);
        robot.backLeftMotor.setPower(0.3);
        robot.backRightMotor.setPower(-0.3);
        robot.frontRightMotor.setPower(-0.3);
    }



    //Strafe Left - (used to strafe towards the center line for parking)
    void strafeLeft(double pwr, Orientation target) {

        //orients
        Orientation targetOrient;
        Orientation currOrient;

        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;


        while((opModeIsActive()) && (gamepad1.x)){

            //gamepad.x is here as that is the button I've been pressing to test this function
            //if you want to have this run properly, you'll need to replace gamepad.x with some other qualifier t
            // that will stop the while loop at some point, some way


            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (-error / 180) / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;

            if ((r < .07) && (r > 0)) {
                r = .07;
            } else if ((r > -.07) && (r < 0)) {
                r = -.07;
            }


            // Normalize the values so none exceeds +/- 1.0
            frontLeft = pwr + r ;
            backLeft = -pwr + r ;
            backRight = -pwr + r ;
            frontRight = pwr + r ;

            //original
            // +    +
            // -    +
            // -    +
            // +    +


            //strafe right
            // -    +
            // +    +
            // +    +
            // -    +

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }



            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);

            telemetry.addData("error", error);

            telemetry.addData("current heading", formatAngle(currOrient.angleUnit, currOrient.firstAngle));
            telemetry.addData("target heading", formatAngle(targetOrient.angleUnit, targetOrient.firstAngle));

            telemetry.update();

            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);


        }


    }



    //strafes left at the heading it was called at
    void strafeRight(double pwr, Orientation target) {

        //orients
        Orientation targetOrient;
        Orientation currOrient;

        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;


        while((opModeIsActive()) && (gamepad1.x)){

            //gamepad.x is here as that is the button I've been pressing to test this function
            //if you want to have this run properly, you'll need to replace gamepad.x with some other qualifier t
            // that will stop the while loop at some point, some way


            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (-error / 180) / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;

            if ((r < .07) && (r > 0)) {
                r = .07;
            } else if ((r > -.07) && (r < 0)) {
                r = -.07;
            }


            // Normalize the values so none exceeds +/- 1.0
            frontLeft = -pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr + r ;
            frontRight = -pwr + r ;

            //original
            // +    +
            // -    +
            // -    +
            // +    +


            //strafe right
            // -    +
            // +    +
            // +    +
            // -    +

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }



            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);

            telemetry.addData("error", error);

            telemetry.addData("current heading", formatAngle(currOrient.angleUnit, currOrient.firstAngle));
            telemetry.addData("target heading", formatAngle(targetOrient.angleUnit, targetOrient.firstAngle));

            telemetry.update();

            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);


        }


    }

//start of graham addition



    //drives straight for a desired distanced based off of the back distance sensor
    void driveStraightDistance(double pwr, Orientation target, double desiredDistance){

        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((robot.backDistance.getDistance(DistanceUnit.MM) < desiredDistance) && (opModeIsActive()))){

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (-error / 180) / (pwr);
            //r = 0;

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr - r ;
            frontRight = pwr - r ;

            frontLeft = -frontLeft;
            backLeft = -backLeft;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }



            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);

            telemetry.addData("current heading", formatAngle(currOrient.angleUnit, currOrient.firstAngle));
            telemetry.addData("target heading", formatAngle(targetOrient.angleUnit, targetOrient.firstAngle));

            telemetry.update();

            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);



        }

    }

    //drives forward at the heading it was called at
    // for example, calling this when the robot is at 60 heading, it will go to that heading, even if it gets knocked off course
    void driveStraightTime(double pwr, Orientation target, double desiredTime){

        //orients
        Orientation targetOrient;
        Orientation currOrient;


        double lastTime = runtime.milliseconds();

        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((runtime.milliseconds() < lastTime + desiredTime) && (opModeIsActive()))){



            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (-error / 180) / (pwr);
            //r = 0;

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr - r ;
            frontRight = pwr - r ;

            frontLeft = -frontLeft;
            backLeft = -backLeft;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }



            telemetry.addData("front left", "%.2f", frontLeft);
            telemetry.addData("front right", "%.2f", frontRight);
            telemetry.addData("back left", "%.2f", backLeft);
            telemetry.addData("back right", "%.2f", backRight);

            telemetry.addData("current heading", formatAngle(currOrient.angleUnit, currOrient.firstAngle));
            telemetry.addData("target heading", formatAngle(targetOrient.angleUnit, targetOrient.firstAngle));

            telemetry.update();

            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);



        }

    }


    //rotates to the given heading
    void rotateToHeading(double heading){

        Orientation currentOrient;
        currentOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = currentOrient.angleUnit.DEGREES.normalize(currentOrient.firstAngle);

        for (double i = .3; i > .1; i = i-.1) {

            blindRotateRight(i);
            while ((currentOrient.angleUnit.DEGREES.normalize(currentOrient.firstAngle) > heading) && opModeIsActive()) {

                currentOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("current heading", formatAngle(currentOrient.angleUnit, currentOrient.firstAngle));
                telemetry.addData("target heading", heading);
                telemetry.update();
            }
            stopDriving();

            blindRotateLeft(i);
            while ((currentOrient.angleUnit.DEGREES.normalize(currentOrient.firstAngle) < heading) && opModeIsActive()) {

                currentOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("current heading", formatAngle(currentOrient.angleUnit, currentOrient.firstAngle));
                telemetry.addData("target heading", heading);
                telemetry.update();
            }
            stopDriving();
        }



        blindRotateRight(.175);
        while ((currentOrient.angleUnit.DEGREES.normalize(currentOrient.firstAngle) > heading) && opModeIsActive()) {

            currentOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading", formatAngle(currentOrient.angleUnit, currentOrient.firstAngle));
            telemetry.addData("target heading", heading);
            telemetry.update();
        }
        stopDriving();




        blindRotateLeft(.175);
        while ((currentOrient.angleUnit.DEGREES.normalize(currentOrient.firstAngle) < heading) && opModeIsActive()) {
            currentOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("current heading", formatAngle(currentOrient.angleUnit, currentOrient.firstAngle));
            telemetry.addData("target heading", heading);
            telemetry.update();
        }
        stopDriving();
    }





    //just rotates to the right
    void blindRotateRight(double pwr){
        pwr = -pwr; // -pwr on all wheels turns right
        // Set power on each wheel
        robot.frontLeftMotor.setPower(pwr);
        robot.frontRightMotor.setPower(pwr);
        robot.backLeftMotor.setPower(pwr);
        robot.backRightMotor.setPower(pwr);

    }

    //just rotates to the left
    void blindRotateLeft(double pwr){

        robot.frontLeftMotor.setPower(pwr);
        robot.frontRightMotor.setPower(pwr);
        robot.backLeftMotor.setPower(pwr);
        robot.backRightMotor.setPower(pwr);

    }
//end of Graham Addition


// Functions related to the wobble goal________________________________________________________________________________________________________

    /*void wobbleUp() {
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void wobbleDown(){
        robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    } */

    void wobbleOpen() {

        robot.wobbleServo.setPosition(1);
    }

    void wobbleClose() {
        robot.wobbleServo.setPosition(0);
    }

    void wobbleDrop() {
       // wobbleDown();
        wobbleOpen();
        telemetry.addData("Wobble goal delivered", 0);
        telemetry.update();
    }

    void wobbleRaise() {
        wobbleClose();
       // wobbleUp();
        telemetry.addData("Wobble goal up", 0);
        telemetry.update();
    }


    //Scan Function_______________________________________________________________________________________________________________

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98); //Change these to match up with where rings will be depending on phone placement

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150; //Change these thresholds depending on our values
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }




    void wobblePosition() {
        double lastTime = runtime.milliseconds();
        double noneTime = 3000;
        double oneTime = 2000;
        double fourTime = 1000;
        double noneForward = 500;
        double oneForward = 500;
        double fourForward = 500;
       /* enum currentPosition = ONE;
        currentPosition = pipeline.position;
        runtime.reset();
        if (currentPosition == pipeline.RingPosition.NONE) {
            while (lastTime < noneTime) {
                strafeRight(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            }
            runtime.reset();
            while (lastTime < noneForward) {
                driveForward();

            }
            stopDriving();
        }

        if (ONE) {
            while (lastTime < oneTime) {
                strafeRight(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            }
            runtime.reset();
            while (lastTime < oneForward) {
                driveForward();
            }
            stopDriving();
        }


        if (FOUR) {
            while (lastTime < fourTime) {
                strafeRight(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            }
            runtime.reset();
            while (lastTime < fourForward) {
                driveForward();
            }
            stopDriving();
        }*/
        while(lastTime < fourTime){
            strafeRight(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
        }
        runtime.reset();
        while(lastTime < fourForward){
            driveForward();
        }
        stopDriving();

    }

    void positionRobot() {
        //drive up to rings
        while (robot.backDistance.getDistance(DistanceUnit.MM) < FRONTDIST) {
            driveForward();
        }
        rotateToHeading(90);
    }

    void park(){
        //park over white line
    }

    void launchRing(){
        //to powershot
    }




    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
