// FTC Team 7080 BACON
// Autonomous code 2021-2022


//**This code is a combination of Graham's Vuforia code and Elisabeth's auto code (testing)


package org.firstinspires.ftc.teamcode;


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
//import org.firstinspires.ftc.teamcode.HardwareBACONbot;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;

import java.util.Locale;

//Vuforia-related libraries
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


//_________________________________________________________________________________________________


@Autonomous(name = "BACON: Autonomous Test 21-22", group = "Opmode")
//@Disabled


public class BACONautoComboCode extends LinearOpMode {

    boolean duckFound = false;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AUOQWxb/////AAABmRP6L/V1T0Bclh/MquexUq8kKPD3h3N5sSIPraEvHInc1KyTB1KSLqkDd0mdJZibl8t7LsWmHogI6fR7p44UvkxD6uBvANg8xebRLgWIHaPvqxf3IqT8IG2VkljyPD/Unlfi357W5qXls0rtkFem3yX5kROTZEfRbmf5ZwtC3KSu6hBzriQwM7zk0zptP/MWtO6B/SZz6OWwLCR6O4I6TkKC7kQS3b1VGNonWq4fFL5jMcVPypqZKohDySdG4URcz0NqxpeEcC9P/c/VL67JKBcFaNBtix+7N/yccggWv8tUKuofNLIS1mUEv5kTzw9n4ps6ApmE2PziqmOjzpNL0MgF+V3KhRddiJjx51nFKEdX";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    HardwareMap2022 robot = new HardwareMap2022();

    //variables

    public void runOpMode() {

        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;


        int side = 0; // 1 = left side start 2 = right side start
        int warehouse = 1;
        int carousel = 2;

        double meetDistance = 860; //Distance from wall to the rings (CM From Wall (BackSensor))

        double lastTime = runtime.milliseconds();



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

        telemetry.addData("teamcolor ", teamcolor);
        telemetry.update();

        // Choosing side
        telemetry.addData("Press A for warehouse, Y for carousel", "");
        telemetry.update();

        while (!gamepad1.a && !gamepad1.y) {
        }
        if (gamepad1.a) {
            side = warehouse;
        }
        if (gamepad1.y) {
            side = carousel;
        }
        telemetry.addData("side ", side);
        telemetry.update();

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) robot.backDistance;


        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            //tfod.setZoom(2, 16.0 / 9.0);

        }



        ///_____________________________________________________________________________________________________
        waitForStart();
        runtime.reset();


        double WSrestPos1 = 0;

        double placeHeight = getPlaceHeight(); //moves robot to phase 2

        telemetry.addData("Place Height: ", placeHeight);
        telemetry.update();
        sleep(5000);

        // Don't burn CPU cycles busy-looping in this sample
        //sleep(50);

        // run until the end of the match (when driver presses STOP)

        if (teamcolor == red && side == warehouse) {
            //sense (includes driving forward a little)
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 300);
            robot.rotateToHeading(0.25,0);
            //drop item (including half back)
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            robot.rotateToHeading(0.25,0);
            robot.spinCarouselServo();
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            //park
        }
        if (teamcolor == red && side == carousel) {
            //sense (includes driving forward a little)
            robot.strafeLeftUsingLeftDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            robot.spinCarouselServo();
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
            robot.rotateToHeading(0.25,0);
            //drop item (half back?)
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 60);
            //park

        }
        if (teamcolor == blue && side == warehouse) {
            //sense (includes driving forward a little)
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),50);
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 700);
            robot.rotateToHeading(.25,0);
            //drop item (including coming back to start (halfback))
            robot.rotateToHeading(.25,-90);
            robot.driveForwardUseFrontDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),50);
            robot.rotateToHeading(0.25,0);
            robot.spinCarouselServo();
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseFrontDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),80);
            //park
        }
        if (teamcolor == blue && side == carousel) {
            //sense (includes driving forward a litle)
            robot.strafeRightUsingRightDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 260);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 200);
            robot.spinCarouselServo();
            robot.strafeLeftUsingRightDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 225);
            robot.rotateToHeading(0.25, 90);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 600);
            robot.rotateToHeading(0.25, 0);
            //drop item
            robot.rotateToHeading(0.25, 90);
            robot.driveForwardUseFrontDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 550);
            //park

        }
        //TODO Flip all of the heading things (counterclockwise is pos)
        //TODO drive 60mm and then rotate ~15deg (getduckposition --> 1,2,3)




        /*public void verticalSlide (duckPlace){
            if duckPlace == {
                linearUp.setPower(-1);
            while ((linearUp.getCurrentPosition() > -1500) && opModeIsActive()) {
                telemetry.addData("verticalSlide pos: ", linearUp.getCurrentPosition());
                telemetry.update();
            }
            linearUp.setPower(0.0);
            }
            if duckPlace == {

            }
            if duckPlace == {

            } */
    }
    //Functions__________________________________________________________________________________________________________________
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.minResultConfidence = 0.8f; //original
        tfodParameters.minResultConfidence = 0.30f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public boolean getDuckLocation() {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel().equalsIgnoreCase("Duck")){

                        //telemetry.addData("LP", recognition.getLeft());

                        //float leftpixel;
                        //leftpixel = recognition.getLeft();
                        //telemetry.addData("LP",leftpixel);


                        duckFound = true;
                        return duckFound;
                    }
                }
                telemetry.update();
            }

        }
        return duckFound;
    }


    public float getPlaceHeight(){
        float funcPlaceHeight = 0;

        double startTime = runtime.milliseconds();


        robot.driveForwardUseBackwardDistance(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
        robot.rotateToHeading(.5,15);

        if(getDuckLocation() == true){ // if middle confirmed
            funcPlaceHeight = 2;
            robot.rotateToHeading(.5,-90);
        }
        else{ // else test right
            robot.rotateToHeading(.5,-15);
            if(getDuckLocation() == true){ //if right confirmed
                funcPlaceHeight = 3;
                robot.rotateToHeading(.5,-90);
            }
            else{  //means it is left
                funcPlaceHeight = 1;
                robot.rotateToHeading(.5,-90);
            }

        }

        telemetry.addData("Duck Detected in Place: ", funcPlaceHeight);
        telemetry.update();
        return funcPlaceHeight;
    }
}

