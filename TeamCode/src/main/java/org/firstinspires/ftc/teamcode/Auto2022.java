package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMap2022;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name="Auto22", group="Opmode")
//@Disabled
public class Auto2022 extends LinearOpMode {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 1920; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution

    double CrLowerUpdate = 40;
    double CbLowerUpdate = 160;
    double CrUpperUpdate = 255;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    double placeHeight = 0;
    double placeHeightAdjust = 0;

    /* Declare OpMode members. */
    HardwareMap2022 robot = new HardwareMap2022();




    private ElapsedTime     runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);



        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });






        int teamcolor = 0; // 1 = Blue 2 = Red
        int blue = 1;
        int red = 2;

        int side = 0; // 1 = left side start 2 = right side start
        int warehouse = 1;
        int carousel = 2;

        double carouselColor = 0;

        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;


        // Choosing the team color
        telemetry.addData("Press X for Blue, B for Red", "");
        telemetry.update();

        while (!gamepad1.x && !gamepad1.b) {
        }

        if(gamepad1.x){
            teamcolor = blue;
            carouselColor = -.75;
        }
        if(gamepad1.b){
            teamcolor = red;
            carouselColor = .75;
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


        while (!isStarted())
        {

            if(myPipeline.error){
                //telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            testing(myPipeline);

            // Watch our YouTube Tutorial for the better explanation

            //telemetry.addData("RectMidpoint: ", myPipeline.getRectMidpointX());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 1300){
                    placeHeight = 3;
                    telemetry.addData("placeHeight: ", placeHeight);
                    telemetry.update();
                    placeHeightAdjust = 75;
                }
                else if(myPipeline.getRectMidpointX() > 600){
                    placeHeight = 2;
                    telemetry.addData("placeHeight: ", placeHeight);
                    telemetry.update();
                    placeHeightAdjust = 90;
                }
                else {
                    placeHeight = 1;
                    telemetry.addData("placeHeight: ", placeHeight);
                    telemetry.update();
                    placeHeightAdjust = 105;
                }
            }
        }






        waitForStart();

        if ((teamcolor == blue) && (side == warehouse)) {

            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            robot.blinkinLedDriver.setPattern(pattern);

            /*
            //PLACE (works but slow)
            robot.driveForwardUseEncoder(.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 125);
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
            robot.rotateToHeading(0,90);
            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
            robot.rotateToHeading(0,160);
            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),350);
            //place on correct level
            */


            //GET TO PLACE
            telemetry.addData("placeHeight: ", placeHeight);
            telemetry.update();
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),75); //was 75 pre ziptie
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            robot.rotateToHeading(0,150, 1250); //1250 safe //150 degrees clears obstacle //was 155 degrees
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500 + placeHeightAdjust); //was 550 ticks

            //PLACE
            robot.lowerIntake();
            if(placeHeight == 3){
                robot.moveLiftMotor(-3000,.25);
            }
            else if(placeHeight == 2){
                robot.moveLiftMotor(-1000,.25);
            }
            else if (placeHeight == 1){
                robot.moveLiftMotor(0,.25);
            }
            else{
            }
            robot.dump();
            robot.moveLiftMotor(0,.25);
            robot.raiseIntake();


            //GET TO WAREHOUSE (place to warehouse)
            robot.driveForwardUseEncoder(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 400);
            robot.rotateToHeading(0, 90, 1000); //1250 safe
            robot.strafeLeft(1, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000);
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1250);



            /*
            //WAREHOUSE STUFF
            robot.rotateToHeading(0,85,250);
            robot.lowerIntake();
            robot.spintakeIn();
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
            robot.spintakeStop();
            robot.raiseIntake();
            robot.rotateToHeading(0,90,250);
            robot.strafeLeft(1,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
            robot.driveBackwardUseAlpha(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),150,1500);



            //PLACE (2nd) (warehouse to place)
            robot.strafeRight(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),100); //to clear off wall
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);
            robot.strafeRight(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);
            robot.rotateToHeading(0,150,1250); //1500 safe
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            //place block


            //GET TO WAREHOUSE (2nd)
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),400);
            robot.rotateToHeading(0, 90,1000);  //1250 safe
            robot.strafeLeft(1, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000); //1250 safe
            robot.driveForwardUseEncoder(.75, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000);
            */




        }

        if((teamcolor == blue) && (side == carousel)){

            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            robot.blinkinLedDriver.setPattern(pattern);

            //GET TO PLACE
            telemetry.addData("placeHeight: ", placeHeight);
            telemetry.update();
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),75); //was 75 pre ziptie
            robot.strafeLeft(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            robot.rotateToHeading(0,-150, 1250); //1250 safe //150 degrees clears obstacle //was 155 degrees
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),650 + placeHeightAdjust); //was 625



            //PLACE
            robot.lowerIntake();
            if(placeHeight == 3){
                robot.moveLiftMotor(-3000,.25);
            }
            else if(placeHeight == 2){
                robot.moveLiftMotor(-1000,.25);
            }
            else if (placeHeight == 1){
                robot.moveLiftMotor(0,.25);
            }
            else{
            }
            robot.dump();
            robot.moveLiftMotor(0,.25);
            robot.raiseIntake();

            robot.driveForwardUseEncoder(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 500);
            robot.rotateToHeading(0,-90,1500);
            robot.strafeRight(.75,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1000);
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);

            robot.strafeLeft(.75,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),250);
            robot.driveForwardUseTime(.2,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1250);
            robot.spinCarouselMotor(carouselColor);

            robot.driveBackwardUseEncoder(.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),600);
            robot.rotateToHeading(0,-30,1000);
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),1000);

        }












        if((teamcolor == red) && (side == warehouse)){
            //GET TO PLACE
            telemetry.addData("placeHeight: ", placeHeight);
            telemetry.update();
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),75); //was 75 pre ziptie
            robot.strafeLeft(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500);
            robot.rotateToHeading(0,-155, 1250); //1250 safe //150 degrees clears obstacle
            robot.driveBackwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),500 + placeHeightAdjust);

            //PLACE
            robot.lowerIntake();
            if(placeHeight == 3){
                robot.moveLiftMotor(600,.25);
            }
            else if(placeHeight == 2){
                robot.moveLiftMotor(300,.25);
            }
            else if (placeHeight == 1){
                robot.moveLiftMotor(0,.25);
            }
            else{
            }
            //dump -- add later
            robot.moveLiftMotor(0,.25);
            robot.raiseIntake();


            //GET TO WAREHOUSE (place to warehouse)
            robot.driveForwardUseEncoder(.5, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 400);
            robot.rotateToHeading(0, -90, 1000); //1250 safe
            robot.strafeRight(1, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1000);
            robot.driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),750);

        }


        if((teamcolor == red) && (side == carousel)){
        }





    }

    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.ConfigureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.ConfigureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        //telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        //telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        //telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        //telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
}
