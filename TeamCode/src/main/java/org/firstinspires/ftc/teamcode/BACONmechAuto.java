// FTC Team 7080 BACON
// Autonomous code 2021-2022

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

//_________________________________________________________________________________________________


@Autonomous(name = "BACON: Autonomous 21-22", group = "Opmode")
//@Disabled


public class BACONmechAuto extends LinearOpMode {

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


        waitForStart();
        runtime.reset();


        double WSrestPos1 = 0;

        // Don't burn CPU cycles busy-looping in this sample
        //sleep(50);

        // run until the end of the match (when driver presses STOP)

        if (teamcolor == red && side == warehouse) {
            robot.sensingSetup(); //sensingSetup drives forward 60mm and then rotates 15deg to be in place to sense for the duck
            //sense (includes driving forward a little)
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 300);
            robot.rotateToHeading(0.25,0);
            //drop item (including half back)
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            robot.rotateToHeading(0.25,0);
            robot.spinCarouselMotor();
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            //park
        }
        if (teamcolor == red && side == carousel) {
            robot.sensingSetup();
            //sense (includes driving forward a little)
            robot.strafeLeftUsingLeftDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),60);
            robot.spinCarouselMotor();
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),300);
            robot.rotateToHeading(0.25,0);
            //drop item (half back?)
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseFrontDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 60);
            //park

        }
        if (teamcolor == blue && side == warehouse) {
            robot.sensingSetup();
            //sense (includes driving forward a little)
            robot.driveForwardUseBackwardDistance(0.25,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),50);
            robot.rotateToHeading(0.25,-90);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 700);
            robot.rotateToHeading(.25,0);
            //drop item (including coming back to start (halfback))
            robot.rotateToHeading(.25,-90);
            robot.driveForwardUseFrontDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),50);
            robot.rotateToHeading(0.25,0);
            robot.spinCarouselMotor();
            robot.rotateToHeading(0.25,90);
            robot.driveForwardUseFrontDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES),80);
            //park
        }
        if (teamcolor == blue && side == carousel) {
            robot.sensingSetup();
            //sense (includes driving forward a litle)
            robot.strafeRightUsingRightDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 260);
            robot.driveForwardUseBackwardDistance(0.25, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 200);
            robot.spinCarouselMotor();
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
}

