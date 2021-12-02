package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.Locale;
@TeleOp(name="Lift Motor Encoder Testing", group="Linear Opmode")
public class driveStraightEncoderTesting extends LinearOpMode {


    HardwareMap2022 robot = new HardwareMap2022();

    public void runOpMode(){

        robot.init(hardwareMap);



        waitForStart();



        while(opModeIsActive()){



        if(gamepad1.a){
            driveForwardUseEncoder(.5,robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 1200);
        }








        }
    }




    public void driveForwardUseEncoder(double positivePWR, Orientation target, double desiredTicks){

        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(robot.backLeftMotor.getCurrentPosition() < desiredTicks){

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (-error / 180) / (positivePWR);
            //r = 0;

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = positivePWR + r ;
            backLeft = positivePWR + r ;
            backRight = positivePWR - r ;
            frontRight = positivePWR - r ;

            frontLeft = -frontLeft;
            backLeft = -backLeft;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(frontRight), Math.abs(frontRight)));
            if (max > 1.0) {
                frontLeft = frontLeft / max;
                frontRight = frontRight / max;
                backLeft = backLeft / max;
                backRight = backRight / max;
            }



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);

            telemetry.addData("Target: ", desiredTicks);
            telemetry.addData("tickPos: ",robot.backLeftMotor.getCurrentPosition());
            telemetry.update();

            //send the power to the motors
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.backRightMotor.setPower(backRight);
            robot.frontRightMotor.setPower(frontRight);



        }
        robot.stopDriving();
    }


}
