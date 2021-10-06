package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.Locale;
@TeleOp(name="RotatePIDtesting", group="Linear Opmode")
public class RotatePIDtesting extends LinearOpMode {


    HardwareMap2022 robot = new HardwareMap2022();

    double pwr = .5;
    double target;
    double kP = 0;
    double kI = 0;
    double kD = 0;

    public void runOpMode(){

        robot.init(hardwareMap);


        if(gamepad1.a){
            rotateToHeadingV1(.5, 90);
        }

        if(gamepad1.x){
            kP = kP + .01;
        }

        if(gamepad1.y){
            kI = kI + .01;
        }

        if(gamepad1.b){
            kD = kD + .01;
        }




    }


    void rotateToHeadingV1(double pwr, double target){



        // set to a big number so it doesn't accidently match the target angle
        //therefore hypothetically completing the while-loop accidently
        double currAng = 10000;

        Orientation targetOrient;
        Orientation currOrient;

        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;

        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



        while( currAng != target ){

            pidTimer.reset();

            currOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            error = target - currAng;

            derivative = (error - lastError) / pidTimer.milliseconds();

            integralSum = integralSum + (error * pidTimer.time());

            out = (kP * error) + (kI * integralSum) + (kD * derivative);
            //Still have to see if out is -1 < out < 1

            robot.frontLeftMotor.setPower(pwr + out);
            robot.frontRightMotor.setPower(pwr - out);
            robot.backLeftMotor.setPower(pwr + out);
            robot.backRightMotor.setPower(pwr - out);

            lastError = error;

        }
    }



}
