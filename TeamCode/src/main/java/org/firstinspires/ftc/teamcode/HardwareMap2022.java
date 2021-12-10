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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;


public class HardwareMap2022
{
    /* Public OpMode members. */
    public DcMotor  frontLeftMotor   = null;
    public DcMotor  frontRightMotor = null;
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;

    public DcMotor  spinTakeMotor = null;

    public DcMotor  leftLinearSlideMotor = null;
    public DcMotor  rightLinearSlideMotor = null;
    
    public DcMotor  liftMotor = null;
    
    public CRServo carouselServo = null;

    public CRServo intakeServo1 = null;
    public CRServo intakeServo2 = null;
    public Servo dropServo = null;

    public DistanceSensor frontDistance = null;
    public DistanceSensor rightDistance = null;
    public DistanceSensor backDistance = null;
    public DistanceSensor leftDistance = null;

    public BNO055IMU imu;

    boolean spintakeToggle = true;
    boolean intakeToggle1 = true;
    boolean intakeToggle2 = true;
    
    int liftMotorTicks = 0;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public HardwareMap2022(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "FLM"); //P0
        frontRightMotor  = hwMap.get(DcMotor.class, "FRM"); //P1
        backLeftMotor = hwMap.get(DcMotor.class, "BLM"); //P2
        backRightMotor = hwMap.get(DcMotor.class, "BRM"); //P3

        spinTakeMotor = hwMap.get(DcMotor.class, "ST"); //H2 P3
        
        liftMotor = hwMap.get(DcMotor.class,"LM"); //H2P0
        
        leftLinearSlideMotor = hwMap.get(DcMotor.class,"LLSM"); //H2P1
        rightLinearSlideMotor = hwMap.get(DcMotor.class, "RLSM"); //H2P2
        
        carouselServo = hwMap.get(CRServo.class,"CS"); //H2ServoP1
        intakeServo1 = hwMap.get(CRServo.class, "IS1"); //H2ServoP2
        intakeServo2 = hwMap.get(CRServo.class, "IS2"); // H2ServoP3
        dropServo = hwMap.get(Servo.class, "DS"); //H1ServoP2

        frontDistance = hwMap.get(DistanceSensor.class,"FDS"); //H1P0
        rightDistance = hwMap.get(DistanceSensor.class,"RDS"); //H1P1
        backDistance = hwMap.get(DistanceSensor.class,"BDS"); //H1P2
        leftDistance = hwMap.get(DistanceSensor.class,"LDS"); //H1P3

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        spinTakeMotor.setPower(0);
        
        leftLinearSlideMotor.setPower(0);
        rightLinearSlideMotor.setPower(0);
        liftMotor.setPower(0);


        carouselServo.setPower(0);
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);
        dropServo.setPosition(0.7);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spinTakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftLinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        leftLinearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }

    public void stopDriving(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void rotateToHeading(double pwr, double target){

        // set to a big number so it doesn't accidentally match the target angle
        //therefore hypothetically completing the while-loop accidentally
        double currAng = 10000;

        Orientation currOrient;

        double integralSum = 0;
        double lastError = 0;
        double error;
        double derivative;
        double out;

        double kP = .04;
        double kI = .0;
        double kD = .99;

        ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime cutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while( (currAng != target) /*&& opModeIsActive()*/){

            pidTimer.reset();

            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            error = target - currAng;
            derivative = (error - lastError) / pidTimer.milliseconds();
            integralSum = integralSum + (error * pidTimer.time());

            if(integralSum > 2000){
                integralSum = 2000;
            }
            if(integralSum < -2000){
                integralSum = -2000;
            }

            out = (kP * error) + (kI * integralSum) + (kD * derivative);

            //telemetry.addData("target: ", "%.2f", target);
            //telemetry.addData("current: ", "%.2f", currAng);
            //telemetry.addData("out: ", "%.2f", out);
            //telemetry.update();

            frontLeftMotor.setPower(pwr + out);
            frontRightMotor.setPower(pwr + out);
            backLeftMotor.setPower(pwr + out);
            backRightMotor.setPower(pwr + out);

            lastError = error;

            if (cutTimer.milliseconds() > 2000){
                break;
            }
        }
        stopDriving();
        //telemetry.addData("target: ", "%.2f", target);
        //telemetry.addData("current: ", "%.2f", currAng);
        //telemetry.update();
    }

    public void driveForwardUseTime(double pwr, Orientation target, double desiredTime){

        //orients
        Orientation targetOrient;
        Orientation currOrient;


        double lastTime = runtime.milliseconds();

        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((runtime.milliseconds() < lastTime + desiredTime) /*&& (opModeIsActive())*/)){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);



        }
        stopDriving();
    }

    public void driveBackwardUseTime(double pwr, Orientation target, double desiredTime){

        //orients
        Orientation targetOrient;
        Orientation currOrient;


        double lastTime = runtime.milliseconds();

        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((runtime.milliseconds() < lastTime + desiredTime) /*&& (opModeIsActive())*/)){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;

            //drive straight's is :::: double r = (-error / 180) / (pwr);
            double r = (error / 180) / (pwr);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(-frontLeft);
            backLeftMotor.setPower(-backLeft);
            backRightMotor.setPower(-backRight);
            frontRightMotor.setPower(-frontRight);



        }
        stopDriving();
    }

    public void driveForwardUseFrontDistance(double pwr, Orientation target, double desiredDistanceMM){

        //orients
        Orientation targetOrient;
        Orientation currOrient;



        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((frontDistance.getDistance(DistanceUnit.MM) > desiredDistanceMM))){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);
            //telemetry.addData("desired Distance", desiredDistanceCM);
            //telemetry.addData("current Distance", frontDistance.getDistance(DistanceUnit.CM));

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
    }

    public void driveForwardUseBackwardDistance(double pwr, Orientation target, double desiredDistanceMM){

        //orients
        Orientation targetOrient;
        Orientation currOrient;



        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((backDistance.getDistance(DistanceUnit.MM) < desiredDistanceMM))){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);
            //telemetry.addData("desired Distance", desiredDistanceCM);
            //telemetry.addData("current Distance", frontDistance.getDistance(DistanceUnit.CM));

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
    }

    public void driveBackwardUseFrontDistance(double pwr, Orientation target, double desiredDistanceMM){

        //orients
        Orientation targetOrient;
        Orientation currOrient;



        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((frontDistance.getDistance(DistanceUnit.MM) < desiredDistanceMM))){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (error / 180) / (pwr);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);
            //telemetry.addData("desired Distance", desiredDistanceCM);
            //telemetry.addData("current Distance", frontDistance.getDistance(DistanceUnit.CM));

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(-frontLeft);
            backLeftMotor.setPower(-backLeft);
            backRightMotor.setPower(-backRight);
            frontRightMotor.setPower(-frontRight);
        }
        stopDriving();
    }

    public void driveBackwardUseBackDistance(double pwr, Orientation target, double desiredDistanceMM){

        //orients
        Orientation targetOrient;
        Orientation currOrient;



        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);

        //rChanger changes the sensitivity of the R value
        //double rChanger = 10;
        double frontLeft, frontRight, backLeft, backRight, max;

        while(((backDistance.getDistance(DistanceUnit.MM) > desiredDistanceMM))){



            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);

            double error = targAng - currAng;


            double r = (error / 180) / (pwr);
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



            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);

            //telemetry.addData("current heading", currAng);
            //telemetry.addData("target heading", targAng);
            //telemetry.addData("desired Distance", desiredDistanceCM);
            //telemetry.addData("current Distance", frontDistance.getDistance(DistanceUnit.CM));

            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(-frontLeft);
            backLeftMotor.setPower(-backLeft);
            backRightMotor.setPower(-backRight);
            frontRightMotor.setPower(-frontRight);
        }
        stopDriving();
    }

    public void strafeLeft(double pwr, Orientation target, double desiredTime) {
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while((((runtime.milliseconds() < lastTime + desiredTime)))){

            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
    }

    public void strafeRight(double pwr, Orientation target, double desiredTime) {
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while((((runtime.milliseconds() < lastTime + desiredTime)))){

            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
                r = -.07;
            }

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = -pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr + r ;
            frontRight = -pwr + r ;

            //original (strafe left)
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
    }


    public void strafeLeftUsingLeftDistance (double pwr, Orientation target, double desiredDistanceMM){
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while (((leftDistance.getDistance(DistanceUnit.MM) > desiredDistanceMM))){
            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);

        }
        stopDriving();

    }

    public void strafeRightUsingRightDistance (double pwr, Orientation target, double desiredDistanceMM){
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while (((rightDistance.getDistance(DistanceUnit.MM)>desiredDistanceMM))){
            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
                r = -.07;
            }

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = -pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr + r ;
            frontRight = -pwr + r ;

            //original (strafe left)
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
    }

    public void strafeLeftUsingRightDistance (double pwr, Orientation target, double desiredDistanceMM){
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while (((rightDistance.getDistance(DistanceUnit.MM) > desiredDistanceMM))){
            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);

        }
        stopDriving();
    }

    public void strafeRightUsingLeftDistance (double pwr, Orientation target, double desiredDistanceMM){
        //orients
        Orientation targetOrient;
        Orientation currOrient;


        //converts the target heading to a double to use in error calculation
        targetOrient = target;
        double targAng = targetOrient.angleUnit.DEGREES.normalize(target.firstAngle);;  // target.angleUnit.DEGREES.normalize(target.firstAngle);


        //rChanger changes the sensitivity of the R value
        double rChanger = 5;
        double frontLeft, frontRight, backLeft, backRight, max;

        double lastTime = runtime.milliseconds();

        while (((leftDistance.getDistance(DistanceUnit.MM)>desiredDistanceMM))){
            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double currAng = currOrient.angleUnit.DEGREES.normalize(currOrient.firstAngle);
            double error = targAng - currAng;

            double r = (-error / 180) / (pwr) ;

            //double r = (-error / 180) / (pwr) ;
            //double r = (-error / 180)  / (rChanger * pwr);
            //double r = (-error/180);
            //r = 0;
            //r=-r;


            if (error > 0){
                r = r;
            }
            if (error < 0){
                r = -r;
            }



            if ((r > .07) && (r > 0)) {
                r = .07;
            } else if ((r < -.07) && (r < 0)) {
                r = -.07;
            }

            // Normalize the values so none exceeds +/- 1.0
            frontLeft = -pwr + r ;
            backLeft = pwr + r ;
            backRight = pwr + r ;
            frontRight = -pwr + r ;

            //original (strafe left)
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

            //telemetry.addData("front left", "%.2f", frontLeft);
            //telemetry.addData("front right", "%.2f", frontRight);
            //telemetry.addData("back left", "%.2f", backLeft);
            //telemetry.addData("back right", "%.2f", backRight);
            //telemetry.addData("error", error);
            //telemetry.addData("currOrient", currOrient);
            //telemetry.addData("r",r);
            //telemetry.addData("targetOrient", targetOrient);

            //telemetry.update();
            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);
        }
        stopDriving();
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

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while(Math.abs(backLeftMotor.getCurrentPosition()) < Math.abs(desiredTicks)){

            currOrient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

            //telemetry.addData("Target: ", desiredTicks);
            //telemetry.addData("tickPos: ",robot.backLeftMotor.getCurrentPosition());
            //telemetry.update();

            //send the power to the motors
            frontLeftMotor.setPower(frontLeft);
            backLeftMotor.setPower(backLeft);
            backRightMotor.setPower(backRight);
            frontRightMotor.setPower(frontRight);



        }
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopDriving();

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    void spinCarouselServo(){
        ElapsedTime  carouselRuntime = new ElapsedTime();

        carouselServo.setPower(-1);
        while(carouselRuntime.milliseconds() < 4250){

        }
        carouselServo.setPower(0);

    }

    /*
    public void spintake () {
        if(spintakeToggle == true){
        spinTakeMotor.setPower(.75);
        spintakeToggle = false;
        }
        if(spintakeToggle == false) {
        spinTakeMotor.setPower(0);
        spintakeToggle = true;
        }
    }
  */
      public void autoDrop(double placeHeight){

        if(placeHeight == 1){
            lowerIntake();
            dropItem();
            liftMotorTicks = -1;
            moveLiftMotor(-1, .5);
            raiseIntake();

        }
        if(placeHeight==2){

            lowerIntake();
            liftMotorTicks = -1200;
            moveLiftMotor(-1200, .5);
            dropItem();
            liftMotorTicks = -1;
            moveLiftMotor(-1, .5);
            raiseIntake();

        }
        if(placeHeight==3){
            lowerIntake();
            liftMotorTicks = -2200;
            moveLiftMotor(-2200, .5);
            dropItem();
            liftMotorTicks = -1;
            moveLiftMotor(-1, .5);
            raiseIntake();
        }
    }

    public void dropItem(){
        dropServo.setPosition(.35);
        ElapsedTime  dropTime = new ElapsedTime();
        while(dropTime.milliseconds() < 1500){

        }
        dropServo.setPosition(0);

    } //drop toggle?


    public void lowerIntake(){
        ElapsedTime  lowerTime = new ElapsedTime();

        intakeServo1.setPower(-1);
        intakeServo2.setPower(1);

        while(lowerTime.milliseconds() < 1000){

        }

        intakeServo1.setPower(0);
        intakeServo2.setPower(0);

    }

    public void raiseIntake(){
        ElapsedTime  raiseTime = new ElapsedTime();

        intakeServo1.setPower(1);
        intakeServo2.setPower(-1);

        while(raiseTime.milliseconds() < 2000){

        }

        intakeServo1.setPower(0);
        intakeServo2.setPower(0);

    }

    public void moveLiftMotor(int myTicks, double positivePWR){
        liftMotor.setTargetPosition(myTicks);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(positivePWR);

        while(liftMotor.isBusy()){

        }
        liftMotor.setPower(0);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



}
