package org.firstinspires.ftc.teamcode;

//these are all your imports
//they will auto generate when write something that needs and import
// don't worry too much about these, they just look scary
// you'll never actually touch them much
// normally you just collapse them visually when you're editing code anyways

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 * This class defines all the specific hardware for a the BACONbot robot.
 */

// class name here should match file name
public class ExampleHardwareMap {
    /* Public OpMode members. */
    // this means we will be able to reference these in the teleop and autonomous files



    // this is where you put all of the Motors and Servos and what not
    // where we create them
    // you can delete these below

    //motor example -- you can delete this
    public DcMotor  frontLeftMotor   = null;

    //Distance Sensor example -- you can delete this
    public DistanceSensor backDistance = null;

    //servo example -- you can delete this
    public  Servo wobbleServo = null;

    //imu example -- you can delete this
    public BNO055IMU imu;




    /* local OpMode members. */
    // stuff only referenced in the hardware map
    // just part of your set up
    // keep these
    private HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();



    /* Constructor */
    //pretty much ignore this -- idek what this does tbh
    // but just make sure it matches the class and file name
    // keep this
    public ExampleHardwareMap() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        // Save reference to Hardware map
        // just leave this as the same thing across your hardware map
        // keep this
        hwMap = ahwMap;



        // these 3 lines below are all how you link the hardware to the code
        // Essentially, the short two/three letter identifiers are used for the configuration
        // more detailed description is on the random help doc on the team drive in the code guides folder


        //Motor Example -- you can delete this
        frontLeftMotor  = hwMap.dcMotor.get("FL"); // H1 0 - motor port

        //general Motor example -- you can delete this
        // MotorNameFromAbove = hwMap.dcMotor.get("Easy name to type into configuration"); // Hub Number, Port Number


        //Servo Examples -- you can delete this
        wobbleServo = hwMap.servo.get("WS"); //H2 P0  ub 2 Port 0

        //Distance Sensor Example -- you can delete this
        backDistance = hwMap.get(DistanceSensor.class, "bsr"); // H2  P2







        // -- you can delete this
        //How to set motor power to zero
        // useful to do this in the hardware map to make sure things go smoothly
        frontLeftMotor.setPower(0);


        //  -- you can delete this
        //How to flip the direction of the motor permanently
        // you can do this with negative power, but if you want to flip overall, use this line
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //  -- you can delete this
        //Setup for motors without encoders
        // encoders basically track the amount of times the wheel has rotated
        // useful for things like a tank drive or motors used to wind something up
        // otherwise for mecanum wheel drive, you probably want your motors to run without encoders
        // this is how you make a motor run without encoder
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //  -- you can delete this
        //How to make the motors turn off and not budge when not given power
        // When the motor is not given power, have it brake
        // this is surprisingly helpful, make sure to use this
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





    }

}
