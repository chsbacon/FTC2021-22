package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardware_map {
    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public DcMotor deadWheel = null;

    private HardwareMap HWMap = null;
    private ElapsedTime period = new ElapsedTime();

    public hardware_map() {
    }

    public void init(HardwareMap ahwMap){
        HWMap = ahwMap;
        motor1 = HWMap.dcMotor.get("M1");
        motor2 = HWMap.dcMotor.get("M2");
        motor3 = HWMap.dcMotor.get("M3");
        motor4 = HWMap.dcMotor.get("M4");
        deadWheel = HWMap.dcMotor.get("DWE0");
    }
}
