package org.firstinspires.ftc.teamcode.Hardware.SkystoneRobotNameBox;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.NavX;

public class RobotMap {
    public static DcMotor bright, fright, bleft, fleft, intLeft, intRight, lift, rotator;
    public static Servo gripper, swinger, hooker, booker;
    public static NavX gyro;
    public static WebcamName stoneCam;
    public static ModernRoboticsI2cRangeSensor frontRange, backRange;

    public RobotMap(HardwareMap hw) {

        bright  = hw.get(DcMotor.class, "bright");
        //RUN_USING_ENCODER gives each motor a PID and ensures the motors run at the same speed every time.
        bright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);

        fright  = hw.get(DcMotor.class, "fright");
        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fright.setDirection(DcMotorSimple.Direction.REVERSE);

        bleft   = hw.get(DcMotor.class, "bleft");
        bleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fleft   = hw.get(DcMotor.class, "fleft");
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        lift    = hw.get(DcMotor.class, "lift");
//        rotator = hw.get(DcMotor.class, "rotator");


//        intLeft = hw.get(DcMotor.class, "intLeft");
//        intRight = hw.get(DcMotor.class, "intRight");

        intRight.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro = new NavX(hw, "navX", 0);

        stoneCam = hw.get(WebcamName.class, "stoned cam");

        gripper = hw.get(Servo.class, "firm grasp");
        swinger = hw.get(Servo.class, "ragtime");
        hooker = hw.get(Servo.class, "hooker");
        booker = hw.get(Servo.class,"booker");

        frontRange = hw.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        backRange = hw.get(ModernRoboticsI2cRangeSensor.class, "backRange");
    }
}