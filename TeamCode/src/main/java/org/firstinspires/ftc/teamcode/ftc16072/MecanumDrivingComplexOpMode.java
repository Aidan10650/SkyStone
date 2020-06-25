package org.firstinspires.ftc.teamcode.ftc16072;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

@TeleOp(name = "ComplexTele", group = "ftc10650")
public class MecanumDrivingComplexOpMode extends ComplexOp {

    @Override
    public Vector2D startPosition() {
        return new Vector2D(0,0);
    }

    @Override
    public void body() throws InterruptedException {
        ComplexMove(SpeedCalcs.topSpeed(1), MotionCalcs.moveWithObjectCentricJoystick(),
                OrientationCalcs.turnWithJoystick(), OtherCalcs.TeleOpMatch(), OtherCalcs.TeleServos(OtherCalcs.Controller.DRIVER));
    }
}
