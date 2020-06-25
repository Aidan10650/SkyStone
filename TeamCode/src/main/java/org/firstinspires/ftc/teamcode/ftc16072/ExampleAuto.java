package org.firstinspires.ftc.teamcode.ftc16072;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ExampleAuto", group = "ftc10650")
public class ExampleAuto extends ComplexOp {

    @Override
    public Vector2D startPosition() {
        return new Vector2D(0.0,0.0);
    }

    @Override
    public void body() throws InterruptedException {

        ComplexMove(
                SpeedCalcs.setSpeed(0.7),
                MotionCalcs.CurveMotion(200,0,90),
                OrientationCalcs.turnWithJoystick(),
                OtherCalcs.DistanceStop(OtherCalcs.Side.FRONT,25,5,0.95,1));

        ComplexMove(
                SpeedCalcs.setSpeed(0.3),
                MotionCalcs.CurveMotion(200,180,270),
                OrientationCalcs.turnWithJoystick(),
                OtherCalcs.DistanceStop(OtherCalcs.Side.FRONT,25,5,0.95,1));
    }
}
