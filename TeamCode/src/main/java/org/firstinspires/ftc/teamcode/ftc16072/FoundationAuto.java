package org.firstinspires.ftc.teamcode.ftc16072;

import org.firstinspires.ftc.teamcode.Calculators.*;
import org.firstinspires.ftc.teamcode.Op.ComplexOp;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FoundationAuto", group = "ftc10650")
public class FoundationAuto extends ComplexOp {

    @Override
    public Vector2D startPosition() {
        return new Vector2D(0,0);
    }

    @Override
    public void body() throws InterruptedException {

        ComplexMove(
                SpeedCalcs.setProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(0.75,10000, SpeedCalcs.ProgressSpeed.timeOrProg.MILLIS),
                        new SpeedCalcs.ProgressSpeed(1,0.5, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.5,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
                MotionCalcs.PointMotion(5,
                        new Vector2D(10, 130),
                        new Vector2D(-10, 70),
                        new Vector2D(50, 10),
                        new Vector2D(0, -150),
                        new Vector2D(0, 0)
                        /*,
                        new Vector2D(0, 0)*/),
                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.0, 0.1, 90),
                        new OrientationCalcs.spinProgress(0.1, 0.2, 0))
                /*OtherCalcs.DistanceStop(OtherCalcs.Side.LEFT,150,145,0.95,1)*/);

        ComplexMove(
                SpeedCalcs.setProgressSpeed(
                        new SpeedCalcs.ProgressSpeed(1,0.2, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.1,0.3, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(1,0.4, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.75,0.95, SpeedCalcs.ProgressSpeed.timeOrProg.PROG),
                        new SpeedCalcs.ProgressSpeed(0.1,1, SpeedCalcs.ProgressSpeed.timeOrProg.PROG)),
                MotionCalcs.PointMotion(5,
                        new Vector2D(-150, 150),
                        new Vector2D(-300, 0),
                        new Vector2D(-150, -150),
                        new Vector2D(0, 0)),
                OrientationCalcs.spinToProgress(
                        new OrientationCalcs.spinProgress(0.1, 0.2, 180),
                        new OrientationCalcs.spinProgress(0.75, 0.85, -90)),
                OtherCalcs.DistanceStop(OtherCalcs.Side.FRONT,15,10,0.95,1));

 
    }
}
