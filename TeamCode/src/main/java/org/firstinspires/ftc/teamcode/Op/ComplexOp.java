package org.firstinspires.ftc.teamcode.Op;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneRobotNameBox.RobotMap;
import org.firstinspires.ftc.teamcode.Utilities.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Calculators.Interfaces;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.CompleteController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

public abstract class ComplexOp extends LinearOpMode{

    private MecanumDrive mecanumDrive;

    public void ComplexMove(Interfaces.SpeedCalc speedCalc,
                            Interfaces.MotionCalc motionCalc,
                            Interfaces.OrientationCalc orientationCalc,
                            Interfaces.OtherCalc... otherCalc) throws InterruptedException{

        float endGameTime = 0;
        d.progress = 0;
        Vector2D vector = new Vector2D();
        d.lastCommand = d.currentCommand;
        d.currentCommand = new Interfaces.MoveData.Command(0, vector,0.0);


        while(d.progress < 1.0) {
            d.heading = d.robot.gyro.getYaw();

            double encoderPreY = d.encoderPos.y;
            double encoderPreX = d.encoderPos.x;
            d.encoderPos.y = mecanumDrive.getDistanceCm()[0];
            d.encoderPos.x = mecanumDrive.getDistanceCm()[1];
            Vector2D deltaMove = new Vector2D(d.encoderPos.x-encoderPreX,d.encoderPos.y-encoderPreY);
            deltaMove.rotateBy(Math.toRadians(d.heading));
            d.wPos.x += deltaMove.x;
            d.wPos.y += deltaMove.y;

            d.currentCommand.orientationSpeed = orientationCalc.CalcOrientation(d);
            d.currentCommand.motionSpeed = motionCalc.CalcMotion(d);
            Vector2D motion = new Vector2D(d.currentCommand.motionSpeed.x, d.currentCommand.motionSpeed.y);
            motion.rotateBy(Math.toRadians(-d.heading));
            d.currentCommand.motionSpeed.x = motion.x;
            d.currentCommand.motionSpeed.y = motion.y;
            d.currentCommand.speed = speedCalc.CalcSpeed(d);

            for (Interfaces.OtherCalc calc : otherCalc) calc.CalcOther(d);

//            boolean didOtherProgress = false;
//            for (Interfaces.OtherCalc calc : otherCalc) {
//                didOtherProgress = calc.doProgress(d);
//                if (didOtherProgress) break;
//            }
//            boolean didProgress =
//                    didOtherProgress ||
//                    speedCalc.doProgress(d) ||
//                    motionCalc.doProgress(d) ||
//                    orientationCalc.doProgress(d);

            if (d.timeRemainingUntilEndgame >= 0) endGameTime = (float)(Math.round(d.timeRemainingUntilEndgame / 100) / 10.0);

            //could add specific telemetry data to show through an implementation of complexOp
//          telemetry.addData("Progress found", didProgress);
            telemetry.addData("heding",d.debugData1);
            telemetry.addData("curve Progress",d.debugData2*1000/10);
            telemetry.addData("frontDist",Math.round(d.frontDist.getDistance(DistanceUnit.CM)*10)/10.0);
            telemetry.addData("Progress", Math.round(d.progress*1000)/10.0);
            telemetry.addData("time until endgame", endGameTime);
            telemetry.addData("time until end of match",Math.round(d.timeRemainingUntilMatch/100)/10.0);
            telemetry.addData("my distance fwd", Math.round(d.wPos.y*10)/10.0);
            telemetry.addData("my distance side", Math.round(d.wPos.x*10)/10.0);
            telemetry.addData("heading", Math.round(d.heading*10)/10.0);
            telemetry.addData("orientation", Math.round(d.currentCommand.orientationSpeed*10)/10.0);
            telemetry.addData("motiony", Math.round(d.currentCommand.motionSpeed.y*10)/10.0);
            telemetry.addData("motionx", Math.round(d.currentCommand.motionSpeed.x*10)/10.0);
            telemetry.addData("speed",Math.round(d.currentCommand.speed*10)/10.0);
            telemetry.update();

            mecanumDrive.driveMecanum(d.currentCommand.motionSpeed.y*d.currentCommand.speed, //I could make one that takes a vector as an arg //that would be cleaner// and my code
                    d.currentCommand.motionSpeed.x*d.currentCommand.speed,
                    d.currentCommand.orientationSpeed);


            d.progress = MathUtil.findMaxList(motionCalc.myProgress(d),orientationCalc.myProgress(d),
                                          speedCalc.myProgress(d));

            for (Interfaces.OtherCalc calc : otherCalc) {
                d.progress = Math.max(d.progress,calc.myProgress(d));
            }


            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
    }

    //How data is transferred between calculators and complexOp
    private Interfaces.MoveData d = new Interfaces.MoveData();//if you delete this the world will end

    void initHardware(HardwareMap hwMap) {
        d.robot = new RobotMap(hwMap);
        mecanumDrive = new MecanumDrive(d.robot);
    }

    public abstract Vector2D startPosition();

    public abstract void body() throws InterruptedException;

    public void exit(){//so we don't run into a wall at full speed
        d.robot.bright.setPower(-1);//this has multiple meanings lol
        d.robot.fright.setPower(-1);
        d.robot.bleft.setPower(-1);
        d.robot.fleft.setPower(-1);
    }



    @Override
    public void runOpMode() throws InterruptedException{

        //INITIALIZATION
        telemetry.addData("Initializing", "Started");
        telemetry.update();

        d.isFinished = false;
        d.isStarted = false;

        d.driver = new CompleteController();
        d.manip = new CompleteController();

        d.driver.CompleteController(gamepad1);
        d.manip.CompleteController(gamepad2);

        initHardware(hardwareMap);

        telemetry.addData("Initializing", "Finished");
        telemetry.update();

        waitForStart();

        //START POSITION
        d.StartPos.set(this.startPosition().clone());
        d.preWPos.set(this.startPosition().clone());
        d.wPos.set(this.startPosition().clone());

        d.isStarted = true;

        telemetry.addData("Body", "Started");
        telemetry.update();

        //BODY
        try {
            body();
        } catch (InterruptedException ie) {
            telemetry.addData("Interrupted","Exception");
            telemetry.update();
        }
        telemetry.addData("Body", "Finished");
        telemetry.update();

        //EXIT
        telemetry.addData("Exit", "Started");
        telemetry.update();
        exit();
        d.isFinished = true;
        telemetry.addData("Exit", "Finished");
        telemetry.update();
    }
}
