package org.firstinspires.ftc.teamcode.Calculators;

import org.firstinspires.ftc.teamcode.Hardware.SkystoneRobotNameBox.RobotMap;
import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Hardware.Sensors.CompleteController;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Interfaces {

    public static class MoveData{
        public static class Command{
            public double speed;
            public Vector2D motionSpeed;
            public double orientationSpeed;
            public Command(double speed, Vector2D motionSpeed, double orientationSpeed){
                this.speed = speed;
                this.motionSpeed = motionSpeed;
                this.orientationSpeed = orientationSpeed;
            }
        }

        public RobotMap robot;
        public Vector2D StartPos = new Vector2D();
        public double heading = 0;
        public double debugData1 = 0;
        public double debugData2 = 0;
        public boolean debugDataBool;
        public DistanceSensor frontDist;
        public DistanceSensor backDist;
        public DistanceSensor rightDist;
        public DistanceSensor leftDist;
        public ColorSensor colorSensor;
        public boolean firstLoop = true;
        public double timeRemainingUntilEndgame = 0;
        public double timeRemainingUntilMatch = 0;
        public CompleteController driver, manip;
        public boolean isStarted, isFinished;
        public Command currentCommand = null;
        public Command lastCommand = null;
        public Vector2D wPos = new Vector2D();//current robot world position //updated outside of calculators
        public Vector2D preWPos = new Vector2D();//this is updated inside of the calculator
        public Vector2D encoderPos = new Vector2D();
        public int currentSpin = 0;
        public boolean foundSpin = false;
        public double orientationError = 0;
        public double orientationP = 0.5;
        public double motionProgress = 0;
        public double orientationProgress = 0;
        public double speedProgress = 0;
        public double otherProgress = 0;
        public double previousSpeed = 0;
        public double progress;


    }

    public interface ProgressCalc{
        double myProgress(MoveData d);
    }


    public interface MotionCalc extends ProgressCalc{
        Vector2D CalcMotion(MoveData d);
    }

    public interface OrientationCalc extends ProgressCalc{
        double CalcOrientation(MoveData d);
    }

    public interface SpeedCalc extends ProgressCalc{
        double CalcSpeed(MoveData d);
    }

    public interface OtherCalc extends ProgressCalc{
        void CalcOther(MoveData d);
    }

}
