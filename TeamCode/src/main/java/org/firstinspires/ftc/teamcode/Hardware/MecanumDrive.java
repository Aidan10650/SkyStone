package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.Hardware.SkystoneRobotNameBox.RobotMap;

public class MecanumDrive {

    RobotMap robot;

    public static double GEAR_RATIO = 1.0; // for simulator - ours should be 0.5f;
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)  GoBilda should be 383.6f

    public static double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    private double maxSpeed = 1.0;

    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);

    private int fleftOffset;
    private int frightOffset;
    private int brightOffset;
    private int bleftOffset;


    public MecanumDrive(RobotMap robot) {
        this.robot = robot;
        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = maxSpeed;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        robot.fleft.setPower(flSpeed / largest);
        robot.fright.setPower(frSpeed / largest);
        robot.bleft.setPower(blSpeed / largest);
        robot.bright.setPower(brSpeed / largest);
    }

    public void driveMecanum(double forward, double strafe, double rotate) {
        double fleftSpeed = forward + strafe + rotate;
        double frightSpeed = forward - strafe - rotate;
        double bleftSpeed = forward - strafe + rotate;
        double brightSpeed = forward + strafe - rotate;

        setSpeeds(fleftSpeed, frightSpeed, bleftSpeed, brightSpeed);
    }

    public double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((robot.fleft.getCurrentPosition() - fleftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((robot.fright.getCurrentPosition() - frightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((robot.bleft.getCurrentPosition() - bleftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setEncoderOffsets() {
        frightOffset = robot.fright.getCurrentPosition();
        fleftOffset = robot.fleft.getCurrentPosition();
        bleftOffset = robot.bleft.getCurrentPosition();
        brightOffset = robot.bright.getCurrentPosition();
    }
}
