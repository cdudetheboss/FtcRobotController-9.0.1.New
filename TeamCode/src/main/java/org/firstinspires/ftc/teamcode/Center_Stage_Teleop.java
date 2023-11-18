package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Center_Stage_Teleop", group = "14174")
public class Center_Stage_Teleop extends LinearOpMode {
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor lift;
    double botHeading = 0;
    double offset = 0;
    //int maxCap = 1500;
    double headingResetValue;
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        lift = hardwareMap.get(DcMotor.class, "lift");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters2);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if(gamepad1.right_bumper) {
                offset = -botHeading;
            }
            botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + offset;
            double liftPosition = lift.getCurrentPosition();
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        if(gamepad1.dpad_up) {
            lift.setPower(0.8);
           /* if(liftPosition>maxCap) {
                lift.setPower(0);
            } */
        }
        if(gamepad1.dpad_down) {
            lift.setPower(-0.8);
            if (liftPosition<=0) {
                lift.setPower(0);
            }
        }


            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]


            if(gamepad1.left_trigger > 0.1) {
                front_left.setPower(0.4 * frontLeftPower);
                back_left.setPower(0.4 * backLeftPower);
                front_right.setPower(0.4 * frontRightPower);
                back_right.setPower(0.4 * backRightPower);
            } else if (gamepad1.right_trigger > 0.1) {
                front_left.setPower(frontLeftPower);
                back_left.setPower(backLeftPower);
                front_right.setPower(frontRightPower);
                back_right.setPower(backRightPower);
            } else {
                front_left.setPower(0.7 * frontLeftPower);
                back_left.setPower(0.7 * backLeftPower);
                front_right.setPower(0.7 * frontRightPower);
                back_right.setPower(0.7 * backRightPower);
            }







            telemetry.addData("Status", "Running");
            telemetry.addData("LiftPosition", liftPosition);
            telemetry.addData("heading", (Math.toDegrees(botHeading)));
            telemetry.update();
        }
    }
}
