package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "driver teleop (new)", group = "14174")
public class jankSwerve extends LinearOpMode {
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor slide;
    public Servo plug;
    public Servo plugArm;
    double offset = 0;
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        slide = hardwareMap.get(DcMotor.class, "slide");
        plug = hardwareMap.get(Servo.class, "plug");
        plugArm = hardwareMap.get(Servo.class, "plugArm");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        plug.setPosition(0.03);
        plugArm.setPosition(0.77);


        BNO055IMU imu;
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters2);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double spool = -gamepad2.right_stick_y;
            double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + offset;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (spool !=0) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setPower(spool);
            } else  if (spool == 0 && gamepad2.right_trigger > 0.01){
                slide.setTargetPosition(slide.getCurrentPosition());
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
            } else {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.setPower(0);
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]


                if(gamepad1.right_trigger > 0.1) {
                    front_left.setPower( 0.4 *frontLeftPower);
                    back_left.setPower(0.4 * backLeftPower);
                    front_right.setPower(0.4 * frontRightPower);
                    back_right.setPower(0.4 * backRightPower);
                } else if (gamepad1.left_trigger > 0.1) {
                    front_left.setPower(frontLeftPower);
                    back_left.setPower(backLeftPower);
                    front_right.setPower(frontRightPower);
                    back_right.setPower(backRightPower);
                } else {
                    front_left.setPower( 0.7 *frontLeftPower);
                    back_left.setPower(0.7 * backLeftPower);
                    front_right.setPower(0.7 * frontRightPower);
                    back_right.setPower(0.7 * backRightPower);
                }

            if(gamepad1.right_bumper) {

                offset = -botHeading;
            }



            if(gamepad1.a) {
                plug.setPosition(0.5);
            }
            if (gamepad1.b) {
                plug.setPosition(0.03);
            }
            if(gamepad2.a) {
                plugArm.setPosition(0.77);
            }
            if(gamepad2.b) {
                plugArm.setPosition(0.18);
            }
            if(gamepad2.x) {
                plugArm.setPosition(plugArm.getPosition() - 0.01);
            }
            if(gamepad2.y) {
                plugArm.setPosition(plugArm.getPosition() + 0.01);
            }
            telemetry.addData("Servo Position", plug.getPosition());
            telemetry.addData("Servo Position", plugArm.getPosition());
            telemetry.addData("Spool Position", slide.getCurrentPosition());
            telemetry.addData("GamePad2Pos", gamepad2.left_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("heading", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
