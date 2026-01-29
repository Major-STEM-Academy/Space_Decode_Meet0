package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class Teleop extends LinearOpMode {

    /*
    private enum State {
        ARM_UP,
        ARM_DOWN,
        HANG_INITIALIZATION,
        HANG,
    }
    State state = State.ARM_DOWN;
*/
    private ElapsedTime runtime = new ElapsedTime();
    //Servo intake = null;
    //Servo tilt = null;
    //Servo extent = null;
    //Servo rotator = null;
    //Servo grabber = null;
    //Servo grabber_tilt = null;
    //TouchSensor touch = null;

    /*
    public static double GRABBER_INIT = 0.5;
    public static double GRABBER_FORWARD = 0;
    public static double GRABBER_BACKWARD = 1;

    public static double GRABBER_ROTATOR_INIT = 0.43;
    public static double GRABBER_ROTATOR_DOWN = 0.38;
    public static double GRABBER_ROTATOR_UP = 0.44;

    public static double EXTENT_INIT = 0.9;
    public static double EXTENT_OUT = 0.3;
    public static double EXTENT_BACK = EXTENT_INIT;

    public static double TILT_INIT = 0.285;
    public static double TILT_DOWN = TILT_INIT;
    public static double TILT_UP = 0.55;

     */
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.00003;

    public static double f = 0.005;

    public static int target = 0;
    public boolean leftOpen = false;
    public boolean rightOpen = false;
    private final double ticks_in_degree = 700 / 180.0;

    private static final double STEP_INCHES = 1;
    private static final int TICKS_PER_INCH = 20;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor Fly = null;
    private DcMotor Intake = null;
    private Servo Feeder = null;
    private Servo Indexer = null;


    private double Feedermin = 1;
    private double Feedermax = 0.5;

    private double Indexerhome = 0;
    private double Indexamount = 0.16666;
    private double Index = Indexerhome;
    private boolean mode = false;


    @Override
    public void runOpMode() throws InterruptedException {



        /*
        frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

         */

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        Fly = hardwareMap.dcMotor.get("Fly");
        Intake = hardwareMap.dcMotor.get("Intake");
        Feeder = hardwareMap.servo.get("Feeder");
        Indexer = hardwareMap.servo.get("Indexer");

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Feeder.setPosition(Feedermin);
        Indexer.setPosition(Indexerhome);



        /*
        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

        CRServo launchServo = hardwareMap.crservo.get("launcher");
        grabberTilt = hardwareMap.servo.get("grabberTilt");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL = hardwareMap.servo.get("grabberL");
        */

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);;
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);;

        controller = new PIDController(p, i, d);

        GamepadEx game1 = new GamepadEx(gamepad1);
        GamepadEx game2 = new GamepadEx(gamepad2);

        /*
        launchServo.setPower(0);
        grabberL.setPosition(BotCoefficients.grabberLeftClose);
        grabberR.setPosition(BotCoefficients.grabberRightClose);
        grabberTilt.setPosition(BotCoefficients.grabberUp);

         */
        telemetry.addData(">","ready");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {

            /*
            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x*0.8;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1.2);
            double frontLeftPower = (vertical - horizontal + turn) / denominator;
            double frontRightPower = (vertical + horizontal - turn) / denominator;
            double backLeftPower = (vertical + horizontal + turn) / denominator;
            double backRightPower = (vertical - horizontal - turn) / denominator;

            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

             */
            double horizontal = -1.0 * gamepad1.left_stick_x * 0.6;
            double vertical = gamepad1.left_stick_y * 0.6;
            double turn = -1.0 * gamepad1.right_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            setDrivePower(flPower, frPower, blPower, brPower);

            //
            if (gamepad1.left_trigger >= 0.05) {
                Intake.setPower(gamepad1.left_trigger);
                mode = false;
            } else {
                Intake.setPower(0);
            }
            if (gamepad1.right_trigger >= 0.05) {
                Fly.setPower(gamepad1.right_trigger);
            } else {
                Fly.setPower(0);
            }
            if (gamepad1.right_bumper) {
                Feeder.setPosition(Feedermax);
                mode = true;
            } else {
                Feeder.setPosition(Feedermin);
            }
            if (gamepad1.right_stick_y >= 0.1) {
                Index = Index + Indexamount;
            } else if (gamepad1.right_stick_y <= -0.1) {
                Index = Index - Indexamount;
            }









        }

    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(leftInches * 39.79);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(rightInches * 39.79);
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * 39.79);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * 39.79);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


}
