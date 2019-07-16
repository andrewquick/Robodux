/*package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name= "Test_Auto_Linear")

public class Test_Auto_Linear extends LinearOpMode {

    Hardware_robodux robot = new Hardware_robodux();


        static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.Testmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Testmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.Testmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Testmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 48, 48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Testmotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.Testmotor2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.Testmotor.setTargetPosition(newLeftTarget);
            robot.Testmotor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Testmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Testmotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Testmotor.setPower(Math.abs(speed));
            robot.Testmotor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (robot.Testmotor.isBusy() && robot.Testmotor2.isBusy())) {


            }

            // Stop all motion;
            robot.Testmotor.setPower(0);
            robot.Testmotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Testmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Testmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }}
*/