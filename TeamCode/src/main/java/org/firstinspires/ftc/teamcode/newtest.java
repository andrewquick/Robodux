package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/26/2017.
 */
@Autonomous(name = "newtest", group = "Blue")
@Disabled
public class newtest extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
    public LynxI2cColorRangeSensor color1 = null;

    Hardware_robodux robot = new Hardware_robodux();
    BNO055IMU imu;


    double arm_down = 0.098;
    double arm_up = .69;


    double red = 0;
    double blue = 0;

    double total_drive = 2555;

    int zone = 0;

    boolean spike = false;

    double initialheading = 0;

    double time = 0;

    double rjs_down = .375;
    double rjs_up = 1;
    double ljs_down = .47;
    double ljs_up = 0;

    boolean once = false;
    int i = 0;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color1");
        robot.init(hardwareMap);

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.jewel_arm.setPosition(arm_up);

        robot.intake_sevr.setPosition(0.63);//one is closed
        robot.intake_sevl.setPosition(0.34);//zero is closed
        robot.clampl.setPosition(0.905);
        robot.clampr.setPosition(0.095);
        robot.right_jewel_stop.setPosition(rjs_up);
        robot.left_jewel_stop.setPosition(ljs_up);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = DEGREES;

        imu.initialize(parameters);

        VuforiaLocalizer.Parameters vuparameters = new VuforiaLocalizer.Parameters();
        vuparameters.vuforiaLicenseKey = "AQ4c5An/////AAAAGXHPS/VK/kM9p1fd0MoGsN5AKZHfSVWfqv0W1ccbbHtwCY0bnCb6rbBGQSzIqLtSUIzim4PQLMdvUS+6Q5g873CX538/L/4FY+HrtluUIIKTQefegsH27VRIlHgue83sg6mFSPUlbvXXND52Axl8eYEV6LHwru/dCqaAkHdT3dp18+l2nOS11sw5P8NeTiO06D5zg5NIB7L+qfGAYpnWrq8YBCJw2xcZFyZKcj+sqQzYJvGBbokn/dKmxwO5xteK3uvW908EcJ/1jGtY73MwnbkXO2QTRnVqN924N509GxecFAG15XY4UDZtNVS5LLB8Ik3u85o7K7nlQ47MUObGCicSIa0I2+tszy6VPsZjIikw\n";

        vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuparameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

    DriveStraight(5000, .4);
    }


    public void GyroTurn(double target_angle, double speed) {


        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double ADiff;
        double MagDiff = 0;
        double EDiff;
        double Scaling_Factor = 500;
        double right_motor_power = 0;
        double left_motor_power = 0;
        Orientation angle;
        double heading;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        ADiff = target_angle - heading;
        MagDiff = abs(ADiff);

        while (MagDiff > 2) {


            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;

            ADiff = target_angle - heading;
            MagDiff = abs(ADiff);
            EDiff = abs(robot.right_front_drive.getCurrentPosition()) - abs(robot.left_front_drive.getCurrentPosition());


            if (ADiff > 0 && MagDiff >= 12) {
                right_motor_power = -speed;
                left_motor_power = speed;
            }

            if (ADiff > 0 && MagDiff < 12) {
                right_motor_power = -speed / 2.5;
                left_motor_power = speed / 2.5;
            }

            if (ADiff < 0 && MagDiff >= 12) {
                right_motor_power = speed;
                left_motor_power = -speed;
            }

            if (ADiff < 0 && MagDiff < 12) {
                right_motor_power = speed / 2.5;
                left_motor_power = -speed / 2.5;
            }

            if (ADiff == 0) {
                right_motor_power = 0;
                left_motor_power = 0;
            }

            if (EDiff > 0 && right_motor_power > 0) {
                right_motor_power = right_motor_power - (abs(EDiff) / Scaling_Factor);
            }
            if (EDiff > 0 && right_motor_power < 0) {
                right_motor_power = right_motor_power + (abs(EDiff) / Scaling_Factor);
            }

            if (EDiff < 0 && left_motor_power > 0) {
                left_motor_power = left_motor_power - (abs(EDiff) / Scaling_Factor);
            }
            if (EDiff < 0 && left_motor_power < 0) {
                left_motor_power = left_motor_power + (abs(EDiff) / Scaling_Factor);
            }


            robot.right_front_drive.setPower(right_motor_power);
            robot.right_rear_drive.setPower(right_motor_power);
            robot.left_front_drive.setPower(left_motor_power);
            robot.left_rear_drive.setPower(left_motor_power);
        }

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initialheading = -angle.firstAngle;

    }


    public void DriveStraight(double distance, double inputpower) {
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .03;
        double accdist = 250;
        double decdist = 350;
        double rightspeed = 0;
        double leftspeed = 0;
        double currentleft = 0;
        double currentright = 0;
        double switchpoint = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        while ((currentright < distance) || (currentleft < distance)) {
            heading = -angle.firstAngle;
            currentright = abs(robot.right_front_drive.getCurrentPosition());
            currentleft = abs(robot.left_front_drive.getCurrentPosition());

            if (distance >= (accdist + decdist)) {
                if (((currentright + currentleft) / 2) <= accdist) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / accdist) * (inputpower - startingspeed)) * (1 + (heading / 50));
                        leftspeed = (startingspeed + (currentleft / accdist) * (inputpower - startingspeed)) * (1 - (heading / 50));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 + (heading / 50));
                        leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 - (heading / 50));
                    }
                }
                if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                    rightspeed = inputpower * (1 + (heading / 50));
                    leftspeed = inputpower * (1 - (heading / 50));
                }
                if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + (heading / 50));
                        leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - (heading / 50));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 + (heading / 50));
                        leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 - (heading / 50));
                    }
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = (accdist + decdist) / 2;

                if (((currentright + currentleft) / 2) <= switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / switchpoint) * (inputpower - startingspeed));
                        leftspeed = (startingspeed + (currentleft / switchpoint) * (inputpower - startingspeed));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / switchpoint) * (-inputpower - startingspeed));
                        leftspeed = -(startingspeed + (currentleft / switchpoint) * (-inputpower - startingspeed));
                    }
                }
                if (((currentright + currentleft) / 2) > switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / switchpoint) * (inputpower - startingspeed));
                        leftspeed = (startingspeed + ((distance - currentleft) / switchpoint) * (inputpower - startingspeed));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / switchpoint) * (-inputpower - startingspeed));
                        leftspeed = -(startingspeed + ((distance - currentleft) / switchpoint) * (-inputpower - startingspeed));
                    }
                }
            }
            robot.left_front_drive.setPower(leftspeed);
            robot.left_rear_drive.setPower(leftspeed);
            robot.right_front_drive.setPower(rightspeed);
            robot.right_rear_drive.setPower(rightspeed);
            telemetry.addData("heading",heading);
            telemetry.addData("rightspeed", rightspeed);
            telemetry.update();
        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

