package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/26/2017.
 */
@Autonomous(name = "Blue_Corner", group = "Blue")
@Disabled
public class Blue_Corner extends LinearOpMode {


    public LynxI2cColorRangeSensor color1 = null;

    Hardware_robodux robot = new Hardware_robodux();
    BNO055IMU imu;


    double arm_down = 0.13;
    double arm_up = .56;


    double red = 0;
    double blue = 0;

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

        robot.intake_sevr.setPosition(0.63);
        robot.intake_sevl.setPosition(0.34);
        robot.clampl.setPosition(0.905);
        robot.clampr.setPosition(0.095);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = DEGREES;

        imu.initialize(parameters);



        waitForStart();
        sleep(20);

        robot.intake_sevl.setPosition(.57);
        robot.intake_sevr.setPosition(.43);

        robot.jewel_arm.setPosition(arm_down);
        robot.clampr.setPosition(0.095);
        robot.clampl.setPosition(0.905);

        sleep(650);

        robot.liftl.setTargetPosition(125);
        robot.liftr.setTargetPosition(125);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);

        sleep(100);

        red = color1.red()/15;
        blue = color1.blue()/11;

        telemetry.addData("Blue ", blue);
        telemetry.addData("Red ", red);
        telemetry.update();

        //If BLUE
        if(red > blue){


            DriveStraight(-160, 0.2);//back up to knock jewel off
            sleep(200);
            robot.jewel_arm.setPosition(arm_up);
            sleep(200);
            DriveStraight(1960, 0.25);//Drive Off stone

        }
        //If RED
        if (blue > red){

            DriveStraight(1800, 0.25);//Knock off Jewel and drive off stone
            robot.jewel_arm.setPosition(arm_up);
            sleep(100);

        }
        //If NONE
        if (blue == red){
            robot.jewel_arm.setPosition(arm_up);
            sleep(1000);
            DriveStraight(1800, .25);//Drive Off
        }
        sleep(100);
        robot.liftl.setTargetPosition(25);
        robot.liftr.setTargetPosition(25);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);
        GyroTurn(89,.2);//Turn 90 DEGREES
        sleep(100);
        DriveStraight(2880,.3);
        sleep(100);
        GyroTurn(1,.2);
        sleep(100);
        DriveStraight(800,.3);
        sleep(100);
        robot.clampr.setPosition(.19);
        robot.clampl.setPosition(.81);
        sleep(100);
        DriveStraight(-325,.3);
        robot.liftl.setTargetPosition(0);
        robot.liftr.setTargetPosition(0);
        robot.liftl.setPower(.5);
        robot.liftr.setPower(.5);
        sleep(2000);

    }

    public void GyroTurn (int target_angle, double speed) {



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

        while(MagDiff > 2){


            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;

            ADiff = target_angle - heading;
            MagDiff = abs(ADiff);
            EDiff = abs(robot.right_front_drive.getCurrentPosition()) - abs(robot.left_front_drive.getCurrentPosition());


            if (ADiff > 0 && MagDiff >= 12){
                right_motor_power = -speed;
                left_motor_power = speed;
            }

            if (ADiff > 0 && MagDiff < 12){
                right_motor_power = -speed/2.5;
                left_motor_power = speed/2.5;
            }

            if (ADiff < 0 && MagDiff >= 12){
                right_motor_power = speed;
                left_motor_power = -speed;
            }

            if (ADiff < 0 && MagDiff < 12){
                right_motor_power = speed/2.5;
                left_motor_power = -speed/2.5;
            }

            if (ADiff == 0){
                right_motor_power = 0;
                left_motor_power = 0;
            }

            if(EDiff > 0 && right_motor_power > 0) {right_motor_power = right_motor_power - (abs(EDiff) / Scaling_Factor);}
            if(EDiff > 0 && right_motor_power < 0) {right_motor_power = right_motor_power + (abs(EDiff) / Scaling_Factor);}

            if (EDiff < 0 && left_motor_power > 0){left_motor_power = left_motor_power -(abs(EDiff) / Scaling_Factor);}
            if (EDiff < 0 && left_motor_power < 0){left_motor_power = left_motor_power +(abs(EDiff) / Scaling_Factor);}


            robot.right_front_drive.setPower(right_motor_power);
            robot.right_rear_drive.setPower(right_motor_power);
            robot.left_front_drive.setPower(left_motor_power);
            robot.left_rear_drive.setPower(left_motor_power);
        }

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void DriveStraight(double distance, double inputpower) {

        double rightpower = 0;
        double leftpower = 0;
        double leftposition = 0;
        double rightposition = 0;
        double accmult;
        double deccmult;
        double diff;
        double scalingfactor = 600;
        double accdist = 132;
        double oneshot = 0;

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (((((abs(leftposition + rightposition)) / 2) + 25) < abs(distance)) || ((((abs(leftposition + rightposition)) / 2) - 25) > abs(distance))) {

            leftposition = robot.left_rear_drive.getCurrentPosition();
            rightposition = robot.right_front_drive.getCurrentPosition();

            if (distance < 0 && oneshot == 0) {
                inputpower = inputpower * (-1);
                oneshot = 1;
            }

            diff = rightposition - leftposition;

            if (diff > 0 && inputpower > 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff < 0 && inputpower > 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);

            }

            if (diff > 0 && inputpower < 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);
            }

            if (diff < 0 && inputpower < 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff == 0.0) {
                rightpower = inputpower;
                leftpower = inputpower;
            }

            if (abs(distance) < (2 * accdist)) {
                accmult = 1;
                deccmult = 1;
            } else {
                accmult = (0.05 / (abs(inputpower))) + ((abs((rightposition + leftposition) / 2)) / accdist);
                if (accmult > 1)
                    accmult = 1;

                if ((abs(distance - ((rightposition + leftposition) / 2))) < accdist) {
                    deccmult = ((0.05 / abs(inputpower)) + ((abs(distance - ((rightposition + leftposition) / 2))) / accdist));
                    if (deccmult > 1)
                        deccmult = 1;
                } else
                    deccmult = 1;
            }
            rightpower = rightpower * accmult * deccmult;
            leftpower = leftpower * accmult * deccmult;

            //set motors to motor powers
            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);



        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

