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
@Autonomous(name = "New Drive Straight")
@Disabled

public class New_Drive_Straight extends LinearOpMode {


    Hardware_robodux robot = new Hardware_robodux();
    BNO055IMU imu;

    double arm_up = .69;

    double rjs_up = 1;
    double ljs_up = 0;

    double distance = 0;
    double inputpower = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = DEGREES;

        imu.initialize(parameters);

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

        waitForStart();

        DriveStraight(1500, .4);
        sleep(100);
        GyroTurn(87, .4);
        sleep(100);
        DriveStraight(1000, -.4);
        sleep(100);
        GyroTurn(-87,.4);
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

    double startingspeed = .09;
    double accdist = 250;
    double decdist = 350;
    double rightspeed = 0;
    double leftspeed = 0;
    double currentleft = 0;
    double currentright = 0;
    double switchpoint = 0;
    double originalheading = 0;

    angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    heading = -angle.firstAngle;
    originalheading = heading;

    while ((currentright < distance) || (currentleft < distance)) {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;
        currentright = abs(robot.right_front_drive.getCurrentPosition());
        currentleft = abs(robot.left_front_drive.getCurrentPosition());

        if (distance >= (accdist + decdist)) {
            if (((currentright + currentleft) / 2) <= accdist) {
                if (inputpower > 0) {
                    rightspeed = (startingspeed + (currentright / accdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 50));
                    leftspeed = (startingspeed + (currentleft / accdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 50));
                }
                if (inputpower < 0) {
                    rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 50));
                    leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 50));
                }
            }
            if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                rightspeed = inputpower * (1 + ((heading - originalheading) / 50));
                leftspeed = inputpower * (1 - ((heading - originalheading) / 50));
            }
            if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                if (inputpower > 0) {
                    rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 50));
                    leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 50));
                }
                if (inputpower < 0) {
                    rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 50));
                    leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 50));
                }
            }
        }
        if (distance < (accdist + decdist)) {
            switchpoint = (accdist + decdist) / 2;

            if (((currentright + currentleft) / 2) <= switchpoint) {
                if (inputpower > 0) {
                    rightspeed = (startingspeed + (currentright / switchpoint) * (inputpower - startingspeed)* (1 + ((heading - originalheading) / 50)));
                    leftspeed = (startingspeed + (currentleft / switchpoint) * (inputpower - startingspeed)* (1 - ((heading - originalheading) / 50)));
                }
                if (inputpower < 0) {
                    rightspeed = -(startingspeed + (currentright / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 50));
                    leftspeed = -(startingspeed + (currentleft / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 50));
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

    public void GyroTurn(double target, double inputspeed) {


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

        double taccdist = 0;
        double tdeccdist = 0;
        double startangle = 0;
        boolean right = false;
        double startspeed = .12;
        double endspeed = .08;
        double speed = 0;
        double rightspeed = 0;
        double leftspeed = 0;
        double ediff = 0;
        double scalefactor = 525;
        double rightpower = 0;
        double leftpower = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        startangle = heading;

        if(target > startangle) {
            right = true;
            taccdist = startangle + 20;
            tdeccdist = target - 40;
        }
        if(target < startangle) {
            right = false;
            taccdist = startangle - 20;
            tdeccdist = target + 40;
        }
        while((heading < (target-.7)) || (heading  > (target + .7))){
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;
            ediff = abs((robot.right_front_drive.getCurrentPosition() + robot.right_rear_drive.getCurrentPosition())/2)-abs((robot.left_front_drive.getCurrentPosition() + robot.left_rear_drive.getCurrentPosition())/2);

            if(right) {
                if (heading < taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading >= taccdist && heading <= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading > tdeccdist) {
                    speed = endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed);
                }
                rightspeed = -speed;
                leftspeed = speed;
            }
            if(!right){
                if (heading > taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading <= taccdist && heading >= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading < tdeccdist) {
                    speed = endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed);
                }
                rightspeed = speed;
                leftspeed = -speed;
            }
            rightpower = rightspeed * (1 - (ediff/scalefactor));
            leftpower = leftspeed * (1 + (ediff/scalefactor));

            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);
        }
        robot.right_front_drive.setPower(0);
        robot.right_rear_drive.setPower(0);
        robot.left_front_drive.setPower(0);
        robot.left_rear_drive.setPower(0);
    }
}
