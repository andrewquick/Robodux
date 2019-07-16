package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/26/2017.
 */
@Disabled
@Autonomous(name = "Backwards", group = "Blue")
public class Backwards extends LinearOpMode {

    Hardware_robodux18 robot = new Hardware_robodux18();
    BNO055IMU imu;


    double initialheading = 0;
    double time = 0;
    boolean once = false;
    int i = 0;
    int position = 0;
    double timer = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.trapdoor.setPosition(0);
        robot.dumpright.setPosition(.92);
        robot.dumpleft.setPosition(.08);

        robot.latch.setPosition(.38);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = DEGREES;

        imu.initialize(parameters);

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        DriveStraight(2000, -.3);
        sleep(100);
        DriveStraight(2000, .3);
        sleep(100);
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
                        rightspeed = (startingspeed + (currentright / accdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + (currentleft / accdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                    }
                }
                if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                    if(inputpower >= 0) {
                        rightspeed = inputpower * (1 + ((heading - originalheading) / 40));
                        leftspeed = inputpower * (1 - ((heading - originalheading) / 40));
                    }
                    if(inputpower < 0){
                        rightspeed = inputpower * (1 - ((heading - originalheading) / 40));
                        leftspeed = inputpower * (1 + ((heading - originalheading) / 40));
                    }
                }
                if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                    }
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = (accdist + decdist) / 2;

                if (((currentright + currentleft) / 2) <= switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / switchpoint) * (inputpower - startingspeed)* (1 + ((heading - originalheading) / 40)));
                        leftspeed = (startingspeed + (currentleft / switchpoint) * (inputpower - startingspeed)* (1 - ((heading - originalheading) / 40)));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
                if (((currentright + currentleft) / 2) > switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / switchpoint) * (inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / switchpoint) * (inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
            }
            robot.left_front_drive.setPower(leftspeed);
            robot.left_rear_drive.setPower(leftspeed);
            robot.right_front_drive.setPower(rightspeed);
            robot.right_rear_drive.setPower(rightspeed);
        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

