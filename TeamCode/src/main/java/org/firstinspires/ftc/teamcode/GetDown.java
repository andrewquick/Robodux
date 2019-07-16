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
@Autonomous(name = "GetDown", group = "Blue")
public class GetDown extends LinearOpMode {

    OpenGLMatrix lastLocation = null;
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
        robot.dumpright.setPosition(.95);
        robot.dumpleft.setPosition(.05);

        robot.latch.setPosition(.38);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = DEGREES;

        imu.initialize(parameters);

        waitForStart();

        robot.lift.setPower(.5);
        sleep(700);
        robot.latch.setPosition(0);
        sleep(500);
        robot.lift.setPower(0);
        sleep(1300);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(75);
        robot.lift.setPower(.5);
        //sleep(75);
        robot.lift.setTargetPosition(-3000);
        sleep(200);
        OldDriveStraight(-1800, .4);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setPower(.4);
        sleep(3000);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveStraight(575, .4);
        robot.intakearm.setPower(.7);
        robot.intakearm.setTargetPosition(-1900);
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
        double endspeed = .05;
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
            tdeccdist = target - 50;
        }
        if(target < startangle) {
            right = false;
            taccdist = startangle - 20;
            tdeccdist = target + 50;
        }
        while((heading < (target-1)) || (heading  > (target + 1))){
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
                        rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                }
                if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                    rightspeed = inputpower * (1 + ((heading - originalheading) / 40));
                    leftspeed = inputpower * (1 - ((heading - originalheading) / 40));
                }
                if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
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
    public void OldDriveStraight(double distance, double inputpower) {

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

