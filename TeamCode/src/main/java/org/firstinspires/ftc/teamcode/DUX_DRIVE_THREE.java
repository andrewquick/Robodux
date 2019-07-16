package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware_robodux18;

import static java.lang.Math.abs;

//*** Created by Quick's on 10/14/2017.

@Disabled
@TeleOp(name = "DUX_DRIVE_THREE")

public class DUX_DRIVE_THREE extends OpMode {
    Hardware_robodux18 robot = new Hardware_robodux18();

    int state = 0;

    double tdopen = .8;
    double tdclosed = 0;

    int armup = 895;
    int armdown = -1900;

    int liftdown = 0;
    int liftprelatch = 15;
    int liftlatch = -1660;
    int liftup = -3310;

    double right = 0;
    double left = 0;
    double delay = .2;
    double time = 0;
    double range = 10;
    boolean oneshot = false;
    boolean ready = true;
    double dumpdown = .95;
    double dumpgold = .4;
    double dumpsilver = .55;
    boolean zerospecial = false;
    boolean ninespecial = false;
    boolean once = true;
    double liftpower = .95;
    double armpower = .85;

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.intakearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.latch.setPosition(0);
        state = 0;
    }

    @Override
    public void loop() {
        if(once){
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakearm.setPower(.85);
            //robot.intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            once = false;
        }
        if(gamepad1.right_bumper){
            left = .28;
            right = .28;
        }
        else if(gamepad1.left_bumper){
            left = -.28;
            right = -.28;
        }
        else{
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }

        if(gamepad1.right_trigger>.95 && gamepad1.left_trigger>.95 && gamepad1.a){
            state = 999;
            ready = false;
            oneshot = true;
        }
        if(ready && state == 999){
            state = 998;
            ready = false;
            oneshot = true;
        }
        if (state == 999) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(.3);
            }

            robot.intakearm.setTargetPosition(armup);
            //robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(0);

            if ((time + 5) < getRuntime()) {
                ready = true;
            }
        }
        if(state == 998){
            if(oneshot){
                ready = false;
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setTargetPosition(0);
                oneshot = false;
            }
            if(!oneshot){
                state = 0;
                ready = true;
            }
        }
        robot.right_front_drive.setPower(right);
        robot.right_rear_drive.setPower(right);
        robot.left_front_drive.setPower(left);
        robot.left_rear_drive.setPower(left);

        //telemetry.addData("state", state);
        //telemetry.addData("ready", ready);
        //telemetry.addData("lift", robot.lift.getCurrentPosition());

        if((state == 1 || state == 2 || state == 3)&& ready && gamepad2.a){
            state = 12;
            ready = false;
            oneshot = true;
        }

        if(state == 0 && gamepad2.dpad_down && ready){
            state = 1;
            ready = false;
            oneshot = true;
            robot.latch.setPosition(0);
        }
        if(state == 1 && gamepad2.right_bumper && ready && !zerospecial){
            state = 2;
            ready = false;
            oneshot = true;
        }
        if((state == 2 || state == 1 || state ==3 || state == 12)&& !gamepad2.right_bumper && !gamepad2.left_bumper && !gamepad2.a && ready){
            state = 1;
            ready = false;
            oneshot = true;
        }
        if(state == 1 && gamepad2.left_bumper && ready){
            state = 3;
            ready = false;
            oneshot = true;
        }
        if((state == 2 || state == 1) && gamepad2.dpad_up && ready){
            state = 4;
            ready = false;
            oneshot = true;
        }
        if(state == 4 && ready){
            state = 0;//shut off intake
            ready = false;
            oneshot = true;
            zerospecial = true;
        }
        if(state == 0 && zerospecial && ready){
            state = 5;//open trap door
            ready = false;
            oneshot = true;
            zerospecial = false;
        }
        if(state == 5 && ready){
            state = 6;//turn on intake
            ready = false;
            oneshot = true;
        }
        if(state == 6 && ready){
            state = 50;//turn off intake
            ready = false;
            oneshot = true;
        }
        if(state == 50 && ready && gamepad2.dpad_down){
            state = 1;
            ready = false;
            oneshot = true;
        }
        if(state == 50 && ready && gamepad2.y){
            state = 7;//raise lift
            ready = false;
            oneshot = true;
        }
        /*if(state == 7 && ready){
            state = 9;//shut trap door
            ready = false;
            oneshot = true;
        }*/
        if(state == 7 && ready && gamepad2.x){
            state = 10;//dump silver
            ready = false;
            oneshot = true;
        }
        if(state == 10 && ready){
            state = 90;
            ready = false;
            oneshot = true;
        }
        if(state == 90 && ready){
            state = 0;
            ready = false;
            oneshot = true;
        }
        if(state == 0 && ready && gamepad2.y){
            state = 9;
            ready = false;
            oneshot = true;
        }
        if((state == 9 || state == 7) && ready && gamepad2.b){
            state = 11;//dump gold
            ready = false;
            oneshot = true;
        }
        if(state == 11 && ready) {
            state = 90;
            ready = false;
            oneshot = true;
        }


        if((state ==0 || state == 7 || state == 9 || state == 6 || state == 50) && ready && gamepad2.right_trigger>.95){
            state = 100;
            ready = false;
            oneshot = true;
        }
        if(state == 100 && ready && gamepad2.left_trigger>.95){
            state = 0;
            ready = false;
            oneshot = true;
        }
        if(state == 100 && ready && gamepad2.right_trigger>.95){
            state = 99;
            ready = false;
            oneshot = true;
        }
        if(state == 99 && ready){
            robot.latch.setPosition(.38);
        }

        if (state == 0) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + delay) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 1) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armdown);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(0);

            if (robot.intakearm.getCurrentPosition() > armdown - range && robot.intakearm.getCurrentPosition() < armdown + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .1) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 2) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armdown);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(1);

            if (robot.intakearm.getCurrentPosition() > armdown - range && robot.intakearm.getCurrentPosition() < armdown + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .1) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 3) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armdown);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(-.3);

            if (robot.intakearm.getCurrentPosition() > armdown - range && robot.intakearm.getCurrentPosition() < armdown + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .1) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 4) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(.6);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(1);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + delay) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 5) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdopen);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .3) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 50) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdopen);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .3) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 6) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdopen);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.75);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .3) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 7) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftup);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdopen);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftup - range && robot.lift.getCurrentPosition() < liftup + range && (time + .5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 9) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftup);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftup - range && robot.lift.getCurrentPosition() < liftup + range && (time + .5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 90) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftup);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftup - range && robot.lift.getCurrentPosition() < liftup + range && (time + 1.25) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 10) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftup);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpsilver);
            robot.dumpleft.setPosition(1-dumpsilver);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftup - range && robot.lift.getCurrentPosition() < liftup + range && (time + 1.5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 99) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(.6);
            robot.lift.setPower(.75);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 100) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftlatch);
            robot.intakearm.setPower(.6);
            robot.lift.setPower(.8);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftlatch - range && robot.lift.getCurrentPosition() < liftlatch + range && (time + .5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 11) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armup);
            robot.lift.setTargetPosition(liftup);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(tdclosed);
            robot.dumpright.setPosition(dumpgold);
            robot.dumpleft.setPosition(1-dumpgold);
            robot.intake.setPower(.0);

            if (robot.intakearm.getCurrentPosition() > armup - range && robot.intakearm.getCurrentPosition() < armup + range && robot.lift.getCurrentPosition() > liftup - range && robot.lift.getCurrentPosition() < liftup + range && (time + 1.5) < getRuntime()) {
                ready = true;
            }
        }
        if (state == 12) {

            if (oneshot) {
                time = getRuntime();
                ready = false;
                oneshot = false;
            }
            robot.intakearm.setTargetPosition(armdown);
            robot.lift.setTargetPosition(liftdown);
            robot.intakearm.setPower(armpower);
            robot.lift.setPower(liftpower);
            robot.trapdoor.setPosition(.35);
            robot.dumpright.setPosition(dumpdown);
            robot.dumpleft.setPosition(1-dumpdown);
            robot.intake.setPower(1);

            if (robot.intakearm.getCurrentPosition() > armdown - range && robot.intakearm.getCurrentPosition() < armdown + range && robot.lift.getCurrentPosition() > liftdown - range && robot.lift.getCurrentPosition() < liftdown + range && (time + .2) < getRuntime()) {
                ready = true;
            }
        }
        //telemetry.addData("lift", robot.lift.getCurrentPosition());
        }

    @Override
    public void stop(){
    }
}
