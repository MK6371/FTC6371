package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Holonomic;

/**
 * Created by 292486 on 10/13/2016.
 */

public class EncoderTrackingHolonomic extends HolonomicDrive {

    double encoderValFLeft = 0;
    double encoderValFRight = 0;
    double encoderValBLeft = 0;
    double encoderValBRight = 0;

    double encoderVal = 0;

    private double y, x, l, r;

    @Override
    public void loop() {
        y = -gamepad1.left_stick_y; //For some reason, the left y-axis stick is negative when pushed forward
        x = gamepad1.right_stick_x;
        l = gamepad1.left_trigger;
        r = gamepad1.right_trigger;

        Holonomic.arcade(y, x, l, r, robot);

        if(gamepad1.a) {
            encoderVal = 0;
        }

        if(gamepad1.x) {
            run();
        }

        telemetry.addData("Loop, enc val ", "L%d & R%d", encoderValFLeft, encoderValFRight);
    }

    private void resetEncoders() {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(robot.frontLeft.getCurrentPosition() != 0) {}

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void run() {
        while((Math.abs(encoderValFLeft) + Math.abs(encoderValFRight)/2 < Math.abs(encoderVal))) {
            Holonomic.move(Holonomic.Direction.FORWARD, .5, robot);
            telemetry.addData("Running, enc val ", "L%d & R%d", encoderValFLeft, encoderValFRight);
            telemetry.update();
        }
        Holonomic.stop(robot);
    }
}
