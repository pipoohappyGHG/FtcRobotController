package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DerekJr2 (Blocks to Java)")
public class DerekJr extends LinearOpMode {

  private DcMotor frontleftMotor;
  private DcMotor backrightMotor;
  private DcMotor backleftMotor;
  private DcMotor frontrightMotor;
  private DcMotor beltMotor;
  private DcMotor shootMotor;
  private Servo kickServo;
  private Servo angleServo;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    double leftY;
    double rightY;
    double leftX;
    double denominator;
    double frontLeftpower;
    double backLeftpower;
    double frontRightpower;
    double backRightpower;

    frontleftMotor = hardwareMap.get(DcMotor.class, "frontleftMotor");
    backrightMotor = hardwareMap.get(DcMotor.class, "backrightMotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    frontrightMotor = hardwareMap.get(DcMotor.class, "frontrightMotor");
    beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
    shootMotor = hardwareMap.get(DcMotor.class, "shootMotor");
    kickServo = hardwareMap.get(Servo.class, "kickServo");
    angleServo = hardwareMap.get(Servo.class, "angleServo");

    waitForStart();
    frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
    backrightMotor.setDirection(DcMotor.Direction.REVERSE);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        leftY = gamepad1.left_stick_y * 1.1;
        rightY = gamepad1.right_stick_y * 1.1;
        leftX = (gamepad1.right_trigger + -gamepad1.left_trigger) * 1.1;
        denominator = Math.max(Math.abs(leftX) + Math.abs(leftY), Math.max(Math.abs(leftX) + Math.abs(rightY), 1));
        frontLeftpower = (leftY - leftX) / denominator;
        backLeftpower = (leftY + leftX) / denominator;
        frontRightpower = (rightY + leftX) / denominator;
        backRightpower = (rightY - leftX) / denominator;
        frontleftMotor.setPower(frontLeftpower);
        backleftMotor.setPower(backLeftpower);
        frontrightMotor.setPower(frontRightpower);
        backrightMotor.setPower(backRightpower);
        beltMotor.setPower(gamepad2.left_trigger);
        if (gamepad2.right_bumper) {
          shootMotor.setPower(0.9);
        } else {
          shootMotor.setPower(0);
        }
        if (gamepad2.left_bumper) {
          beltMotor.setPower(-1);
          shootMotor.setPower(-0.5);
        }
        if (gamepad2.square) {
          kickServo.setPosition(1);
        }
        if (gamepad2.squareWasReleased()) {
          kickServo.setPosition(0);
        }
        if (gamepad2.dpad_up) {
          angleServo.setPosition(0.4);
        }
        if (gamepad2.dpad_down) {
          angleServo.setPosition(0.8);
        }
      }
    }
  }
}
