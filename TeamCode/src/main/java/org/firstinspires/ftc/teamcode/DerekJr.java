package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DerekJr (Blocks to Java)")
public class DerekJr extends LinearOpMode {

  private DcMotor rightshooterMotor;
  private DcMotor frontleftMotor;
  private DcMotor backrightMotor;
  private DcMotor backleftMotor;
  private DcMotor frontrightMotor;
  private DcMotor leftshooterMotor;
  private DcMotor beltMotor;
  private DcMotor centershooterMotor;

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

    rightshooterMotor = hardwareMap.get(DcMotor.class, "rightshooterMotor");
    frontleftMotor = hardwareMap.get(DcMotor.class, "frontleftMotor");
    backrightMotor = hardwareMap.get(DcMotor.class, "backrightMotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    frontrightMotor = hardwareMap.get(DcMotor.class, "frontrightMotor");
    leftshooterMotor = hardwareMap.get(DcMotor.class, "leftshooterMotor");
    beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
    centershooterMotor = hardwareMap.get(DcMotor.class, "centershooterMotor");

    waitForStart();
    rightshooterMotor.setDirection(DcMotor.Direction.REVERSE);
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
        leftshooterMotor.setPower(1 * gamepad2.right_trigger);
        rightshooterMotor.setPower(1 * gamepad2.right_trigger);
        beltMotor.setPower(gamepad2.left_trigger);
        if (gamepad2.square) {
          centershooterMotor.setPower(1);
        } else {
          centershooterMotor.setPower(0);
        }
        if (gamepad2.left_bumper) {
          beltMotor.setPower(-1);
        }
        if (gamepad2.right_bumper) {
          leftshooterMotor.setPower(0.8);
          rightshooterMotor.setPower(0.8);
        } else {
          leftshooterMotor.setPower(0);
          rightshooterMotor.setPower(0);
        }
      }
    }
  }
}
