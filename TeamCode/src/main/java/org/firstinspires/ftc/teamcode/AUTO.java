package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.commands.core.LynxIsMotorAtTargetResponse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "AUTO (Blocks to Java)", preselectTeleOp = "Derek Jr.")
@Disabled
public class AUTO extends LinearOpMode {

  private DcMotor leftshooterMotor;
  private DcMotor rightshooterMotor;
  private DcMotor centershooterMotor;
  private DcMotor beltMotor;
  private DcMotor backleftMotor;
  private DcMotor frontrightMotor;
  private DcMotor backrightMotor;
  private DcMotor frontleftMotor;

  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;

  /**
   * Describe this function...
   */
  private void GPP__ID_21_() {
    USE_WEBCAM = false;
    backwards(0.7, 1850);
    sleep(400);
    Turn_Left_3();
    sleep(400);
    collect_1();
    sleep(400);
    backwards(0.7, 2000);
    sleep(400);
    Turn_Right_2();
    sleep(400);
    forwards(0.7, 2300);
  }

  /**
   * Describe this function...
   */
  private void PGP__ID_22_() {
    USE_WEBCAM = false;
    backwards(0.7, 800);
    sleep(400);
    Turn_Left_3();
    sleep(400);
    collect_1();
    sleep(400);
    backwards(0.7, 2000);
    Turn_Right();
    sleep(400);
    forwards(0.7, 1000);
    sleep(400);
    Turn_Left_2();
    sleep(400);
    shoot_3_close();
  }

  /**
   * Describe this function...
   */
  private void PPG__ID_23_() {
    USE_WEBCAM = false;
    straight_2();
    sleep(300);
    Turn_Left_2();
    sleep(400);
    collect_1();
    sleep(400);
    backwards(0.7, 2000);
    sleep(400);
    Turn_Right();
    sleep(400);
    shoot_3_close();
  }

  /**
   * Describe this function...
   */
  private void shoot_3_close() {
    leftshooterMotor.setPower(0.7);
    rightshooterMotor.setPower(0.7);
    centershooterMotor.setPower(0.5);
    sleep(400);
    beltMotor.setPower(1);
    sleep(6000);
    centershooterMotor.setPower(0);
    rightshooterMotor.setPower(0);
    leftshooterMotor.setPower(0);
    beltMotor.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void collect_1() {
    beltMotor.setPower(1);
    sleep(500);
    straight_1 (0.7, 2500);
    sleep(500);
    beltMotor.setPower(0);
  }

  /**
   * This OpMode illustrates the basics of AprilTag recognition and pose estimation.
   *
   * For an introduction to AprilTags, see the FTC-DOCS link below:
   * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
   *
   * In this sample, any visible tag ID will be detected and
   * displayed, but only tags that are included in the default
   * "TagLibrary" will have their position and orientation information displayed. This default TagLibrary contains
   * the current Season's AprilTags and a small set of "test Tags" in the high number range.
   *
   * When an AprilTag in the TagLibrary is detected, the SDK provides
   * location and orientation of the tag, relative to the camera.
   * This information is provided in the "ftcPose" member of the returned
   * "detection", and is explained in the ftc-docs page linked below.
   * https://ftc-docs.firstinspires.org/apriltag-detection-values
   */
  @Override
  public void runOpMode() {
    leftshooterMotor = hardwareMap.get(DcMotor.class, "leftshooterMotor");
    rightshooterMotor = hardwareMap.get(DcMotor.class, "rightshooterMotor");
    centershooterMotor = hardwareMap.get(DcMotor.class, "centershooterMotor");
    beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
    backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
    frontrightMotor = hardwareMap.get(DcMotor.class, "frontrightMotor");
    backrightMotor = hardwareMap.get(DcMotor.class, "backrightMotor");
    frontleftMotor = hardwareMap.get(DcMotor.class, "frontleftMotor");

    // Initialize AprilTag before waitForStart.
    initAprilTag();
    USE_WEBCAM = true;
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch START to start OpMode");
    backleftMotor.setDirection(DcMotor.Direction.REVERSE);
    frontrightMotor.setDirection(DcMotor.Direction.REVERSE);
    rightshooterMotor.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    telemetry.update();
    if (opModeIsActive()) {
      // Put run blocks here.
      backwards(0.7, 2500);
      sleep(500);
      shoot_3_close();
      sleep(500);
      Turn_Right();
      telemetry.update();
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetryAprilTag();
        // Push telemetry to the Driver Station.
        telemetry.update();
        // Share the CPU.
        sleep(20);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void backwards(double Speed, int Target_pos) {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(-1 * Target_pos);
    backrightMotor.setTargetPosition(-1 * Target_pos);
    frontleftMotor.setTargetPosition(-1 * Target_pos);
    frontrightMotor.setTargetPosition(-1 * Target_pos);
    backleftMotor.setPower(Speed);
    backrightMotor.setPower(Speed);
    frontleftMotor.setPower(Speed);
    frontrightMotor.setPower(Speed);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void Turn_Left_1(double Speed, int Target_pos) {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(-1 * Target_pos);
    backrightMotor.setTargetPosition(1 * Target_pos);
    frontleftMotor.setTargetPosition(-1 * Target_pos);
    frontrightMotor.setTargetPosition(1 * Target_pos);
    backleftMotor.setPower(Speed);
    backrightMotor.setPower(Speed);
    frontleftMotor.setPower(Speed);
    frontrightMotor.setPower(Speed);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void straight_1(double Speed, int Target_pos) {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(1500);
    backrightMotor.setTargetPosition(1500);
    frontleftMotor.setTargetPosition(1500);
    frontrightMotor.setTargetPosition(1500);
    backleftMotor.setPower(0.25);
    backrightMotor.setPower(0.25);
    frontleftMotor.setPower(0.25);
    frontrightMotor.setPower(0.25);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void straight_3() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(700);
    backrightMotor.setTargetPosition(700);
    frontleftMotor.setTargetPosition(700);
    frontrightMotor.setTargetPosition(700);
    backleftMotor.setPower(0.5);
    backrightMotor.setPower(0.5);
    frontleftMotor.setPower(0.5);
    frontrightMotor.setPower(0.5);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void forwards(double Speed, int Target_pos) {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(Target_pos);
    backrightMotor.setTargetPosition(Target_pos);
    frontleftMotor.setTargetPosition(Target_pos);
    frontrightMotor.setTargetPosition(Target_pos);
    backleftMotor.setPower(Speed);
    backrightMotor.setPower(Speed);
    frontleftMotor.setPower(Speed);
    frontrightMotor.setPower(Speed);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    VisionPortal myVisionPortal;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("Low ");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      } else {
      }
      if (myAprilTagDetection.id == 21) {
        GPP__ID_21_();
      }
      if (myAprilTagDetection.id == 22) {
        PGP__ID_22_();
      }
      if (myAprilTagDetection.id == 23) {
        PPG__ID_23_();
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

  /**
   * Describe this function...
   */
  private void Turn_Right() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(600);
    backrightMotor.setTargetPosition(-600);
    frontleftMotor.setTargetPosition(600);
    frontrightMotor.setTargetPosition(-600);
    backleftMotor.setPower(0.5);
    backrightMotor.setPower(0.5);
    frontleftMotor.setPower(0.5);
    frontrightMotor.setPower(0.5);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void straight_2() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(400);
    backrightMotor.setTargetPosition(400);
    frontleftMotor.setTargetPosition(400);
    frontrightMotor.setTargetPosition(400);
    backleftMotor.setPower(0.7);
    backrightMotor.setPower(0.7);
    frontleftMotor.setPower(0.7);
    frontrightMotor.setPower(0.7);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void Turn_Left_3() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(-975);
    backrightMotor.setTargetPosition(975);
    frontleftMotor.setTargetPosition(-975);
    frontrightMotor.setTargetPosition(975);
    backleftMotor.setPower(0.5);
    backrightMotor.setPower(0.5);
    frontleftMotor.setPower(0.5);
    frontrightMotor.setPower(0.5);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void Turn_Left_2() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(-700);
    backrightMotor.setTargetPosition(700);
    frontleftMotor.setTargetPosition(-700);
    frontrightMotor.setTargetPosition(700);
    backleftMotor.setPower(0.7);
    backrightMotor.setPower(0.7);
    frontleftMotor.setPower(0.7);
    frontrightMotor.setPower(0.7);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }

  /**
   * Describe this function...
   */
  private void Turn_Right_2() {
    backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontrightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleftMotor.setTargetPosition(800);
    backrightMotor.setTargetPosition(-800);
    frontleftMotor.setTargetPosition(800);
    frontrightMotor.setTargetPosition(-800);
    backleftMotor.setPower(0.5);
    backrightMotor.setPower(0.5);
    frontleftMotor.setPower(0.5);
    frontrightMotor.setPower(0.5);
    backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (opModeIsActive() && backrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontleftMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && frontrightMotor.isBusy()) {
      telemetry.update();
    }
    while (opModeIsActive() && backleftMotor.isBusy()) {
      telemetry.update();
      break;
    }
  }
}
