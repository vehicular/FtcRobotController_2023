package org.firstinspires.ftc.teamcode.core.subsystems;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.Gamepad;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.util.Constants;


public class ChassisMecanum extends Subsystem {

    int constru = 0;

    boolean isAutonomous = false;

    private DcMotor backLeft = null; private DcMotor backRight = null;
    private DcMotor frontLeft = null; private DcMotor frontRight = null;

    //private ModernRoboticsI2cGyro myGyro= null;// Additional Gyro device
    //private GyroSensor sensorGyro=null;
    private BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    /** The colorSensor field will contain a reference to our color sensor hardware object */
    private NormalizedColorSensor colorSensor;

    private HardwareMap hardMap = null;
    private Telemetry telemetry = null;

    private ElapsedTime runt4 = new ElapsedTime(); //button to reverse the robot face


    private double speedmodifier=1;
    private boolean inv=false;


    private static final double DRIVE_GEAR_REDUCTION = 1;
    private static final double TICKS_PER_REV=537.7; // eg: GoBILDA 312 RPM Yellow Jacket;
    private static final double WHEEL_DIAMETER_INCHES=4;
    private static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    //private MediaPlayer hatz= new MediaPlayer();

    //int zAcummulated,
    int heading,xVal,yVal,zVal;

    double LFMotorMultiplier = 0.5;
    double RFMotorMultiplier = 0.5;
    double LBMotorMultiplier = 0.5;
    double RBMotorMultiplier = 0.5;

    public ChassisMecanum(HardwareMap hardwareMap, boolean isTankDrive)
    {
        super(hardwareMap);
        frontLeft = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        constru++;
        if (isAutonomous) {
            //sleep(250);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //if autonomous
        } else {
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        imu = hardwareMap.get(BNO055IMU.class, Constants.imu);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    /**
     * Drives the robot with a basic tank drive.
     *
     * @param gamepad1
     * @param gamepad2
     */
    private final double TRIGGER_THRESHOLD = 0.75;
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            SlowControl(gamepad1); // robot coordinate
        }
        else {
            FastControl(gamepad1); // field coordinate
        }
    }

    private void FastControl(Gamepad gamepad)
    {
        double x = -gamepad.left_stick_x; // Remember, this is reversed!
        double y = gamepad.left_stick_y * 1.1; // Counteract imperfect strafing
        double rx = -gamepad.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        //the translation joystick values need to be rotated by the robot heading
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower*LFMotorMultiplier);
        backLeft.setPower(backLeftPower*LBMotorMultiplier);
        frontRight.setPower(frontRightPower*RFMotorMultiplier);
        backRight.setPower(backRightPower*RBMotorMultiplier);
    }

    private void SlowControl(Gamepad gamepad)
    {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower*0.2);
        backLeft.setPower(backLeftPower*0.2);
        frontRight.setPower(frontRightPower*0.2);
        backRight.setPower(backRightPower*0.2);
    }

    /**
     * Sets power to motors to zero. Always call when done using this class for safety.
     */
    @Override
    public void stop()
    {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    @Override
    public void autoInit() {

    }

    @Override
    public void teleopInit() {

    }

    /**
     * Called during op modes to provide information aobut this subsystem.
     *
     * @return Returns the text to add to the bottom of the  driver station.
     */
    @Override
    public String addTelemetry()
    {
        String s = "Chassis \n";
//        s+= "[Left Drive]" + leftDrive.toString() + "\t[Right Drive: " +rightDrive.toString();
        //   s+= "Heading: " +gyro.getAngularOrientation().toString();
//        s+= "Displacement: " + gyro.getPosition().toString();
        //s += "Velocity " + gyro.getVelocity().toString();
        s += "left Front Wheel: " + frontLeft.getCurrentPosition() + "\n";
        s += "Left Back Wheel: " + backLeft.getCurrentPosition();
        s += "Right Front Wheel: " + frontRight.getCurrentPosition() + "\n";
        s += "Right Back Wheel: " + backRight.getCurrentPosition() + "\n";
        s += "constructor ran #: " + constru;

        return s;
    }

    public void analog_control(double x, double y, double rotation,boolean key_invert,boolean bmp1,boolean bmp2,boolean dpad1,boolean dpad2,boolean dpad3,boolean dpad4) {

        if(key_invert&&runt4.milliseconds()>=500) {
            inv=!inv;
            runt4.reset();
        }
        if(bmp1) {
            speedmodifier=.5;
        }else if(bmp2){
            speedmodifier=1;
        }
        double wheelSpeed[] = new double[4];

        wheelSpeed[0] = (-x + y + rotation); //bl
        wheelSpeed[1] = (x + y - rotation);//br
        wheelSpeed[2] = (x + y + rotation);//fl
        wheelSpeed[3] = (-x + y - rotation);//fr

        if(dpad1) { //left
            wheelSpeed[0] =  .5;//bl
            wheelSpeed[1] = -.5;//br
            wheelSpeed[2] = -.5;//fl
            wheelSpeed[3] = .5;//fr
        } else if(dpad2) { //right
            wheelSpeed[0] =  -.5;//bl
            wheelSpeed[1] = .5;//br
            wheelSpeed[2] = .5;//fl
            wheelSpeed[3] = -.5;//fr
        }else if(dpad3){ //down
            wheelSpeed[0] = -.5;
            wheelSpeed[1] = -.5;
            wheelSpeed[2] = -.5;
            wheelSpeed[3] = -.5;
        }else if(dpad4){ //up
            wheelSpeed[0] = .5;
            wheelSpeed[1] = .5;
            wheelSpeed[2] = .5;
            wheelSpeed[3] = .5;
        }
        wheelSpeed[0]*=speedmodifier;
        wheelSpeed[1]*=speedmodifier;
        wheelSpeed[2]*=speedmodifier;
        wheelSpeed[3]*=speedmodifier;
        if(!inv) {
            backLeft.setPower(Range.clip(wheelSpeed[0],-1,1));
            backRight.setPower(Range.clip(wheelSpeed[1],-1,1));
            frontLeft.setPower(Range.clip(wheelSpeed[2],-1,1));
            frontRight.setPower(Range.clip(wheelSpeed[3],-1,1));
        } else {
            backLeft.setPower(Range.clip(-wheelSpeed[3],-1,1));
            backRight.setPower(Range.clip(-wheelSpeed[2],-1,1));
            frontLeft.setPower(Range.clip(-wheelSpeed[1],-1,1));
            frontRight.setPower(Range.clip(-wheelSpeed[0],-1,1));
        }
    }


    public  void TargetDrive(double backLeftInches, double backRightInches, double frontLeftInches,
                              double frontRightInches, double maxSpeed, int timeoutInSeconds)  {


        int a, b, c, d;

        a = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
        b = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
        c = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
        d = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);


        frontRight.setTargetPosition(a);
        frontLeft.setTargetPosition(b);
        backRight.setTargetPosition(c);
        backLeft.setTargetPosition(d);

        ElapsedTime runtime = new ElapsedTime();

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(maxSpeed);
        backLeft.setPower(maxSpeed);
        backRight.setPower(maxSpeed);
        frontRight.setPower(maxSpeed);


        while ((runtime.seconds() < timeoutInSeconds) && (backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy() || frontLeft.isBusy())) {
            // Display it for the driver.

        }


        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public  void DriveStraight(double Inches, double maxSpeed, int timeoutInSeconds, double target)  {


            int a, b, c, d;
            a = frontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            b = frontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            c = backRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            d = backLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);


            frontRight.setTargetPosition(a);
            frontLeft.setTargetPosition(b);
            backRight.setTargetPosition(c);
            backLeft.setTargetPosition(d);


            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ElapsedTime runtime = new ElapsedTime();
            while ((runtime.seconds() < timeoutInSeconds) && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy()) {
                // Display it for the driver.

                if(Inches>0) {
                    frontLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                    backLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                    backRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                    frontRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                } else {
                    frontLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                    backLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                    backRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                    frontRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                }
            }


            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);


          //  motor_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // motor_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    public  void DriveStrafe(double Inches,double maxSpeed, int timeoutInSeconds, double target)  {//mecanum dreapta default

        int a, b, c, d;

        a = frontRight.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        b = frontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        c = backRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        d = backLeft.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);


        frontRight.setTargetPosition(a);
        frontLeft.setTargetPosition(b);
        backRight.setTargetPosition(c);
        backLeft.setTargetPosition(d);


        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutInSeconds) && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy()) {
            // Display it for the driver.

            if(Inches<0) {
                frontLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
            }else {
                frontLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
            }
        }


        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);


        //motor_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public  void FirstDiagonalDrive(double Inches,double maxSpeed, int timeoutInSeconds, double target)  {

        int b, c;

        b = frontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        c = backRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(b);
        backRight.setTargetPosition(c);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutInSeconds) && backRight.isBusy() && frontLeft.isBusy()) {
            // Display it for the driver.

            if(Inches>0) {
                frontLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
            }else {
                frontLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
            }
        }
        frontLeft.setPower(0);
        backRight.setPower(0);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public  void SecondDiagonalDrive(double Inches,double maxSpeed, int timeoutInSeconds, double target)  {

        int b, c;

        b = frontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        c = backLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);

        frontRight.setTargetPosition(b);
        backLeft.setTargetPosition(c);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElapsedTime runtime = new ElapsedTime();
        while ((runtime.seconds() < timeoutInSeconds) && backLeft.isBusy() && frontRight.isBusy()) {
            // Display it for the driver.

            if(Inches>0) {
                frontRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
            }else {
                frontRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
            }
        }
        frontRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void debug() {
        telemetry.addData("Wheels Encoders:"," ");
        telemetry.addData("motor_bl", backLeft.getCurrentPosition());
        telemetry.addData("motor_br", backRight.getCurrentPosition());
        telemetry.addData("motor_fl", frontLeft.getCurrentPosition());
        telemetry.addData("motor_fr", frontRight.getCurrentPosition());
        telemetry.addData("motor_bl power", backLeft.getPower());
        telemetry.addData("motor_br pw", backRight.getPower());
        telemetry.addData("motor_fl pw", frontLeft.getPower());
        telemetry.addData("motor_fr pw ", frontRight.getPower());
    }

    public void play_john(boolean bt1,boolean bt2) {
        /*if(bt1) hatz.start();
        else if(bt2)
        {
            hatz.pause();
            hatz.seekTo(0);
            hatz.start();
        }*/
    }

}


