package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract hardware class that defines required/useful universal methods
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 10/21/16
 */
public abstract class Hardware {

    private HardwareMap hardwareMap;
    public TelemetryArrayList telemetry;

    public static final String VUFORIA_LICENSE_KEY = "AdYQ1UT/////AAAAGduICslDnEnttzNkRGI2spxKjPBtdo/7cWrldv0MqHAmbK0Fyjw65zsW4JCkN6GRGjkwkLLX4kMkfjY2j/7K9K74AA1RRn1QaxpqfHqWfPXudWPzt4Y3PaLHK5c6ge6m6PyDYTZMxZmgb2jS5JaR0KPUf8Vmu1ysEOZfSNcSC20G56QRO/9VpJRrfetFMlsDiAwmsj+muYdKN5fwRCW3N8KK7CVD2ad9mXKvv45082O9PL0zXxq1vYPeKmn/27V1ihKOI0JHL5vEIeN3XeA56SM1f7yiLk2LFmkY+sM6K+HaDL+wLIulcuUIidqZ0xwKFFHCPjssVaZ25RtHYUY4nIvS+LJdzO+FdTYNDqtOn95Q";

    /**
     * Constructor initializes hardware map
     *
     * @param hardwareMap robot's hardware map
     */
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        telemetry = new TelemetryArrayList();
        initializeHardwareMap();
        initializeRobot();
    }

    /**
     * Initializes all the hardware on the robot
     */
    public abstract void initializeRobot();

    /**
     * Short way of initializing hardware. Example:
     * leftMotor = getDevice(dcMotor, "lf");
     *
     * @param deviceMapping device type
     * @param name          name in configuration file
     * @return device, or null if not found
     */
    public HardwareDevice getDevice(HardwareMap.DeviceMapping<DcMotor> deviceMapping, String name) {
        try {
            return deviceMapping.get(name);
        } catch (Exception e) {
            DbgLog.error("Device not found: " + name);
            return null;
        }
    }

    public void addTelemetry(String key, String data) {

    }

    /**
     * Gets references from hardwareMap so that getDevice() is easier to use
     */
    private void initializeHardwareMap() {
        dcMotor = hardwareMap.dcMotor;
        servo = hardwareMap.servo;
        colorSensor = hardwareMap.colorSensor;
        touchSensor = hardwareMap.touchSensor;
        irSeekerSensor = hardwareMap.irSeekerSensor;
        accelerationSensor = hardwareMap.accelerationSensor;
        compassSensor = hardwareMap.compassSensor;
        crservo = hardwareMap.crservo;
        led = hardwareMap.led;
        lightSensor = hardwareMap.lightSensor;
        opticalDistanceSensor = hardwareMap.opticalDistanceSensor;
        analogInput = hardwareMap.analogInput;
        analogOutput = hardwareMap.analogOutput;
        dcMotorController = hardwareMap.dcMotorController;
        deviceInterfaceModule = hardwareMap.deviceInterfaceModule;
        digitalChannel = hardwareMap.digitalChannel;
        gyroSensor = hardwareMap.gyroSensor;
        legacyModule = hardwareMap.legacyModule;
        i2cDevice = hardwareMap.i2cDevice;
        i2cDeviceSynch = hardwareMap.i2cDeviceSynch;
        pwnOutput = hardwareMap.pwmOutput;
        servoController = hardwareMap.servoController;
        ultrasonicSensor = hardwareMap.ultrasonicSensor;
        voltageSensor = hardwareMap.voltageSensor;
        touchSensorMultiplexer = hardwareMap.touchSensorMultiplexer;
    }

    protected HardwareMap.DeviceMapping dcMotor;
    protected HardwareMap.DeviceMapping servo;
    protected HardwareMap.DeviceMapping colorSensor;
    protected HardwareMap.DeviceMapping touchSensor;
    protected HardwareMap.DeviceMapping irSeekerSensor;
    protected HardwareMap.DeviceMapping accelerationSensor;
    protected HardwareMap.DeviceMapping compassSensor;
    protected HardwareMap.DeviceMapping crservo;
    protected HardwareMap.DeviceMapping led;
    protected HardwareMap.DeviceMapping lightSensor;
    protected HardwareMap.DeviceMapping opticalDistanceSensor;
    protected HardwareMap.DeviceMapping analogInput;
    protected HardwareMap.DeviceMapping analogOutput;
    protected HardwareMap.DeviceMapping dcMotorController;
    protected HardwareMap.DeviceMapping deviceInterfaceModule;
    protected HardwareMap.DeviceMapping digitalChannel;
    protected HardwareMap.DeviceMapping gyroSensor;
    protected HardwareMap.DeviceMapping legacyModule;
    protected HardwareMap.DeviceMapping i2cDevice;
    protected HardwareMap.DeviceMapping i2cDeviceSynch;
    protected HardwareMap.DeviceMapping pwnOutput;
    protected HardwareMap.DeviceMapping servoController;
    protected HardwareMap.DeviceMapping ultrasonicSensor;
    protected HardwareMap.DeviceMapping voltageSensor;
    protected HardwareMap.DeviceMapping touchSensorMultiplexer;




    public class DcMotorWrapper {
        DcMotor motor;

        DcMotorWrapper(HardwareDevice motor) {
            try {
                this.motor = (DcMotor) motor;
            } catch (Exception e) {
                this.motor = null;
            }
        }

        public void setPower(double power) {
            if (motor != null) {
                power = Range.clip(power, -1, 1);
                motor.setPower(power);
            }
        }

        public void setDirection(DcMotor.Direction direction) {
            if (motor != null) {
                motor.setDirection(direction);
            }
        }

        public void setMode(DcMotor.RunMode runMode) {
            if (motor != null) {
                motor.setMode(runMode);
            }
        }

        public void setMaxSpeed(int encoderTicksPerSecond) {
            if (motor != null) {
                motor.setMaxSpeed(encoderTicksPerSecond);
            }
        }

        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            if (motor != null) {
                motor.setZeroPowerBehavior(zeroPowerBehavior);
            }
        }

        public void setTargetPosition(int position) {
            if (motor != null) {
                motor.setTargetPosition(position);
            }
        }

        public DcMotorController getController() {
            if (motor != null) {
                return motor.getController();
            }
            return null;
        }

        public int getMaxSpeed() {
            if (motor != null) {
                return motor.getMaxSpeed();
            }
            return 0;
        }

        public int getCurrentPosition() {
            if (motor != null) {
                return motor.getCurrentPosition();
            }
            return 0;
        }

        public DcMotor.RunMode getMode() {
            if (motor != null) {
                return motor.getMode();
            }
            return null;
        }

        public int getPortNumber() {
            if (motor != null) {
                return motor.getPortNumber();
            }
            return 0;
        }

        public int getTargetPosition() {
            if (motor != null) {
                return motor.getTargetPosition();
            }
            return 0;
        }

        public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
            if (motor != null) {
                return motor.getZeroPowerBehavior();
            }
            return null;
        }

        public boolean getPowerFloat() {
            if (motor != null) {
                return motor.getPowerFloat();
            }
            return false;
        }

        public DcMotor.Direction getDirection() {
            if (motor != null) {
                return motor.getDirection();
            }
            return null;
        }

        public String toString() {
            if (motor != null) {
                return motor.toString();
            }
            return null;
        }

        public String getConnectionInfo() {
            if (motor != null) {
                return motor.getConnectionInfo();
            }
            return null;
        }

        public boolean isBusy() {
            if (motor != null) {
                return motor.isBusy();
            }
            return false;
        }

        public String getDeviceName() {
            if (motor != null) {
                return motor.getDeviceName();
            }
            return null;
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (motor != null) {
                return motor.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (motor != null) {
                motor.resetDeviceConfigurationForOpMode();
            }
        }
    }

    public class ServoWrapper {
        Servo servo;

        ServoWrapper(HardwareDevice servo) {
            try {
                this.servo = (Servo) servo;
            } catch (Exception e) {
                this.servo = null;
            }
        }

        public void setPosition(double position) {
            if (servo != null) {
                position = Range.clip(position, 0, 1);
                servo.setPosition(position);
            }
        }

        public void setDirection(Servo.Direction direction) {
            if (servo != null) {
                servo.setDirection(direction);
            }
        }



        public int getPortNumber() {
            if (servo != null) {
                return servo.getPortNumber();
            }
            return 0;
        }



        public Servo.Direction getDirection() {
            if (servo != null) {
                return servo.getDirection();
            }
            return null;
        }

        public String toString() {
            if (servo != null) {
                return servo.toString();
            }
            return null;
        }

        public String getConnectionInfo() {
            if (servo != null) {
                return servo.getConnectionInfo();
            }
            return null;
        }


        public String getDeviceName() {
            if (servo != null) {
                return servo.getDeviceName();
            }
            return null;
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (servo != null) {
                return servo.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (servo != null) {
                servo.resetDeviceConfigurationForOpMode();
            }
        }

        public double getPosition(){
            if (servo != null){
                return servo.getPosition();
            }
            return -1;
        }

        public void scaleRange(double min, double max){
            if (servo != null){
                servo.scaleRange(min, max);
            }
        }


    }

    public class UltrasonicSensorWrapper {
        private UltrasonicSensor sensor;


        UltrasonicSensorWrapper(HardwareDevice device) {
            try {
                this.sensor = (UltrasonicSensor) device;
            } catch (Exception e) {
                this.sensor = null;
            }
        }





        public double getUltrasonicLevel() {
            if (sensor != null) {
                return sensor.getUltrasonicLevel();
            }
            return -1;
        }

        public String toString() {
            if (sensor != null) {
                return sensor.toString();
            }
            return null;
        }

        public String getConnectionInfo() {
            if (sensor != null) {
                return sensor.getConnectionInfo();
            }
            return null;
        }

        public int getVersion() {
            if (sensor != null) {
                return sensor.getVersion();
            }
            return 0;
        }

        public int hashCode() {
            if (sensor != null) {
                return sensor.hashCode();
            }
            return 0;
        }

        public String status() {
            if (sensor != null) {
                return sensor.status();
            }
            return "Ultrasonic sensor is null";
        }

        public String getDeviceName() {
            if (sensor != null) {
                return sensor.getDeviceName();
            }
            return "Ultrasonic sensor is null";
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (sensor != null) {
                return sensor.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (sensor != null) {
                sensor.resetDeviceConfigurationForOpMode();
            }
        }
    }

    public class ColorSensorWrapper{
        ColorSensor sensor;

        ColorSensorWrapper(HardwareDevice device) {
            try {
                this.sensor = (ColorSensor) device;
            } catch (Exception e) {
                this.sensor = null;
            }
        }

        public int alpha(){
            if (sensor != null) {
                return sensor.alpha();
            }
            return -1;
        }

        public int blue(){
            if (sensor != null) {
                return sensor.blue();
            }
            return -1;
        }

        public int red(){
            if (sensor != null) {
                return sensor.red();
            }
            return -1;
        }

        public int green(){
            if (sensor != null) {
                return sensor.green();
            }
            return -1;
        }

        public int argb(){
            if (sensor != null) {
                return sensor.argb();
            }
            return -1;
        }

        public void enableLed (boolean isLedEnabled){
            if (sensor != null) {
                sensor.enableLed(isLedEnabled);
            }
        }

        public I2cAddr getI2CAddress (boolean isLedEnabled){
            if (sensor != null) {
                sensor.getI2cAddress();
            }
            return null;
        }

        public void setI2CAddress (I2cAddr addr){
            if (sensor != null) {
                sensor.setI2cAddress(addr);
            }
        }


        public String toString() {
            if (sensor != null) {
                return sensor.toString();
            }
            return "Color sensor is null";
        }

        public String getConnectionInfo() {
            if (sensor != null) {
                return sensor.getConnectionInfo();
            }
            return null;
        }

        public int getVersion() {
            if (sensor != null) {
                return sensor.getVersion();
            }
            return 0;
        }

        public int hashCode() {
            if (sensor != null) {
                return sensor.hashCode();
            }
            return 0;
        }


        public String getDeviceName() {
            if (sensor != null) {
                return sensor.getDeviceName();
            }
            return "Color sensor is null";
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (sensor != null) {
                return sensor.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (sensor != null) {
                sensor.resetDeviceConfigurationForOpMode();
            }
        }


    }

    public class OpticalDistanceSensorWrapper{
        OpticalDistanceSensor sensor;

        OpticalDistanceSensorWrapper(HardwareDevice device) {
            try {
                this.sensor = (OpticalDistanceSensor) device;
            } catch (Exception e) {
                this.sensor = null;
            }
        }

        public double getLightDetected(){
            if (sensor != null) {
                return sensor.getLightDetected();
            }
            return -1;
        }

        public double getRawLightDetected(){
            if (sensor != null) {
                return sensor.getRawLightDetected();
            }
            return -1;
        }

        public double getRawLightDetectedMax(){
            if (sensor != null) {
                return sensor.getRawLightDetectedMax();
            }
            return -1;
        }

        public void enableLed (boolean isLedEnabled){
            if (sensor != null) {
                sensor.enableLed(isLedEnabled);
            }
        }

        public void close(){
            if (sensor != null) {
                sensor.close();
            }
        }

        public String status (){
            if (sensor != null) {
                return sensor.status();
            }
            return null;
        }




        public String toString() {
            if (sensor != null) {
                return sensor.toString();
            }
            return "Optical Distance sensor is null";
        }

        public String getConnectionInfo() {
            if (sensor != null) {
                return sensor.getConnectionInfo();
            }
            return null;
        }

        public int getVersion() {
            if (sensor != null) {
                return sensor.getVersion();
            }
            return 0;
        }

        public int hashCode() {
            if (sensor != null) {
                return sensor.hashCode();
            }
            return 0;
        }


        public String getDeviceName() {
            if (sensor != null) {
                return sensor.getDeviceName();
            }
            return "Optical Distance sensor is null";
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (sensor != null) {
                return sensor.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (sensor != null) {
                sensor.resetDeviceConfigurationForOpMode();
            }
        }


    }



    public class GyroSensorWrapper{
        GyroSensor sensor;
        int offset;

        GyroSensorWrapper(HardwareDevice device) {
            try {
                this.sensor = (GyroSensor) device;
            } catch (Exception e) {
                this.sensor = null;
            }
            offset = 0;
        }

        public void centerOffset(){
            offset = 0;
            offset = getHeading();
        }

        public int getHeading(){
            if (sensor != null) {
                int heading = sensor.getHeading() - offset;
                while (heading < 0){
                    heading += 360;
                }
                heading %= 360;
                return heading;
            }
            return 0;
        }

        public int rawX(){
            if (sensor != null) {
                return sensor.rawX();
            }
            return -1;
        }

        public int rawY(){
            if (sensor != null) {
                return sensor.rawY();
            }
            return -1;
        }

        public int rawZ(){
            if (sensor != null) {
                return sensor.rawZ();
            }
            return -1;
        }

        public double getRotationFraction(){
            if (sensor != null) {
                return sensor.getRotationFraction();
            }
            return -1;
        }

        public boolean isCalibrating(){
            if (sensor != null) {
                return sensor.isCalibrating();
            }
            return false;
        }

        public void calibrate (){
            if (sensor != null) {
                sensor.calibrate();
            }
        }

        public void resetZAxisIntegrator (){
            if (sensor != null) {
                sensor.resetZAxisIntegrator();
            }
        }

        public String status (){
            if (sensor != null) {
                return sensor.status();
            }
            return null;
        }


        public String toString() {
            if (sensor != null) {
                return sensor.toString();
            }
            return "Gyro sensor is null";
        }

        public String getConnectionInfo() {
            if (sensor != null) {
                return sensor.getConnectionInfo();
            }
            return "Gyro sensor is null";
        }

        public int getVersion() {
            if (sensor != null) {
                return sensor.getVersion();
            }
            return 0;
        }

        public int hashCode() {
            if (sensor != null) {
                return sensor.hashCode();
            }
            return 0;
        }


        public String getDeviceName() {
            if (sensor != null) {
                return sensor.getDeviceName();
            }
            return "Gyro sensor is null";
        }

        public HardwareDevice.Manufacturer getManufacturer() {
            if (sensor != null) {
                return sensor.getManufacturer();
            }
            return null;
        }

        public void resetDeviceConfigurationForOpMode() {
            if (sensor != null) {
                sensor.resetDeviceConfigurationForOpMode();
            }
        }


    }

    private class TelemetryArrayList {
        List<String> keys;
        List<String> msgs;

        TelemetryArrayList() {
            keys = new ArrayList<String>();
            msgs = new ArrayList<String>();
        }

        public void addData(String key, String msg) {
            for (int i = 0; i < keys.size(); i++) {
                if (keys.get(i).equals(key)) {
                    msgs.set(i, msg);
                    return;
                }
            }
            keys.add(key);
            msgs.add(msg);
        }

        public void clear() {
            keys.clear();
            msgs.clear();
        }


        /**
         * Returns two column 2D array with telemetry. 0th column is keys, 1st column is messages
         *
         * @return telemetry array
         */
        public String[][] getTelemetry() {
            String[][] telemetry = new String[2][keys.size()];
            for (int i = 0; i < telemetry[0].length; i++) {
                telemetry[0][i] = keys.get(i);
                telemetry[1][i] = msgs.get(i);
            }
            return telemetry;
        }
    }

    public static double findMedianOfDoubleList(List<Double> list) {
        if (list != null && list.size() > 0) {
            int middleIndex = list.size() / 2;
            if (list.size() % 2.0 == 0) {
                return (list.get(middleIndex) + list.get(middleIndex - 1)) / 2.0;
            } else {
                return list.get(middleIndex);
            }
        }
        return -1;
    }

    public static double power(double value, double exponent){
        boolean isNegative = value < 0;
        value = Math.abs(value);
        double result = Math.pow(value, exponent);
        if (isNegative){
            result *= -1;
        }
        return result;
    }

}