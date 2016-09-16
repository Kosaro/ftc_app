package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract hardware class that defines required/useful universal methods
 *
 * @author Oscar Kosar-Kosarewicz
 * @version 9/13/16
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

    public class UltrasonicSensorWrapper {
        private UltrasonicSensor sensor;
        private CalculateUltrasonicMedianValue calcMedianThread;


        UltrasonicSensorWrapper(HardwareDevice device) {
            try {
                this.sensor = (UltrasonicSensor) device;
            } catch (Exception e) {
                this.sensor = null;
            }
        }

        private class CalculateUltrasonicMedianValue implements Runnable {
            int intervalMiliSeconds;
            ElapsedTime elapsedTime;
            List<Double> values;
            boolean isThreadStopped;
            double value;

            CalculateUltrasonicMedianValue(int intervalMiliSeconds) {
                this.intervalMiliSeconds = intervalMiliSeconds;
                elapsedTime = new ElapsedTime();
                values = new ArrayList<>();
                isThreadStopped = false;
                value = getUltrasonicLevel();
            }


            public void run() {
                elapsedTime.reset();
                while (!isThreadStopped) {
                    values.add(getUltrasonicLevel());
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (elapsedTime.milliseconds() > intervalMiliSeconds) {
                        value = findMedianOfDoubleList(values);
                        elapsedTime.reset();
                        values.clear();
                    }
                }
            }

            public double getValue() {
                return value;
            }

            public void stopThread() {
                isThreadStopped = true;

            }
        }


        public double getUltrasonicLevelMedian() {
            if (sensor != null) {
                if (calcMedianThread == null) {
                    calcMedianThread = new CalculateUltrasonicMedianValue(200);
                    calcMedianThread.run();
                }
                return calcMedianThread.getValue();
            }
            return -1;
        }

        public void stopMedianCalculatingThread(){
            if (sensor != null) {
                if (calcMedianThread != null) {
                    calcMedianThread.stopThread();
                }
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
}