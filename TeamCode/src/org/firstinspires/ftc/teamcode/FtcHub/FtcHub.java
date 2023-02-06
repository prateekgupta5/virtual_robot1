package org.firstinspires.ftc.teamcode.FtcHub;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.*;
import java.util.Vector;

public class FtcHub {
    FtcHub(String configFile, HardwareMap hardwareMap) throws IOException {
        this.configFile = "/sdcard/FIRST/" + configFile;
        devices = parseFile();
        this.hardwareMap = hardwareMap;
    }

    HardwareMap hardwareMap;
    final String configFile;
    Vector<DeviceRepFile>[] devices;

    public Vector<DeviceRepFile>[] getDevices() {
        return devices;
    }

    enum Edit {
        TYPE,
        NAME
    }

    enum Hub {
        CONTROL_HUB,
        EXPANSION_HUB
    }

    enum PortType {
        MOTOR,
        SENSOR,
        SERVO
    }

    public Vector[] parseFile() throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(configFile)); //file i
        String line; //line currently under scrutiny
        short hubNum = 0; //expansion is 0, control is 1
        Vector<DeviceRepFile>[] devices = new Vector[]{ //storage of device name, type, port with line in file
                new Vector<>(),
                new Vector<>()
        };

        //parsing the file line by line
        for (int i = 0; (line = br.readLine()) != null; i++) {
            if (line.contains("/LynxModule")) {
                hubNum++;
            } //switch hubs

            else if (line.contains("            ")) { //if the line is indented enough to be a device
                devices[hubNum].add(new DeviceRepFile(
                        line.substring(13, line.indexOf(" n")), //type
                        line.substring(line.indexOf("name=\"") + 5, line.indexOf(" p")), //name
                        (short) Integer.parseInt(line.substring(line.indexOf("port=\"") + 6, line.indexOf("port=\"") + 7)), //port
                        (short) i //line
                ));
            }
        }

        br.close(); //remember to close your boxes :)
        return devices;
    }

    private void internalEditFile(DeviceRepFile device, Edit edit, String diff) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(configFile));
        String line; //general container
        Vector<String> file = new Vector<>(); //stores file as a list of lines

        while ((line = br.readLine()) != null) {
            file.add(line);
        } //fetches file into container

        if (edit == Edit.NAME) {
            int a = file.get(device.line).indexOf("name=\"") + 5; //figure out offset for where the name is
            line = file.get(device.line).substring(0, a); //append line prior to name into final
            line += '\"' + diff + '\"'; //append new name into final
            line += file.get(device.line).substring(a + device.name.length()); //append line post to name into final
            file.set(device.line, line); //overwrite old line with new line
        } else {
            line = file.get(device.line).substring(0, 13); //get part prior to type
            line += diff; //add new type
            line += file.get(device.line).substring(13 + device.type.length()); //add part after type
            file.set(device.line, line); //overwite  old line
        }

        line = ""; //clear temp

        for (String fileLine : file) {
            line += fileLine + '\n';
        } //bunch file into single string

        BufferedWriter bw = new BufferedWriter(new FileWriter(configFile)); //flush the file and prep to write

        bw.append(line); //write new file to buffer
        bw.flush(); //flush buffer into file

        //close your tupperware :)
        bw.close();
        br.close();
    }

    public DeviceRepFile getDeviceRep(short port, PortType type) {
        for (int i = 0; i < 2; i++) {
            for (DeviceRepFile device : devices[i]) {
                if(type == PortType.MOTOR && (device.type == "goBILDA5202SeriesMotor") && device.port == port) { //add other types for motors
                    return device;
                }
                else if (type == PortType.SENSOR && (device.type == "PLACEHOLDER") && device.port == port) { //add types for sensors
                    return device;
                }
                else if (( device.type == "Servo") && device.port == port) { //add other types for servos
                    return device;
                }
            }
        }

        return null; //if no device found, return null
    }

    private void internalChangeDeviceName (short port, PortType type, String name) throws IOException {
        internalEditFile(getDeviceRep(port, type), Edit.NAME, name);
    }

    public void changeMotorName (short port, String name) throws IOException {
        internalChangeDeviceName(port, PortType.MOTOR, name);
    }

    public void changeSensorName (short port, String name) throws IOException {
        internalChangeDeviceName(port, PortType.SENSOR, name);
    }

    public void changeServoName (short port, String name) throws IOException {
        internalChangeDeviceName(port, PortType.SERVO, name);
    }

    public <T> T getDevice (short port, PortType type, Class<? extends T> deviceType) {
        return hardwareMap.get(deviceType, getDeviceRep(port, type).name);
    }
}