package org.firstinspires.ftc.teamcode.FtcHub;

public class DeviceRepFile {
    DeviceRepFile (String type, String name, short port, short line) {
        this.name = name;
        this.port = port;
        this.type = type;
        this.line = line;
    }

    String name;
    String type;
    short port;

    short line;
}
