#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <csignal>

using namespace std;

bool keepRunning = true;

void signalHandler(int signum) {
    keepRunning = false;
    cout << "\nTerminando escritura y cerrando archivo..." << endl;
}

std::string getField(const std::string& sentence, int index) {
    std::stringstream ss(sentence);
    std::string item;
    for (int i = 0; i <= index; ++i) {
        if (!getline(ss, item, ',')) return "";
    }
    return item;
}

float parseNMEACoordinate(const std::string& field, const std::string& direction) {
    if (field.empty()) return 0.0;
    float deg = std::stof(field.substr(0, field.find('.') - 2));
    float min = std::stof(field.substr(field.find('.') - 2));
    float coord = deg + (min / 60.0);
    if (direction == "S" || direction == "W") coord = -coord;
    return coord;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Uso: ./gps_logger <nombre_archivo.csv>" << endl;
        return 1;
    }

    signal(SIGINT, signalHandler);  // Captura Ctrl+C

    const char* port = "/dev/ttyUSB0"; // Cambiar si es necesario
    int serial_port = open(port, O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_port < 0) {
        perror("Error abriendo el puerto serie");
        return 1;
    }

    struct termios tty;
    tcgetattr(serial_port, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tcsetattr(serial_port, TCSANOW, &tty);

    ofstream outfile(argv[1]);
    outfile << "time,latitude,longitude,altitude,satellites,speed_kmh" << endl;

    string buffer;
    char c;
    string gga_data, rmc_data;

    while (keepRunning) {
        buffer.clear();
        while (read(serial_port, &c, 1) == 1) {
            if (c == '\n') break;
            if (c != '\r') buffer += c;
        }

        if (buffer.find("$GPGGA") == 0) {
            gga_data = buffer;
        } else if (buffer.find("$GPRMC") == 0) {
            rmc_data = buffer;
        }

        if (!gga_data.empty() && !rmc_data.empty()) {
            std::string time = getField(rmc_data, 1);
            std::string lat = getField(gga_data, 2);
            std::string lat_dir = getField(gga_data, 3);
            std::string lon = getField(gga_data, 4);
            std::string lon_dir = getField(gga_data, 5);
            std::string alt = getField(gga_data, 9);
            std::string sats = getField(gga_data, 7);
            std::string spd = getField(rmc_data, 7);

            float latitude = parseNMEACoordinate(lat, lat_dir);
            float longitude = parseNMEACoordinate(lon, lon_dir);
            float speed = spd.empty() ? 0.0 : std::stof(spd) * 1.852; // Nudos a km/h

            outfile << time << ","
                    << std::fixed << std::setprecision(6) << latitude << ","
                    << std::fixed << std::setprecision(6) << longitude << ","
                    << alt << ","
                    << sats << ","
                    << std::fixed << std::setprecision(2) << speed << endl;

            cout << "[" << time << "] "
                 << "Lat: " << latitude << ", Lon: " << longitude
                 << ", Alt: " << alt << "m, Satélites: " << sats
                 << ", Velocidad: " << speed << " km/h" << endl;

            gga_data.clear();
            rmc_data.clear();
        }
    }

    close(serial_port);
    outfile.close();
    return 0;
}
