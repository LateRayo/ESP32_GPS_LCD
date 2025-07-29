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

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Uso: ./sat_logger <nombre_archivo.csv>" << endl;
        return 1;
    }

    signal(SIGINT, signalHandler);

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
    outfile << "time,prn,elevation,azimuth,snr" << endl;

    string buffer;
    char c;
    string last_time = "";

    while (keepRunning) {
        buffer.clear();
        while (read(serial_port, &c, 1) == 1) {
            if (c == '\n') break;
            if (c != '\r') buffer += c;
        }

        if (buffer.find("$GPGSV") == 0) {
            int total_msgs = stoi(getField(buffer, 1));
            int msg_num = stoi(getField(buffer, 2));
            int sats_in_view = stoi(getField(buffer, 3));

            // Usamos hora aproximada de sistema para referencia temporal
            time_t now = time(nullptr);
            tm* tm_info = localtime(&now);
            char time_str[9];
            strftime(time_str, 9, "%H%M%S", tm_info);

            for (int i = 4; i + 3 <= 16 && i + 3 < buffer.length(); i += 4) {
                std::string prn = getField(buffer, i);
                std::string elev = getField(buffer, i + 1);
                std::string azim = getField(buffer, i + 2);
                std::string snr  = getField(buffer, i + 3);

                if (!prn.empty()) {
                    outfile << time_str << ","
                            << prn << ","
                            << elev << ","
                            << azim << ","
                            << snr << endl;

                    cout << "[" << time_str << "] PRN: " << prn
                         << ", Elev: " << elev << "°"
                         << ", Azim: " << azim << "°"
                         << ", SNR: " << snr << endl;
                }
            }
        }
    }

    close(serial_port);
    outfile.close();
    return 0;
}
