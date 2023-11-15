#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <mosquitto.h>
#include <json-c/json.h>

/*-------------------------------------------------------------------------------------*/
#define WIFI_SERIAL_PORT "/dev/ttyUSB0"
#define BLUETOOTH_SERIAL_PORT "/dev/ttyUSB1"

// Định nghĩa SSID và mật khẩu WiFi
#define WIFI_SSID "WIFI_ESP8266"
#define WIFI_PASSWORD "12345678"

/*-------------------------------------------------------------------------------------*/
pthread_t wifi_thread, bluetooth_thread, mqtt_thread;
pthread_mutex_t wifi_mutex, bluetooth_mutex, mqtt_mutex;

int wifi_serial_port, bluetooth_serial_port;
struct termios tty_wifi, tty_bluetooth;

char wifi_buffer[256];
int wifi_bytes_read;

char response[256];
int bytes_read;

char bluetooth_buffer[256];
int bluetooth_bytes_read;

char value[256];

struct mosquitto* mosq;
    const char* host = "demo.thingsboard.io";   // "karolineserver.duckdns.org"  "demo.thingsboard.io"
    const char* username = "TTRGAltWlJTw8VaZ3n3i";  // "iotgateway"; "TTRGAltWlJTw8VaZ3n3i"
    const char* topic = "v1/devices/me/telemetry"; //"gateway"
char payload[256];

const char *filename = "file.txt";  // Tên của tệp văn bản đích

/*-------------------------------------------------------------------------------------*/
void send_at_command(int serial_port, const char* command);
void read_at_response(int serial_port);
void configure_access_point(const char* ssid, const char* password);
void configure_hc05_as_master();
void* wifi_task(void* arg);
void* bluetooth_task(void* arg);
void on_connect(struct mosquitto* mosq, void* obj, int rc);
void* mqtt_task(void* arg);
void init_uart(int* serial_port, struct termios* tty, const char* device, speed_t baud_rate);
void processJsonString(const char *jsonStr);
char* processJsonData(const char *jsonStr);
void writeJsonToFile(const char *jsonData, const char *filename);
void processDeviceData(const char *jsonStr);
const char *extractValueByKey(const char *json_string, const char *key);
void calculatePerformance(const char *jsonStr, const char *key);
void send_data_to_esp8266(const char* data, char* nodeID);
/*-------------------------------------------------------------------------------------*/
int main() {
    // Khởi tạo mutex
    pthread_mutex_init(&wifi_mutex, NULL);
    pthread_mutex_init(&bluetooth_mutex, NULL);
    pthread_mutex_init(&mqtt_mutex, NULL);

    // Khởi tạo UART cho WiFi
    init_uart(&wifi_serial_port, &tty_wifi, WIFI_SERIAL_PORT, B115200);

    // Khởi tạo UART cho Bluetooth
    init_uart(&bluetooth_serial_port, &tty_bluetooth, BLUETOOTH_SERIAL_PORT, B38400);

    // Cấu hình module
    configure_access_point(WIFI_SSID, WIFI_PASSWORD);
    configure_hc05_as_master();

    // Tạo luồng cho WiFi, Bluetooth, MQTT 
    if (pthread_create(&wifi_thread, NULL, wifi_task, NULL) != 0) {
        fprintf(stderr, "Error creating WiFi thread\n");
        return -1;
    }

    if (pthread_create(&bluetooth_thread, NULL, bluetooth_task, NULL) != 0) {
        fprintf(stderr, "Error creating Bluetooth thread\n");
        return -1;
    }

    if (pthread_create(&mqtt_thread, NULL, mqtt_task, NULL) != 0) {
        fprintf(stderr, "Error creating MQTT thread\n");
        return -1;
    }

    pthread_join(wifi_thread, NULL);
    pthread_join(bluetooth_thread, NULL);
    pthread_join(mqtt_thread, NULL);

    // Đóng cổng UART
    close(wifi_serial_port);
    close(bluetooth_serial_port);

    return 0;
}

/*-------------------------------------------------------------------------------------*/
void init_uart(int* serial_port, struct termios* tty, const char* device, speed_t baud_rate) {
    *serial_port = open(device, O_RDWR);

    if (*serial_port < 0) {
        if (strcmp(device, WIFI_SERIAL_PORT) == 0) {
            perror("Lỗi khi mở cổng UART cho WiFi");
        }else if (strcmp(device, BLUETOOTH_SERIAL_PORT) == 0) {
            perror("Lỗi khi mở cổng UART cho Bluetooth");
        }
        exit(-1);
    }

    // Cấu hình tốc độ baudrate và các thông số khác cho cổng UART của WiFi
    tcgetattr(*serial_port, tty);
    tty->c_cflag = baud_rate; // Tốc độ baudrate 9600 (hoặc theo tốc độ của ESP8266)
    tty->c_cflag |= CS8;    // 8-bit data
    tty->c_cflag |= CREAD;  // Cho phép đọc dữ liệu
    tty->c_cflag |= CLOCAL; // Không sử dụng UART để điều khiển
    tty->c_iflag = 0;   // Không sử dụng chế độ đầu vào đặc biệt
    tty->c_oflag = 0;   // Không sử dụng chế độ đầu vào đặc biệt
    tty->c_lflag = 0;   // Không sử dụng chế độ đầu vào đặc biệt

    tcflush(*serial_port, TCIFLUSH);    // Xoá bộ nhớ đệm đầu vào

    if (tcsetattr(*serial_port, TCSANOW, tty) != 0) {
        if (device == WIFI_SERIAL_PORT) {
            perror("Lỗi khi cài đặt cấu hình cổng UART cho WiFi\n");
        }else if (device == BLUETOOTH_SERIAL_PORT) {
            perror("Lỗi khi cài đặt cấu hình cổng UART cho Bluetooth\n");
        }
        exit(-1);
    }
}

void send_at_command(int serial_port, const char* command) {
    write(serial_port, command, strlen(command));
    usleep(100 * 1000); // Chuyển đổi đơn vị từ millisecond sang microsecond
}

void read_at_response(int serial_port) {
    memset(response, 0, sizeof(response)); 
    bytes_read = read(serial_port, response, sizeof(response)); 
    if (bytes_read > 0) {
        response[bytes_read] = '\0';
        if (serial_port == wifi_serial_port) {
            printf("Response from ESP8266: %s\n", response);
        } else if (serial_port == bluetooth_serial_port) {
            printf("Response from HC05: %s\n", response);
        }
    }
}

void configure_access_point(const char* ssid, const char* password) {
    send_at_command(wifi_serial_port, "AT\r\n");
    read_at_response(wifi_serial_port);
    
    send_at_command(wifi_serial_port, "AT+CWMODE=2\r\n");
    read_at_response(wifi_serial_port);
   
    char set_ssid_command[128];
    sprintf(set_ssid_command, "AT+CWSAP=\"%s\",\"%s\",5,3\r\n", ssid, password);
    send_at_command(wifi_serial_port, set_ssid_command);
    read_at_response(wifi_serial_port);

    send_at_command(wifi_serial_port, "AT+CIPMUX=1\r\n");
    read_at_response(wifi_serial_port);

    send_at_command(wifi_serial_port, "AT+CIPSERVER=1,80\r\n");
    read_at_response(wifi_serial_port);

    send_at_command(wifi_serial_port, "AT+CIPSTATUS\r\n");
    usleep(60000);
    read_at_response(wifi_serial_port); 

    send_at_command(wifi_serial_port, "AT+CWJAP?\r\n");
    read_at_response(wifi_serial_port);
    
    send_at_command(wifi_serial_port, "AT+CWJAP\r\n");
    read_at_response(wifi_serial_port);
    
    send_at_command(wifi_serial_port, "AT+CWLIF\r\n");
    read_at_response(wifi_serial_port);

    send_at_command(wifi_serial_port, "AT+CIPSEND=0,5\r\n");
    read_at_response(wifi_serial_port);
    usleep(60000);
    send_at_command(wifi_serial_port, "hello");

}

void configure_hc05_as_master() {

    send_at_command(bluetooth_serial_port, "AT\r\n");
    read_at_response(bluetooth_serial_port);
    
    send_at_command(bluetooth_serial_port, "AT+ORGL\r\n");
    read_at_response(bluetooth_serial_port);

    send_at_command(bluetooth_serial_port, "AT+RMAAD\r\n");
    read_at_response(bluetooth_serial_port);

    send_at_command(bluetooth_serial_port, "AT+PSWD=0000\r\n");
    read_at_response(bluetooth_serial_port);

    send_at_command(bluetooth_serial_port, "AT+ROLE=0\r\n");
    read_at_response(bluetooth_serial_port);

    send_at_command(bluetooth_serial_port, "AT+CMODE=0\r\n");
    read_at_response(bluetooth_serial_port);

    
    
    // send_at_command(bluetooth_serial_port, "AT+INIT\r\n");
    // read_at_response(bluetooth_serial_port);

    // send_at_command(bluetooth_serial_port, "AT+INQ\r\n");
    // read_at_response(bluetooth_serial_port);

    // send_at_command(bluetooth_serial_port, "AT\r\n");
    // read_at_response(bluetooth_serial_port);
    // // Đặt HC-05 làm Master
    // send_at_command(bluetooth_serial_port, "AT+ROLE=0\r\n");
    // read_at_response(bluetooth_serial_port);
    
    // // Đặt chế độ kết nối để kết nối với các thiết bị khác
    // send_at_command(bluetooth_serial_port, "AT+CMODE=0\r\n");
    // read_at_response(bluetooth_serial_port);
    

}
void* wifi_task(void* arg) {
    while (1) {
        wifi_bytes_read = read(wifi_serial_port, wifi_buffer, sizeof(wifi_buffer));
        if (wifi_bytes_read > 0) {
            wifi_buffer[wifi_bytes_read] = '\0';
            if (processJsonData(wifi_buffer) != NULL) {
                printf("Received data from WiFi: %s\n", processJsonData(wifi_buffer));
                processJsonData(processJsonData(wifi_buffer));
                // free(processJsonData(wifi_buffer));
                // printf("Temperature: %s\n", extractValueByKey(processJsonData(wifi_buffer), "_A_"));
                // printf("Temperature: %s\n", extractValueByKey(processJsonData(wifi_buffer), "temperature_B"));
                calculatePerformance(processJsonData(wifi_buffer), "_B_");
                //printf("Received data from WiFi: %s\n", extractValueByKey(processJsonData(wifi_buffer), "_A_"));

            }
            pthread_mutex_lock(&wifi_mutex);
            // Xử lý và gửi dữ liệu đến Bluetooth
            pthread_mutex_unlock(&wifi_mutex);
        }
        send_data_to_esp8266("HELO", "_A_");
        usleep(10000);
    }
    pthread_exit(NULL);
}
// Hàm tính hiệu suất

void calculatePerformance(const char *jsonStr, const char *key) {
    static int start, current, last = 0;
    static int  i = 0;
    if (strstr(jsonStr, key) != NULL){
        if (i == 0) {
            start = atoi(extractValueByKey(jsonStr, key));
        }
        current = atoi(extractValueByKey(jsonStr, key));
        if (current != last) {
            i++;
        }else {
            last = current;
        }

        if (i <= 50) {
            printf("%d\n", i);
            printf("%d\n", start);
            printf("%d\n", current);
            printf("Hiệu suất của %s: %.2f\n", key, ((float)i/(current-start+1))*100);
        }
    }
}

void* bluetooth_task(void* arg) {
    while (1) {
        bluetooth_bytes_read = read(bluetooth_serial_port, bluetooth_buffer, sizeof(bluetooth_buffer));
        if (bluetooth_bytes_read > 0) {
            bluetooth_buffer[bluetooth_bytes_read] = '\0';
            //if (processJsonData(bluetooth_buffer) != NULL) {
                printf("Received data from Bluetooth: %s\n", processJsonData(bluetooth_buffer));
                free(processJsonData(bluetooth_buffer));
            //}
        }
        usleep(10000);
    }
    pthread_exit(NULL);
}

void on_connect(struct mosquitto* mosq, void* obj, int rc) {
    if (rc == 0) {
        printf("Connected to MQTT broker successfully.\n");
        mosquitto_subscribe(mosq, NULL, topic, 0);
    } else {
        fprintf(stderr, "Failed to connect to the broker\n");
        exit(1);
    }
}

void* mqtt_task(void* arg) {
    mosquitto_lib_init();

    mosq = mosquitto_new(NULL, true, NULL);
    mosquitto_username_pw_set(mosq, username, NULL);
    mosquitto_connect_callback_set(mosq, on_connect);

    if (mosquitto_connect(mosq, host, 1883, 60) == MOSQ_ERR_SUCCESS) {
        while (1) {
            mosquitto_loop_start(mosq);

            // Khai báo một biến để lưu giá trị ID gói dữ liệu gần nhất đã gửi
            int lastMessageID = -1, currentMessageID = 0;
            
            if (wifi_bytes_read > 0) {
                wifi_buffer[wifi_bytes_read] = '\0';
                if (strstr(wifi_buffer, "temperature") != NULL) {
                    char* start = strchr(wifi_buffer, '{');
                    if (start != NULL) {
                        char* end = strchr(start, '}');
                        if (end != NULL) {
                            size_t length = end - start + 1;
                            strncpy(payload, start, length);
                            payload[length] = '\0';
                            // Kiểm tra xem gói dữ liệu đã được gửi chưa
                            if (lastMessageID != currentMessageID) {
                                pthread_mutex_lock(&mqtt_mutex);
                                mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, false);
                                pthread_mutex_unlock(&mqtt_mutex);
                                currentMessageID = lastMessageID;
                            }
                            // Tăng giá trị currentMessageID để sử dụng cho gói dữ liệu tiếp theo
                            currentMessageID++;
                        }
                    }
                }
            }
            usleep(10000);
        }
    } else {
        fprintf(stderr, "Cannot connect to the MQTT broker\n");
    }
    mosquitto_loop_forever(mosq, -1, 1);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    pthread_exit(NULL);
}

void processJsonString(const char *jsonStr) {
    struct json_object *jsonObj = json_tokener_parse(jsonStr);

    if (jsonObj != NULL) {
        enum json_type type;
        json_object_object_foreach(jsonObj, key, val) {
            type = json_object_get_type(val);

            if (type == json_type_object) {
                // Nếu giá trị là một đối tượng JSON, đệ quy xử lý nó
                printf("Object Key: %s\n", key);
                processJsonString(json_object_to_json_string(val));
            } else {
                printf("Key: %s, Value: %s\n", key, json_object_to_json_string(val));
            }
        }
        json_object_put(jsonObj); // Giải phóng bộ nhớ
    } else {
        printf("Invalid JSON\n");
    }
}

// Hàm trích xuất dữ liệu JSON từ chuỗi
char* processJsonData(const char *jsonStr) {
    char *start = strchr(jsonStr, '{'); // Tìm dấu mở ngoặc nhọn
    char *end = strchr(jsonStr, '}');   // Tìm dấu đóng ngoặc nhọn

    if (start != NULL && end != NULL) {
        // Tính độ dài của chuỗi JSON
        size_t length = end - start + 1;
        
        // Sao chép chuỗi JSON vào một bộ đệm
        char *result = (char *)malloc(length + 1);
        strncpy(result, start, length);
        result[length] = '\0'; // Kết thúc chuỗi đúng cách

        // // In chuỗi JSON trích xuất được
        // printf("Extracted JSON: %s\n", jsonBuffer);

        return result;
        
    } else {
        //printf("\n");
        return NULL;
    }
}

// Hàm để xử lý dữ liệu từ một thiết bị
// void processDeviceData(const struct json_object* deviceData) {
//     struct json_object* id;
//     struct json_object* temperature;
//     struct json_object* humidity;

//     if (json_object_object_get_ex(deviceData, "id", &id) &&
//         json_object_object_get_ex(deviceData, "temperature", &temperature) &&
//         json_object_object_get_ex(deviceData, "humidity", &humidity)) {
//         int id_value = json_object_get_int(id);
//         double temperature_value = json_object_get_double(temperature);
//         double humidity_value = json_object_get_double(humidity);

//         printf("Thiết bị - ID: %d, Temperature: %.2f, Humidity: %.2f\n", id_value, temperature_value, humidity_value);
//     } else {
//         printf("Dữ liệu JSON cho thiết bị không hợp lệ\n");
//     }
// }

// Hàm để xử lý dữ liệu từ một thiết bị
// Hàm trích xuất dữ liệu từ chuỗi JSON dựa trên key và trả về giá trị dưới dạng chuỗi
const char *extractValueByKey(const char *json_string, const char *key) {
    struct json_object *json_obj = json_tokener_parse(json_string);
    const char *value = NULL;

    if (json_obj) {
        enum json_type type;
        json_object_object_foreach(json_obj, json_key, json_val) {
            if (strcmp(json_key, key) == 0) {
                type = json_object_get_type(json_val);

                switch (type) {
                    case json_type_string:
                        value = json_object_get_string(json_val);
                        break;
                    case json_type_int:
                        {
                            int int_value = json_object_get_int(json_val);
                            char buffer[32];
                            snprintf(buffer, sizeof(buffer), "%d", int_value);
                            value = strdup(buffer); // Cấp phát đủ bộ nhớ và sao chép giá trị
                        }
                        break;
                    case json_type_double:
                        {
                            double double_value = json_object_get_double(json_val);
                            char buffer[32];
                            snprintf(buffer, sizeof(buffer), "%.2f", double_value);
                            value = strdup(buffer); // Cấp phát đủ bộ nhớ và sao chép giá trị
                        }
                        break;
                    default:
                        value = "Unsupported data type";
                        break;
                }
                break; // Tìm thấy key, thoát khỏi vòng lặp
            }
        }
    }

    // Giải phóng bộ nhớ cho đối tượng JSON
    json_object_put(json_obj);
    return value;
}

// void processDeviceData(const char *jsonStr) {
//     struct json_object *root = json_tokener_parse(jsonStr);

//     if (root) {
//         // Liệt kê các thiết bị cần xử lý
//         const char *devices[] = { "_A_", "_B_" };
//         int numDevices = sizeof(devices) / sizeof(devices[0]);

//         for (int i = 0; i < numDevices; i++) {
//             struct json_object *deviceData;

//             // Kiểm tra và lấy dữ liệu của thiết bị cụ thể
//             if (json_object_object_get_ex(root, devices[i], &deviceData)) {
//                 printf("Device: %s\n", devices[i]);

//                 struct json_object *temperatureObj, *humidityObj;

//                 // Kiểm tra và lấy nhiệt độ và độ ẩm
//                 if (json_object_object_get_ex(deviceData, "temperature", &temperatureObj) &&
//                     json_object_object_get_ex(deviceData, "humidity", &humidityObj)) {
//                     double temperature = json_object_get_double(temperatureObj);
//                     double humidity = json_object_get_double(humidityObj);
//                     printf("Temperature: %.2f\n", temperature);
//                     printf("Humidity: %.2f\n", humidity);
//                 } else {
//                     printf("Không tìm thấy dữ liệu nhiệt độ và độ ẩm cho thiết bị %s.\n", devices[i]);
//                 }
//             } else {
//                 printf("Không tìm thấy dữ liệu cho thiết bị: %s\n", devices[i]);
//             }
//         }

//         json_object_put(root); // Giải phóng bộ nhớ
//     } else {
//         printf("Invalid JSON\n");
//     }
// }

// Hàm để ghi dữ liệu JSON vào tệp văn bản
//const char *jsonData = "{\"temperature\": 24.90, \"humidity\": 91.00}";  // Dữ liệu JSON của bạn
// const char *filename = "file.txt";  // Tên của tệp văn bản đích

// writeJsonToFile(jsonData, filename);
void writeJsonToFile(const char *jsonData, const char *filename) {
    FILE *file = fopen(filename, "w");  // Mở tệp với quyền ghi ("w")
    if (file == NULL) {
        printf("Không thể mở tệp.\n");
        return;
    }

    fprintf(file, "%s", jsonData);  // Ghi dữ liệu JSON vào tệp

    fclose(file);  // Đóng tệp

    //printf("Dữ liệu JSON đã được ghi vào %s.\n", filename);
}

// Hàm để gửi dữ liệu đến ESP8266
void send_data_to_esp8266(const char* data, char* nodeID) {
    char command[256];
    snprintf(command, sizeof(command), "AT+CIPSEND=%lu\r\n", strlen(data));

    // Gửi lệnh để khởi tạo quá trình truyền dữ liệu
    send_at_command(wifi_serial_port, command);
    //read_at_response(wifi_serial_port);

    // Gửi dữ liệu thực tế
    write(wifi_serial_port, data, strlen(data));

    // Chờ phản hồi
    usleep(10000); // Điều chỉnh thời gian chờ theo nhu cầu

    // // Kiểm tra phản hồi sau khi gửi dữ liệu
    // read_at_response(wifi_serial_port);
}
