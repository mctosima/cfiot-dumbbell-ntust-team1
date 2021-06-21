// ================================================================
// ===                 Lift counter libraries                   ===
// ================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
#include "Wire.h"
#include "driver/gpio.h"

// ================================================================
// ===               Face recognition libraries                 ===
// ================================================================
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"

// ================================================================
// ===                          MQTT                            ===
// ================================================================
#include <PubSubClient.h>
#include <ArduinoWebsockets.h>

// ================================================================
// ===                   Select camera model                    ===
// ================================================================
//#define CAMERA_MODEL_ESP_EYE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ================================================================
// ===              Software I2C & Interrupt pins               ===
// ================================================================
#define I2C_SDA 15
#define I2C_SCL 13
#define BUZZER 14

// ================================================================
// ===                  Enrollment variables                    ===
// ================================================================
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

// ================================================================
// ===     Change this line based on your WiFi connection       ===
// ================================================================
/*
const char ssid[] = "BEST@LAB";
const char password[] = "bestlab1234";
*/

const char ssid[] = "Factory2_2.4G";
const char password[] = "118factory2";

/*
const char ssid[] = "Martin's iPhone";
const char password[] = "abcd1234";
*/
// ================================================================
// ===                   MQTT configuration                     ===
// ================================================================

char* mqtt_server = "140.118.25.64";
int mqtt_port = 21883 ;
char* mqtt_clientID = "ESP32Node_1624252594";
char* mqtt_username = "012dc4009a4c47a599";
char* mqtt_password = "r:6d90c293e86a4450a1";
char* mqtt_publish_topic = "qiot/things/Team1/ESP32Node/UserID";
char* mqtt_repetition_topic = "qiot/things/Team1/ESP32Node/Rep";
/*
char* mqtt_server = "192.168.1.2";
int mqtt_port = 21883 ;
char* mqtt_clientID = "dumbell2_1624095647";
char* mqtt_username = "85d628ed-3728-4105-8169-546663e12515";
char* mqtt_password = "r:38f97d548338ef8ac14e8e4290238dde";
char* mqtt_publish_topic = "qiot/things/admin/dumbell2/UserID";
char* mqtt_repetition_topic = "qiot/things/admin/dumbell2/Rep";
*/
char* mqtt_server_2 = "192.168.50.150";
int mqtt_port_2 = 1883 ;
char* mqtt_clientID_2 = "dumbell2_1624095647";
char* mqtt_username_2 = "ntust";
char* mqtt_password_2 = "ntustee";
char* mqtt_publish_topic_2 = "smartgym/dumbell1/user";
char* mqtt_repetition_topic_2 = "smartgym/dumbell1/repetition";

WiFiClient espClient;
PubSubClient client(espClient);
PubSubClient client2(espClient);

// ================================================================
// ===                   Camera initializtion                   ===
// ================================================================
camera_fb_t * fb = NULL;

long current_millis;
long last_detected_millis = 0;

void app_facenet_main();

typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

typedef enum
{
  START_STREAM,
  START_DETECT,
  SHOW_FACES,
  START_RECOGNITION,
  START_ENROLL,
  ENROLL_COMPLETE,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state=START_RECOGNITION;

typedef struct
{
  char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT to QNAP");

    if (client.connect(mqtt_clientID,mqtt_username,mqtt_password)){
      Serial.println("Connected to QNAP");
    }
    else {
      Serial.print("Failed QNAP, RC=");
      Serial.print(client.state());
      Serial.println("Try Again ...");
      delay(1000);
    }
  }
}

void reconnect_2() {
  while (!client2.connected()) {
    Serial.println("Attempting MQTT to RPi");

    if (client2.connect(mqtt_clientID_2, mqtt_username_2, mqtt_password_2)){
      Serial.println("Connected to RPi");
    }
    else {
      Serial.print("Failed Rpi, RC=");
      Serial.print(client2.state());
      Serial.println("Try Again ...");
      delay(1000);
    }
  }
}

// ================================================================
// ===               Lift counter initialization                ===
// ================================================================
MPU6050 mpu;
LiquidCrystal_I2C lcd(0x3F,16,2);  //設定LCD位置0x3F,設定LCD大小為16*2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(void* arg) {
  uint32_t gpio_num = (uint32_t) arg;  
  mpuInterrupt = true;
}

// ================================================================
// ===               Lift Counter initialization                ===
// ================================================================
int lift_counter;
bool Top, Bottom;
bool RECOGNISED;
int lift_left;

struct user {
  //String user_name;
  char* user_name;
  int lift_left; 
};

user usr[5];

void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  Serial.println(F("Initializing Camera..."));

  Serial.println(F("Initializing Server..."));
  //client.setServer(mqtt_server, mqtt_port);
  client2.setServer(mqtt_server_2, mqtt_port_2);
  //reconnect();
  //reconnect_2();
  
  RECOGNISED = false;

  usr[0].user_name = "Ernie";
  usr[0].lift_left = 10;
  usr[1].user_name = "Martin";
  usr[1].lift_left = 15;
  usr[2].user_name = "Yoga";
  usr[2].lift_left = 20;
  usr[3].user_name = "Bernard";
  usr[3].lift_left = 10;
  usr[4].user_name = "John";
  usr[4].lift_left = 25;

  pinMode(BUZZER, OUTPUT);
  
// ================================================================
// ===                       Camera Setup                       ===
// ================================================================
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

// ================================================================
// ===                       Camera init                        ===
// ================================================================
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  app_facenet_main();

// ================================================================
// ===                         I2C Setup                        ===
// ================================================================

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif


// ================================================================
// ===                Lift variables & LCD Setup                ===
// ================================================================

  lift_counter = 0;
  Top = false;
  Bottom = false;
  
  // LCD Display
  lcd.init(); //初始化LCD 
  lcd.backlight(); //開啟背光
  lcd.print("Initializing...");

// ================================================================
// ===           MPU6050 load and configure the DMP             ===
// ================================================================

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-25);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

// ================================================================
// ===           Add ESP32 interrupt handler for DMP            ===
// ================================================================
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    err = gpio_isr_handler_add(GPIO_NUM_12, dmpDataReady, (void *)GPIO_NUM_12);
    if (err != ESP_OK) {
      Serial.printf("handler add failed with error 0x%x \r\n", err);
    }
    err = gpio_set_intr_type(GPIO_NUM_12, GPIO_INTR_POSEDGE);
    if (err != ESP_OK) {
      Serial.printf("set intr type failed with error 0x%x \r\n", err);
    }
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void app_facenet_main()
{
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id)
{
  ESP_LOGD(TAG, "START ENROLLING");
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d",
           st_name.enroll_name,
           ENROLL_CONFIRM_TIMES - left_sample_face);
  return left_sample_face;
}

static esp_err_t send_face_list()
{
  Serial.println("delete_faces");
  face_id_node *head = st_face_list.head;
  char add_face[64];
  for (int i = 0; i < st_face_list.count; i++) // loop current faces
  {
    sprintf(add_face, "listface:%s", head->id_name);
    Serial.println(add_face);
    head = head->next;
  }
}

static esp_err_t delete_all_faces()
{
  delete_face_all_in_flash_with_name(&st_face_list);
  Serial.println("delete_faces");
}

void loop() {

  //client.loop();
  //client2.loop();
  
  if (RECOGNISED == false) {
    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
    http_img_process_result out_res = {0};
    out_res.image = image_matrix->item;
    
    send_face_list();
    
    while (1) {    
      fb = esp_camera_fb_get();
  
      if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION)
      {
        out_res.net_boxes = NULL;
        out_res.face_id = NULL;
  
        fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
        out_res.net_boxes = face_detect(image_matrix, &mtmn_config);
  
        if (out_res.net_boxes)
        {
          if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK)
          {
  
            out_res.face_id = get_face_id(aligned_face);
            last_detected_millis = millis();
            if (g_state == START_DETECT) {
              Serial.println("FACE DETECTED");
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("FACE DETECTED");
            }
            // RECOGNITION button is pressed and face are detected
            if (g_state == START_RECOGNITION  && (st_face_list.count > 0))
            {
              face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
              if (f)
              {
                char recognised_message[64];
                char counter_message[16];

                /*
                if (!client.connected()){
                    reconnect();
                }
                */
                
                if (!client2.connected()){
                    reconnect_2();
                }
                
                //client.loop();
                client2.loop();

                Serial.printf("publishing %s\n", f->id_name);
                //client.publish(mqtt_publish_topic, f->id_name);
                client2.publish(mqtt_publish_topic_2, f->id_name);
                
                lcd.clear();
                lcd.setCursor(0,0);

                for(int i=0 ; i<5 ; ++i){
                  if (!strcmp(usr[i].user_name, f->id_name)){
                    lift_left = usr[i].lift_left;
                  }
                }
                
                sprintf(recognised_message, "%-7s: %2s left", f->id_name, String(lift_left));
                lcd.print(recognised_message);
                
                lcd.setCursor(0,1);
                sprintf(counter_message, "Lifted %2s times", String(lift_counter));
                lcd.print(counter_message);
                
                RECOGNISED = true;
                
                delay(2000);
                break;
              }
              else
              {
                Serial.println("CAN'T RECOGNISED");
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("CAN'T RECOGNISED");
              }
            }
            dl_matrix3d_free(out_res.face_id);
          }
        }
        else
        {
          if (g_state != START_DETECT) {
            Serial.println("NO FACE DETECTED");
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("NO FACE DETECTED");
          }
        }
  
        if (g_state == START_DETECT && millis() - last_detected_millis > 500) { // Detecting but no face detected
          Serial.println("DETECTING...");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("DETECTING...");
        }
      }
      
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }
  else {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      /*
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
      */
      if (aaReal.z > 500) {
        Top = true;
        Bottom = false;
      }

      if (aaReal.z < -500) {
        Bottom = true;
      }

      if (Top & Bottom) {
        Top = false;
        Bottom = false;

        char lift_message[16];

        lift_counter++;
        lift_left--;
        char buf[10];
        if (lift_left > 0) {
          /*
          if (!client.connected()){
              reconnect();
          }
        
          client.loop();
          */
          String lift_times = String(lift_counter);
          lift_times.toCharArray(buf, 10);
          //client.publish(mqtt_repetition_topic, buf);
          
          digitalWrite(BUZZER, HIGH);
          lcd.setCursor(9,0);
          sprintf(lift_message, "%2s left", String(lift_left));
          lcd.print(lift_message);
          
          lcd.setCursor(7,1);
          lcd.print(lift_counter);
          lcd.print(" times!");
          Serial.println(lift_counter);
          delay(200);
          digitalWrite(BUZZER, LOW);
        }
        else {
          
          if (!client2.connected()){
              reconnect_2();
          }
          client2.loop();
          
          
          String lift_times = String(lift_counter);
          lift_times.toCharArray(buf, 10);
          //client.publish(mqtt_repetition_topic, buf);
          client2.publish(mqtt_repetition_topic_2, buf);
          
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Training Finish!");
          lcd.setCursor(0,1);
          lcd.print("Congratulations!");
          digitalWrite(BUZZER, HIGH);
          delay(1000);
          digitalWrite(BUZZER, LOW);
          delay(5000);
          
          RECOGNISED = false;
          lift_counter = 0;
          lift_left = 0;
        }
      }
    }
  }
}
