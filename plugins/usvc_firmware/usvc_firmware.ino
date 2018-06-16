#include <avr/pgmspace.h>
#include <limits.h>
#include <EEPROM.h>
#include <Wire.h>
//#include <HMC58X3.h>
#include <LSM303.h>
#include <Servo.h>

#define USE_NEOSWSERIAL
#ifdef USE_NEOSWSERIAL
#include <NeoSWSerial.h>
#else
#include <SoftwareSerial.h>
#endif
LSM303 compass;
Servo servo;

//#define PI 3.14159265

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//mega
#define BT_DUMP
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//uno
#define USE_NEOSWSERIAL
#ifdef USE_NEOSWSERIAL
#include <NeoSWSerial.h>
NeoSWSerial Serial3(12, 13); // RX, TX
#else
#include <SoftwareSerial.h>
SoftwareSerial Serial3(12, 13); // RX, TX
#endif
#endif

#define SOFTWARE_VERSION "0.1"
#define MAX_WAY_POINT_NUM 64
typedef struct __attribute__ ((packed)) _EEPROM_DATA {
	uint16_t max_way_point_num;
	uint32_t North_way_point[MAX_WAY_POINT_NUM]; //%3.6f fixed frew[deg]
	uint32_t East_way_point[MAX_WAY_POINT_NUM]; //%3.6f fixed frew[deg]
	uint16_t allowable_error_dis[MAX_WAY_POINT_NUM]; //[m]
	uint32_t gps_baudrate;
	int16_t rudder_pwm_offset;
	bool rudder_pwm_inv;
	uint8_t rudder_mode;
} EEPROM_DATA;

#define MAX_PULSE 2000        //入力パルスの上限(曇り:2100/晴れ:1750)
#define MIN_PULSE 1000          //入力パルスの下限(曇り:850/晴れ:1100)
#define MID_PULSE 1500      //中間の入力パルス
#define SERVO_PIN_NUM 2    //サーボモータに使用しているピン番号
#define LOOP_CNT 100
#define SAMPLE_TIME 0.030//サンプリングタイム[sec]
#define KP -1200          //舵制御用Kpゲイン
#define KV -40            //舵制御用Kvゲイン
//#define KP -1200          //舵制御用Kpゲイン
//#define KV -40            //舵制御用Kvゲイン
#define LOW_GAIN_DEG 5     //ゲインを下げる誤差角の範囲 [deg]
#define LOW_GAIN_KP -200
#define LOW_GAIN_KV -10
//#define LOW_GAIN_KP -200
//#define LOW_GAIN_KV -10
#define COMPASS_OFFSET  2         //コンパスが北を向いたときの角度β[deg]
#define GPS_OFFSET 0
#define SIGNAL_CNT_NUM 100          //信号を出力するためのループ数

#define HEADING_LPF 0            //ソーラーボート方位角にローパスを使用する場合1 使用しない場合0
#define CUTOFF_FREQ_HEADING 5    //カットオフ周波数[Hz]
#define ZETA_HEADING 1.0

#define PULSE_LPF 1              //制御入力パルスにローパスフィルタを使用する場合1 使用しない場合0
#define CUTOFF_FREQ_PULSE 5      //カットオフ周波数[Hz]
#define ZETA_PULSE 1.0

float LPF_OnePassEx_heading(double x);
int LPF_OnePassEx_pulse(double x);
void InitLPF_heading(double sampTime, double cutoffFreq_heading, double zeta_heading);
void InitLPF_pulse(double sampTime, double cutoffFreq_pulse, double zeta_pulse);
void ResetLPF_heading(void);
void ResetLPF_pulse(void);
double Get_Compass(void);

//---------------------
//グローバル変数定義
//---------------------

//コンパス用変数

double alpha;    //地図上での目標角度[rad]
double beta;      //ボートの方位角[rad]
double d_direction;      //目的地までの角度[rad]
double p_d_direction = 0;    //1ループ前の目的地までの角度[rad]

int max_way_point_num = 0;
int way_point_cnt = 0;      //次のウェイポイントの番号
char GPS[6];              //GPGGAの判定用文字配列
int flag_doll = 0;        //GPSの$判定用フラグ
int flag_GPS_start = 0;    //GPGGAが取得できたときのフラグ
int get_GPS_flag = 0;      //GPSの座標が取得できたときの-フラグ
int doll_cnt = 0;          //$以降の5文字を取得するためのカウンター
int GPS_cnt = 0;            //座標の値だけを取得するためのカウンター
int north_cnt = 0;          //取得した値から数字だけを抜き出すためのカウンター
int east_cnt = 0;

char North_char1[10];        //GPSからの北緯データ格納用変数
char North_char2[10];
char East_char1[10];          //GPSからの東経データ格納用変数
char East_char2[10];

long North_long;
long East_long;

long long int North;        //GPSから取得した北緯のlong long int型の変数[deg]
long long int East;          //GPSから取得した東経のlong long int型の変数[deg]

long int d_North;        //目的地までの北緯方向の距離[deg]
long int d_East;          //目的地までの東経方向の距離[deg]

double d_North_dis;        //目的地までの北緯方向の距離[m]
double d_East_dis;         //目的地までの東経の距離[m]

double distance;        //目的地までの距離[m]

//制御用変数

int rudder_pwm = MID_PULSE;
int rudder_pwm_offset = 0;
bool rudder_pwm_inv = 0;
#define RUDDER_MODE_MANUAL 0
#define RUDDER_MODE_AUTO 1
#define RUDDER_MODE_AUX 2
uint8_t rudder_mode = 0; //0:auto 1:manual
int GPS_loop_cnt = 0;

//ローパスフィルタ用変数

double g_lpf_wT_2_heading;
double g_lpf_a0_heading;
double g_lpf_a1_heading;
double g_lpf_a2_heading;
double g_lpf_passCnt_heading;

double g_lpf_wT_2_pulse;
double g_lpf_a0_pulse;
double g_lpf_a1_pulse;
double g_lpf_a2_pulse;
double g_lpf_passCnt_pulse;

#ifdef BT_DUMP
void int_char_conv(long int x, char data[], int data_num);
void double_char_conv(double x, char data[], int data_num, int floating_point_num);
//データ送信用変数
char start_signal = 200;

char North_deg[5] = {0, 0, 0, 0};
char East_deg[6] = {0, 0, 0, 0, 0};

char next_way_point_num = 0;

char boat_rad[3] = {0, 0};    //bluetooth送信用データ（1.042[rad]→{10,42}，）

int signal_cnt = 0;
#endif

//for command handler
char cmd[128];
uint8_t cmd_cur;

#define STRNCMP(cmd, target) strncmp(cmd, target, strlen(target))
#define offsetof_in_array(data, param, cur) offsetof(data, param) + cur*sizeof(*data::param)

void print_fixed_few_6(uint32_t v) {
	char msg[64];
	sprintf(msg, "%ld.%06ld", (uint32_t) v / (uint32_t) 1E6, (uint32_t) v % (uint32_t) 1E6);
	Serial.print(msg);
}

uint32_t parse_fixed_few_6(char *str) {
	uint32_t d = 0;
	uint32_t f = 0;
	char *d_str = strtok(str, ".");
	if (d_str != NULL) {
		d = atol(d_str) * (uint32_t) 1E6;

		char *f_str = strtok(NULL, ".");
		if (f_str != NULL) {
			f_str[6] = '\0';
			for (int i = 0; i < 6; i++) {
				if (f_str[i] == '\0') {
					f_str[i] = '0';
					f_str[i + 1] = '\0';
				}
			}
			f = atol(f_str);
		}
	}
	return d + f;
}

void print_NMEA(const char *cmd) {
	int checksum = 0;
	for (int i = 0; cmd[i] != '\0'; i++) {
		checksum ^= cmd[i];
	}
	Serial3.print("$");
	Serial3.print(cmd);
	Serial3.print("*");
	Serial3.println(checksum, HEX);
}

//---------------------
//セットアップ
//---------------------
void setup() {
	int i = 0;
	
	Serial.begin(9600);
	Serial.println("$INFO,setup started*");

	uint32_t gps_baudrate = (uint32_t) EEPROM_readlong(offsetof(EEPROM_DATA, gps_baudrate));
	Serial.print("$INFO,connect gps ");
	Serial.print(gps_baudrate);
	Serial.println("*");
	Serial3.begin(gps_baudrate);

	rudder_pwm_offset = EEPROM_readint(offsetof(EEPROM_DATA, rudder_pwm_offset));
	Serial.print("$INFO,rudder pwm offset ");
	Serial.print(rudder_pwm_offset);
	Serial.println("*");

	rudder_pwm_inv = EEPROM.read(offsetof(EEPROM_DATA, rudder_pwm_inv));
	Serial.print("$INFO,rudder pwm invert ");
	Serial.print(rudder_pwm_inv);
	Serial.println("*");

	rudder_mode = EEPROM.read(offsetof(EEPROM_DATA, rudder_mode));
	Serial.print("$INFO,rudder mode ");
	Serial.print(rudder_mode);
	Serial.println("*");

#ifdef BT_DUMP
	Serial2.begin(115200);
#endif
	Wire.begin();

	servo.attach(SERVO_PIN_NUM);

	compass.m_min = (LSM303::vector<int16_t> ) { -32767, -32767, -32767 };
	compass.m_max = (LSM303::vector<int16_t> ) { +32767, +32767, +32767 };

	// no delay needed as we have already a delay(5) in HMC5843::init()
	compass.init(); // Dont set mode yet, we'll do that later on.
	// Calibrate HMC using self test, not recommended to change the gain after calibration.
	//compass.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
	// Single mode conversion was used in calibration, now set continuous mode

	compass.enableDefault();
//  compass.setMode(0);
	InitLPF_heading(SAMPLE_TIME, CUTOFF_FREQ_HEADING, ZETA_HEADING);  //LPF初期設定
	InitLPF_pulse(SAMPLE_TIME, CUTOFF_FREQ_PULSE, ZETA_PULSE);  //LPF初期設定

	way_point_cnt = 0;      //最初のウェイポイントの番号を設定
	max_way_point_num = EEPROM_readint(offsetof(EEPROM_DATA, max_way_point_num));
	if (max_way_point_num > MAX_WAY_POINT_NUM) {
		max_way_point_num = MAX_WAY_POINT_NUM;
	}

	//dump way point
	for (i = 0; i < max_way_point_num; i++) {
		uint32_t north = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, North_way_point, i));
		uint32_t east = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, East_way_point, i));
		uint32_t aed = (uint16_t) EEPROM_readint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, i));
		Serial.print("$INFO,");
		Serial.print(i);
		Serial.print(" ");
		print_fixed_few_6(north);
		Serial.print(" ");
		print_fixed_few_6(east);
		Serial.print(" ");
		Serial.print(aed);
		Serial.println("*");
	}
	Serial.println("$INFO,setup completed*");
}

//---------------------
//main
//---------------------
void loop() {
	int i;
	char c;

	{
		int16_t pwm = 0;
		if (rudder_pwm_inv) {
			pwm = -(rudder_pwm + rudder_pwm_offset - MID_PULSE) + MID_PULSE;
		} else {
			pwm = rudder_pwm + rudder_pwm_offset;
		}
		servo.writeMicroseconds(pwm);
	}

	if (Serial.available() > 0) {
		// read the incoming byte:
		int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (cmd_cur != 0) {
				cmd[cmd_cur] = '\0';
				cmd_cur = 0;

				char *p = strtok(cmd, " ");
				if (strcmp(p, "set_way_point") == 0) {
					uint32_t cur = 0, north = 0, east = 0, aed = INT_MAX;
					char north_str[16], east_str[16];
					p = strtok(NULL, "\0");
					sscanf(p, "%d %16s %16s %d", &cur, &north_str, &east_str, &aed);
					north = parse_fixed_few_6(north_str);
					east = parse_fixed_few_6(east_str);
					if (cur >= max_way_point_num) {
						Serial.print("$RET,");
						Serial.print("error");
						Serial.println("*");
					} else {
						EEPROM_writelong(offsetof_in_array(EEPROM_DATA, North_way_point, cur), north);
						EEPROM_writelong(offsetof_in_array(EEPROM_DATA, East_way_point, cur), east);
						if (aed < INT_MAX) {
							EEPROM_writeint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur), aed);
						}
						Serial.print("$RET,");
						Serial.print("ok");
						Serial.println("*");
					}
				} else if (strcmp(p, "set_rudder_mode") == 0) {
					int16_t value = 0;
					p = strtok(NULL, "\0");
					sscanf(p, "%d", &value);
					if (value >= 0 && value <= 2) {
						rudder_mode = value;
						if (rudder_mode == RUDDER_MODE_MANUAL) {
							rudder_pwm = MID_PULSE;
						}
						EEPROM.write(offsetof(EEPROM_DATA, rudder_mode), rudder_mode);
						Serial.print("$RET,");
						Serial.print("ok");
						Serial.println("*");
					} else {
						Serial.print("$RET,");
						Serial.print("error");
						Serial.println("*");
					}
				} else if (strcmp(p, "set_rudder_pwm_offset") == 0) {
					int16_t value = 0;
					p = strtok(NULL, "\0");
					sscanf(p, "%d", &value);
					rudder_pwm_offset = value;
					EEPROM_writeint(offsetof(EEPROM_DATA, rudder_pwm_offset), rudder_pwm_offset);
					Serial.print("$RET,");
					Serial.print("ok");
					Serial.println("*");
				} else if (strcmp(p, "set_rudder_pwm_inv") == 0) {
					int16_t value = 0;
					p = strtok(NULL, "\0");
					sscanf(p, "%d", &value);
					rudder_pwm_inv = value;
					EEPROM.write(offsetof(EEPROM_DATA, rudder_pwm_inv), rudder_pwm_inv);
					Serial.print("$RET,");
					Serial.print("ok");
					Serial.println("*");
				} else if (strcmp(p, "set_rudder_pwm") == 0) {
					int16_t value = 0;
					p = strtok(NULL, "\0");
					sscanf(p, "%d", &value);
					if (rudder_mode == RUDDER_MODE_MANUAL && value >= MIN_PULSE && value <= MAX_PULSE) {
						rudder_pwm = value;
						Serial.print("$RET,");
						Serial.print("ok");
						Serial.println("*");
					} else {
						Serial.print("$RET,");
						Serial.print("error");
						Serial.println("*");
					}
				} else if (strcmp(p, "set_gps_baudrate") == 0) {
					char nmea_cmd[64];
					uint32_t gps_baudrate = 9600;
					p = strtok(NULL, "\0");
					sscanf(p, "%ld", &gps_baudrate);
					EEPROM_writelong(offsetof(EEPROM_DATA, gps_baudrate), gps_baudrate);
					Serial.print("$RET,");
					Serial.print("ok");
					Serial.println("*");

					sprintf(nmea_cmd, "PMTK251,%ld", gps_baudrate);
					print_NMEA(nmea_cmd);

					Serial3.begin(gps_baudrate);
				} else if (strcmp(p, "set_aed") == 0) {
					uint32_t cur, aed = 1;
					p = strtok(NULL, "\0");
					sscanf(p, "%ld %ld", &cur, &aed);
					if (cur >= max_way_point_num) {
						Serial.print("$RET,");
						Serial.print("error");
						Serial.println("*");
					} else {
						EEPROM_writeint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur), aed);
						Serial.print("$RET,");
						Serial.print("ok");
						Serial.println("*");
					}
				} else if (strcmp(p, "set_max_way_point_num") == 0) {
					p = strtok(NULL, "\0");
					sscanf(p, "%d", &max_way_point_num);
					if (max_way_point_num > MAX_WAY_POINT_NUM) {
						max_way_point_num = MAX_WAY_POINT_NUM;
					}
					EEPROM_writeint(offsetof(EEPROM_DATA, max_way_point_num), (uint16_t) max_way_point_num);
					Serial.print("$RET,");
					Serial.print("ok");
					Serial.println("*");
				} else if (strcmp(p, "get_way_point") == 0) {
					uint32_t cur;
					p = strtok(NULL, "\0");
					sscanf(p, "%ld", &cur);
					if (cur >= max_way_point_num) {
						Serial.print("$RET,");
						Serial.print("error");
						Serial.println("*");
					} else {
						Serial.print("$RET,");
						uint32_t north = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, North_way_point, cur));
						uint32_t east = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, East_way_point, cur));
						uint32_t aed = (uint16_t) EEPROM_readint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur));
						print_fixed_few_6(north);
						Serial.print(",");
						print_fixed_few_6(east);
						Serial.print(",");
						Serial.print(aed);
						Serial.println("*");
					}
				} else if (strcmp(p, "get_max_way_point_num") == 0) {
					Serial.print("$RET,");
					Serial.print(max_way_point_num);
					Serial.println("*");
				} else {
					Serial.print("$RET,");
					Serial.print("error,unknown,");
					Serial.print(cmd);
					Serial.println("*");
				}
			}
		} else if (cmd_cur < sizeof(cmd) - 1) {
			cmd[cmd_cur++] = c;
		}
	}

	if (Serial3.available()) {
		c = Serial3.read();
		//Serial.print(c);

		if (flag_doll == 1 || doll_cnt != 0) {
			GPS[doll_cnt] = c;
			doll_cnt++;
		}

		if (doll_cnt == 5) {
			if (GPS[0] == 'G' && GPS[1] == 'P' && GPS[2] == 'G' && GPS[3] == 'G' && GPS[4] == 'A') {         //ASCIIコードでG=71,P=80,A=65
				flag_GPS_start = 1;
			}
			flag_doll = 0;
			doll_cnt = 0;
		}

		if (flag_GPS_start == 1) {
			if (GPS_cnt >= 13 && GPS_cnt <= 22) {
				North_char1[GPS_cnt - 13] = c;

				if (c != '.') {
					North_char2[north_cnt] = c;
					north_cnt++;
				}

				if (GPS_cnt == 22) {
					North_long = atol(North_char2);

					North = (int) (North_long / 1000000) * 1000000;
					//Serial.print(North_long % 1000000);////////

					North = North + (North_long - North) * 100 / 60;

					north_cnt = 0;
				}
			}

			if (GPS_cnt >= 25 && GPS_cnt <= 34) {
				East_char1[GPS_cnt - 25] = c;
				if (c != 46) {
					East_char2[east_cnt] = c;
					east_cnt++;

				}
				if (GPS_cnt == 34) {
					East_long = atol(East_char2);
					East = (int) (East_long / 1000000) * 1000000;
					East = East + (East_long - East) * 100 / 60;
					get_GPS_flag = 1;

					east_cnt = 0;
				}

			}

			GPS_cnt++;
			if (GPS_cnt == 35) {
				GPS_cnt = 0;
				flag_GPS_start = 0;
			}
		}

		if (c == '$') {
			flag_doll = 1;
		}
	}
	if (get_GPS_flag == 1) {
		delay(20);
		get_GPS_flag = 0;

		d_North = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, North_way_point, i)) - North;
		d_East = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, East_way_point, i)) - East;

		d_North_dis = (double) d_North * 111 / 1000;
		d_East_dis = (double) d_East * 91 / 1000;

		distance = sqrt((long double) d_North_dis * (long double) d_North_dis + (long double) d_East_dis * (long double) d_East_dis);

		if (distance < (uint16_t) EEPROM_readint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, way_point_cnt))) {

			way_point_cnt++;
			if (way_point_cnt == max_way_point_num) {
				way_point_cnt = 0;
			}
		}
		alpha = atan2(d_East_dis, d_North_dis);
		/*if(alpha < 0)
		 {
		 alpha += 2 * PI;
		 }  */

		beta = Get_Compass() - PI;

		beta -= COMPASS_OFFSET * PI / 180;

		if (beta < -PI) {
			beta += 2 * PI;
		}
		if (beta > PI) {
			beta -= 2 * PI;
		}

		d_direction = alpha - beta;
		//   d_direction = alpha + beta;
		d_direction -= GPS_OFFSET * PI / 180;

		if (d_direction < (-1) * PI) {
			d_direction += 2 * PI;
		}
		if (d_direction > PI) {
			d_direction -= 2 * PI;
		}

		/*  if(d_direction < (-1) * PI)
		 {
		 d_direction += 2 * PI; 
		 }
		 if(d_direction > PI)
		 {
		 d_direction -= 2 * PI; 
		 }*/

		d_direction /= 2;

		/*if(d_direction < 0)
		 {
		 d_direction += 2 * PI; 
		 }
		 if(d_direction > 2 * PI)
		 {
		 d_direction -= 2 * PI; 
		 }*/

		if (rudder_mode == RUDDER_MODE_AUTO) {
			if (d_direction < LOW_GAIN_DEG * PI / 180 && d_direction > LOW_GAIN_DEG * PI / 180 * (-1)) {
				rudder_pwm = LOW_GAIN_KP * d_direction - LOW_GAIN_KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;
			} else {
				rudder_pwm = KP * d_direction - KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;
			}

			if (PULSE_LPF != 0) {
				rudder_pwm = LPF_OnePassEx_pulse((double) rudder_pwm);      //入力パルスにローパスフィルタをかける
			}

			if (rudder_pwm < MIN_PULSE)      //制御入力パルスを上限、下限の範囲以内にする
			{
				rudder_pwm = MIN_PULSE;
			}
			if (rudder_pwm > MAX_PULSE) {
				rudder_pwm = MAX_PULSE;
			}
		}

		p_d_direction = d_direction;

		Serial.print("$STATUS,");
		print_fixed_few_6(North);
		Serial.print(",");
		print_fixed_few_6(East);
		Serial.print(",");
		Serial.print(distance);
		Serial.print(",");
		Serial.print(way_point_cnt);
		Serial.print(",");
		Serial.print(alpha * 180 / PI);
		Serial.print(",");
		Serial.print(beta * 180 / PI);
		Serial.print(",");
		Serial.print(rudder_mode);
		Serial.print(",");
		Serial.print(rudder_pwm);
		Serial.println("*");

#ifdef BT_DUMP
		if (signal_cnt >= SIGNAL_CNT_NUM)    //SIGNAL_CNT_NUM回に1度bluetoothにてデータを送信する
		{
			int_char_conv(North * 5 / 3, North_deg, 8);
			int_char_conv(East * 5 / 3, East_deg, 9);
			double_char_conv(beta, boat_rad, 4, 3);
			next_way_point_num = way_point_cnt + 1;

			Serial2.print(start_signal);

			for (i = 0; i < 4; i++) {
				Serial2.print(North_deg[i]);
			}
			for (i = 0; i < 5; i++) {
				Serial2.print(East_deg[i]);
			}
			Serial2.print(next_way_point_num);
			for (i = 0; i < 2; i++) {
				Serial2.print(boat_rad[i]);
			}
			signal_cnt = 0;
		}
		signal_cnt++;
#endif
	}
	GPS_loop_cnt++;
}

//////////////////
//方位角取得関数//
//////////////////
double Get_Compass(void) {

	compass.read();
	float headingd = compass.heading();
	float heading = headingd * PI / 180;

	delay(100);

//  int ix,iy,iz;
	// float fx,fy,fz;
	float heading_LPF;

	// compass.getValues(&ix,&iy,&iz);
//  compass.getValues(&fx,&fy,&fz);

//  float heading = atan2(fx, fz);
//  if(heading < 0) {
//    heading += 2 * PI;
	//}

	heading_LPF = LPF_OnePassEx_heading(heading);
	if (HEADING_LPF != 0) {
		return heading_LPF;
	} else {
		return heading;
	}
}

/////////////////////////
// ローパスフィルタ    //
/////////////////////////

//ローパスフィルタの設定を行う
void InitLPF_heading(double sampTime, double cutoffFreq, double zeta) {
	double wAF_heading = cutoffFreq * (2 * PI);     //アナログフィルタにおけるカットオフ角周波数 [rad/s]
	double w_heading = atan2(wAF_heading * sampTime, 2) * 2 / sampTime; //デジタルフィルタにおけるカットオフ角周波数 [rad/s]
	double wT_heading = w_heading * sampTime;        //ω･T

	g_lpf_wT_2_heading = wT_heading * wT_heading;         //(ω･T)^2 を計算する
	g_lpf_a0_heading = 4 + (4 * zeta * wT_heading) + g_lpf_wT_2_heading;   //ローパスフィルタの係数を計算する
	g_lpf_a1_heading = -8 + (2 * g_lpf_wT_2_heading);
	g_lpf_a2_heading = 4 - (4 * zeta * wT_heading) + g_lpf_wT_2_heading;

	ResetLPF_heading();           //ローパスフィルタを通過した回数をリセットする
}

//ローパスフィルタを通過した回数をリセットする
void ResetLPF_heading(void) {
	g_lpf_passCnt_heading = 0;          //ローパスフィルタを通過した回数を初期化する
}

//ローパスフィルタ通過後の信号を取得する
float LPF_OnePass_heading(double x, double xp, double xpp, double yp, double ypp) {
	//ローパスフィルタ通過後の信号を計算する
	return (g_lpf_wT_2_heading * (x + 2 * xp + xpp) - g_lpf_a1_heading * yp - g_lpf_a2_heading * ypp) / g_lpf_a0_heading;
}

//ローパスフィルタ通過後の信号を取得する（信号蓄積型）
float LPF_OnePassEx_heading(double x) {
	static double xp;               //元の信号（１つ前の値）
	static double xpp;                //（２つ前の値）
	static double yp;               //フィルタ適用後の信号（１つ前の値）
	static double ypp;                //（２つ前の値）
	float y;                 //（現在の値）

	if (g_lpf_passCnt_heading == 0) {               //1 回目なら
		xpp = x;                //元の信号を蓄積する
		y = ypp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
		g_lpf_passCnt_heading++;              //ローパスフィルタを通過した回数をカウント
	} else if (g_lpf_passCnt_heading == 1) {                    //2 回目なら
		xp = x;                 //元の信号を蓄積する
		y = yp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
		g_lpf_passCnt_heading++;                                                        //ローパスフィルタを通過した回数をカウント
	}

	else {                                         //3 回目以降なら（不必要なので、通過回数のカウントは行わない）
		y = LPF_OnePass_heading(x, xp, xpp, yp, ypp);                     //ローパスフィルタ通過後の信号を取得する
		xpp = xp;               //次回以降に備えて、信号をずらして蓄積する
		xp = x;
		ypp = yp;
		yp = y;
	}
	return y;
}

void InitLPF_pulse(double sampTime, double cutoffFreq, double zeta) {
	double wAF_pulse = cutoffFreq * (2 * PI);     //アナログフィルタにおけるカットオフ角周波数 [rad/s]
	double w_pulse = atan2(wAF_pulse * sampTime, 2) * 2 / sampTime; //デジタルフィルタにおけるカットオフ角周波数 [rad/s]
	double wT_pulse = w_pulse * sampTime;        //ω･T

	g_lpf_wT_2_pulse = wT_pulse * wT_pulse;         //(ω･T)^2 を計算する
	g_lpf_a0_pulse = 4 + (4 * zeta * wT_pulse) + g_lpf_wT_2_pulse;   //ローパスフィルタの係数を計算する
	g_lpf_a1_pulse = -8 + (2 * g_lpf_wT_2_pulse);
	g_lpf_a2_pulse = 4 - (4 * zeta * wT_pulse) + g_lpf_wT_2_pulse;

	ResetLPF_pulse();           //ローパスフィルタを通過した回数をリセットする
}

//ローパスフィルタを通過した回数をリセットする
void ResetLPF_pulse(void) {
	g_lpf_passCnt_pulse = 0;          //ローパスフィルタを通過した回数を初期化する
}

//ローパスフィルタ通過後の信号を取得する
float LPF_OnePass_pulse(double x, double xp, double xpp, double yp, double ypp) {
	//ローパスフィルタ通過後の信号を計算する
	return (g_lpf_wT_2_pulse * (x + 2 * xp + xpp) - g_lpf_a1_pulse * yp - g_lpf_a2_pulse * ypp) / g_lpf_a0_pulse;
}

int LPF_OnePassEx_pulse(double x) {
	static double xp;               //元の信号（１つ前の値）
	static double xpp;                //（２つ前の値）
	static double yp;               //フィルタ適用後の信号（１つ前の値）
	static double ypp;                //（２つ前の値）
	float y;                 //（現在の値）

	if (g_lpf_passCnt_pulse == 0) {               //1 回目なら
		xpp = x;                //元の信号を蓄積する
		y = ypp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
		g_lpf_passCnt_pulse++;              //ローパスフィルタを通過した回数をカウント
	} else if (g_lpf_passCnt_pulse == 1) {                    //2 回目なら
		xp = x;                 //元の信号を蓄積する
		y = yp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
		g_lpf_passCnt_pulse++;                                                        //ローパスフィルタを通過した回数をカウント
	}

	else {                                         //3 回目以降なら（不必要なので、通過回数のカウントは行わない）
		y = LPF_OnePass_pulse(x, xp, xpp, yp, ypp);                     //ローパスフィルタ通過後の信号を取得する
		xpp = xp;               //次回以降に備えて、信号をずらして蓄積する
		xp = x;
		ypp = yp;
		yp = y;
	}
	return (int) y;
}

#ifdef BT_DUMP
void int_char_conv(long int x, char data[], int data_num) {
	int i = 0;
	int j = 0;
	long int pow_up = 0;
	long int pow_down = 0;

	for (i = 0; i < data_num / 2 + data_num % 2; i++) {
		if (data_num % 2 != 1 || i != data_num / 2) {
			for (j = 0, pow_up = 1, pow_down = 1; j < data_num - 2 * i; j++) {
				pow_up *= 10;
				if (j < data_num - 2 * (i + 1)) {
					pow_down *= 10;
				}
			}
			data[i] = x % pow_up / pow_down;
		} else {
			data[i] = x % 10;
		}
		if (i == 0 && x < 0) {
			data[i] += 100;
		}
	}
}

void double_char_conv(double x, char data[], int data_num, int floating_point_num) {
	int i = 0;
	int j = 0;
	int offset = 0;
	long int pow_up = 0;
	long int pow_down = 0;

	for (j = 0, offset = 1; j < floating_point_num; j++) {
		offset *= 10;
	}

	for (i = 0; i < data_num / 2 + data_num % 2; i++) {
		if (data_num % 2 != 1 || i != data_num) {
			for (j = 0, pow_up = 1, pow_down = 1; j < data_num - 2 * i; j++) {
				pow_up *= 10;
				if (j < data_num - 2 * (i + 1)) {
					pow_down *= 10;
				}
			}
			data[i] = (long int) (x * (double) offset) % pow_up / pow_down;
		} else {
			data[i] = (int) (x * (double) offset) % 10;
		}
	}
}
#endif

//
//EEPROM HELPER START
//
// read double word from EEPROM, give starting address
unsigned long EEPROM_readlong(int address) {
	//use word read function for reading upper part
	unsigned long dword = EEPROM_readint(address);
	//shift read word up
	dword = dword << 16;
	// read lower word from EEPROM and OR it into double word
	dword = dword | EEPROM_readint(address + 2);
	return dword;
}

//write word to EEPROM
void EEPROM_writeint(int address, int value) {
	EEPROM.write(address, highByte(value));
	EEPROM.write(address + 1, lowByte(value));
}

//write long integer into EEPROM
void EEPROM_writelong(int address, unsigned long value) {
	//truncate upper part and write lower part into EEPROM
	EEPROM_writeint(address + 2, word(value));
	//shift upper part down
	value = value >> 16;
	//truncate and write
	EEPROM_writeint(address, word(value));
}

unsigned int EEPROM_readint(int address) {
	unsigned int word = word(EEPROM.read(address), EEPROM.read(address + 1));
	return word;
}

//
//EEPROM HELPER END
//
