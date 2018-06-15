#include <avr/pgmspace.h>
#include <limits.h>
#include <EEPROM.h>
#include <Wire.h>
//#include <HMC58X3.h>
#include <LSM303.h>
#include <Servo.h>
#include <SoftwareSerial.h>
LSM303 compass;
Servo servo;

//#define PI 3.14159265

//#define POLING_DUMP
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//mega
#define BT_DUMP
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//uno
SoftwareSerial Serial3(12, 13);// RX, TX
#endif

#define SOFTWARE_VERSION "0.1"
#define MAX_WAY_POINT_NUM 64
typedef struct __attribute__ ((packed)) _EEPROM_DATA {
	uint16_t max_way_point_num;
	uint32_t North_way_point[MAX_WAY_POINT_NUM]; //%3.6f fixed frew[deg]
	uint32_t East_way_point[MAX_WAY_POINT_NUM]; //%3.6f fixed frew[deg]
	uint16_t allowable_error_dis[MAX_WAY_POINT_NUM]; //[m]
	uint32_t gps_baudrate;
} EEPROM_DATA;

#define MAX_PULSE 2200        //入力パルスの上限(曇り:2100/晴れ:1750)
#define MIN_PULSE 1000          //入力パルスの下限(曇り:850/晴れ:1100)
#define MID_PULSE 1600      //中間の入力パルス
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
void int_char_conv(long int x, char data[], int data_num);
void double_char_conv(double x, char data[], int data_num, int floating_point_num);

//---------------------
//グローバル変数定義
//---------------------

//コンパス用変数

double alpha;    //地図上での目標角度[rad]
double alpha2;
double beta;      //ボートの方位角[rad]
double d_direction;      //目的地までの角度[rad]
double p_d_direction = 0;    //1ループ前の目的地までの角度[rad]

int j;
int max_way_point_num = 0;
int way_point_cnt = 0;      //次のウェイポイントの番号
char c;
char GPS[6];              //GPGGAの判定用文字配列
int flag_doll = 0;        //GPSの$判定用フラグ
int flag_GPS_start = 0;    //GPGGAが取得できたときのフラグ
int get_GPS_flag = 0;      //GPSの座標が取得できたときの-フラグ
int doll_cnt = 0;          //$以降の5文字を取得するためのカウンター
int GPS_cnt = 0;            //座標の値だけを取得するためのカウンター
int north_cnt = 0;          //取得した値から数字だけを抜き出すためのカウンター
int east_cnt = 0;
long int GPS_temp = 0;          //long long intから分割してシリアル送信するための変数

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

int input_pulse;              //サーボモータへの入力パルス
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

//データ送信用変数
char start_signal = 200;

char North_deg[5] = { 0, 0, 0, 0 };
char East_deg[6] = { 0, 0, 0, 0, 0 };

char next_way_point_num = 0;

char boat_rad[3] = { 0, 0 };    //bluetooth送信用データ（1.042[rad]→{10,42}，）

int signal_cnt = 0;

#define STRNCMP(cmd, target) strncmp(cmd, target, strlen(target))
#define offsetof_in_array(data, param, cur) offsetof(data, param) + cur*sizeof(*data::param)

void auto_detect_baud_rate(SoftwareSerial *serial) {
	const unsigned int bauds[] = { 57600, 38400, 28800, 14400, 9600, 4800 };

	Serial.print("INFO:auto detect... ");

	for (int i = 0; i < (sizeof(bauds) / sizeof(bauds[0])); i++) {
		int p = 0;
		int r = 0;
		serial->begin(bauds[i]);
		serial->flush();
		do {
			if (serial->available()) {
				if (isprint(serial->read())) {
					p++;
				}
				r++;
			}
		} while (r < 20);
		if (p > 15) {
			Serial.print(bauds[i]);
			Serial.println(" ok");
			return;
		}
		delay(100);
	}

	Serial.println("fail auto_detect_baud_rate");
	while (1)
		;
}

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
			int keta = strlen(f_str);
			f = atol(f_str);
			for (int i = keta; i < 6; i++) {
				f *= 10;
			}
		}
	}
	return d + f;
}

//---------------------
//セットアップ
//---------------------
void setup() {
	Serial.begin(9600);
	Serial.println("INFO:setup started");

	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	int i = 0;
	//int j = 0;
	uint32_t gps_baudrate = (uint32_t) EEPROM_readlong(offsetof(EEPROM_DATA, gps_baudrate));
	if (gps_baudrate >= 9600 && gps_baudrate <= 115200) {
		Serial.print("INFO:connect gps ");
		Serial.println(gps_baudrate);
		Serial3.begin(gps_baudrate);
	} else {
		auto_detect_baud_rate (&Serial3);
	}
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
		Serial.print("INFO:");
		Serial.print(i);
		Serial.print(" ");
		print_fixed_few_6(north);
		Serial.print(" ");
		print_fixed_few_6(east);
		Serial.print(" ");
		Serial.print(aed);
		Serial.print("\r\n");
	}
	Serial.println("INFO:setup completed");
}

//---------------------
//main
//---------------------
char cmd[256];
uint8_t cmd_cur;
void loop() {
	int i;

	if (Serial.available() > 0) {
		// read the incoming byte:
		int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (cmd_cur != 0) {
				cmd[cmd_cur] = '\0';
				cmd_cur = 0;
				if (STRNCMP(cmd, "set_way_point") == 0) {
					uint32_t cur = 0, north = 0, east = 0, aed = INT_MAX;
					char north_str[16], east_str[16];
					sscanf(cmd, "set_way_point %d %16s %16s %d", &cur, &north_str, &east_str, &aed);
					north = parse_fixed_few_6(north_str);
					east = parse_fixed_few_6(east_str);
					if (cur >= max_way_point_num) {
						Serial.println("error");
					} else {
						EEPROM_writelong(offsetof_in_array(EEPROM_DATA, North_way_point, cur), north);
						EEPROM_writelong(offsetof_in_array(EEPROM_DATA, East_way_point, cur), east);
						if (aed < INT_MAX) {
							EEPROM_writeint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur), aed);
						}
						Serial.println("done");
					}
				} else if (STRNCMP(cmd, "set_gps_baudrate") == 0) {
					uint32_t gps_baudrate = 9600;
					sscanf(cmd, "set_gps_baudrate %ld", &gps_baudrate);
					EEPROM_writelong(offsetof(EEPROM_DATA, gps_baudrate), gps_baudrate);
					Serial.println("done");
				} else if (STRNCMP(cmd, "set_aed") == 0) {
					uint32_t cur, aed = 1;
					sscanf(cmd, "set_aed %ld %ld", &cur, &aed);
					if (cur >= max_way_point_num) {
						Serial.println("error");
					} else {
						EEPROM_writeint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur), aed);
						Serial.println("done");
					}
				} else if (STRNCMP(cmd, "set_max_way_point_num") == 0) {
					sscanf(cmd, "set_max_way_point_num %d", &max_way_point_num);
					if (max_way_point_num > MAX_WAY_POINT_NUM) {
						max_way_point_num = MAX_WAY_POINT_NUM;
					}
					EEPROM_writeint(offsetof(EEPROM_DATA, max_way_point_num), (uint16_t) max_way_point_num);
					Serial.println("done");
				} else if (STRNCMP(cmd, "get_way_point") == 0) {
					uint32_t cur;
					sscanf(cmd, "get_way_point %ld", &cur);
					if (cur >= max_way_point_num) {
						Serial.println("error");
					} else {
						uint32_t north = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, North_way_point, cur));
						uint32_t east = (uint32_t) EEPROM_readlong(offsetof_in_array(EEPROM_DATA, East_way_point, cur));
						uint32_t aed = (uint16_t) EEPROM_readint(offsetof_in_array(EEPROM_DATA, allowable_error_dis, cur));
						print_fixed_few_6(north);
						Serial.print(" ");
						print_fixed_few_6(east);
						Serial.print(" ");
						Serial.print(aed);
						Serial.print("\r\n");
					}
				} else if (STRNCMP(cmd, "get_max_way_point_num") == 0) {
					Serial.println(max_way_point_num);
				} else if (STRNCMP(cmd, "get_gps_point") == 0) {
					print_fixed_few_6(North);
					Serial.print(" ");
					print_fixed_few_6(East);
					Serial.println();
				}
			}
		} else {
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

					GPS_temp = North / 100000;

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
					GPS_temp = East / 100000;
					GPS_temp = East % 100000;
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

		if (c == 36) {
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
		//    alpha = atan2(d_East_dis,d_North_dis);
		//    alpha = atan2(d_East_dis,d_North_dis);
		alpha = atan2(d_East_dis, d_North_dis);
		//   alpha2 = PI/2 - atan2(d_North_dis,d_East_dis);
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

		if (d_direction < LOW_GAIN_DEG * PI / 180 && d_direction > LOW_GAIN_DEG * PI / 180 * (-1)) {
			input_pulse = LOW_GAIN_KP * d_direction - LOW_GAIN_KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;
		} else {
			input_pulse = KP * d_direction - KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;
		}

		if (PULSE_LPF != 0) {
			input_pulse = LPF_OnePassEx_pulse((double) input_pulse);      //入力パルスにローパスフィルタをかける
		}

		if (input_pulse < MIN_PULSE)      //制御入力パルスを上限、下限の範囲以内にする
		{
			input_pulse = MIN_PULSE;
		}
		if (input_pulse > MAX_PULSE) {
			input_pulse = MAX_PULSE;
		}

		p_d_direction = d_direction;

		servo.writeMicroseconds(input_pulse);

#ifdef POLING_DUMP
		Serial.print("INFO:");
		Serial.print("  N=");
		print_fixed_few_6(North);

		Serial.print(" E=");
		print_fixed_few_6(East);

		Serial.print(" d=");
		Serial.print(distance);

		Serial.print(" cnt= ");
		Serial.print(way_point_cnt);

		Serial.print("  al=");
		Serial.print(alpha * 180 / PI);

		Serial.print("  be=");
		Serial.print(beta * 180 / PI);

		//Serial.print("  al2=");
		//Serial.print(alpha2 *180/PI);

		Serial.print("  dir=");
		Serial.print(d_direction * 180 / PI);

		Serial.print(" IP=");
		Serial.print(input_pulse);

		Serial.println("");
#endif

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
	GPS_loop_cnt = 0;
	// }
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
