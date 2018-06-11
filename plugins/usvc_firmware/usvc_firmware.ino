#include <Wire.h>
//#include <HMC58X3.h>
#include <LSM303.h>
#include <Servo.h>
LSM303 compass;
Servo servo;

#define PI 3.14159265

#define MAP_NUM  2     //1:実験用コース3,2:琵琶湖環境科学研究センター,3:マキノサニービーチ(大会コース),4:実験用コース1,5:実験用コース2
#define START_WAY_POINT_NUM 0      //0を最初のウェイポイント番号とする
#define RITS_WAY_POINT_NUM 48                  //ウェイポイントの数

#define MAKINO_WAY_POINT_NUM 50
#define MAX_WAY_POINT_NUM 51        //ウェイポイントの登録最大数    こを変更する場合way_pointの数も変更する
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
void double_char_conv(double x,char data[],int data_num, int floating_point_num);

//---------------------
//グローバル変数定義
//---------------------

//コンパス用変数

double Northes;    //地図上での目標角度[rad]
double Eastes;      //ボートの方位角[rad]


double alpha;    //地図上での目標角度[rad]
double alpha2;
double beta;      //ボートの方位角[rad]
double d_direction;      //目的地までの角度[rad]
double p_d_direction = 0;    //1ループ前の目的地までの角度[rad]

//GPS用変数
//大会用のwaypoint

//GPS用変数

//立命館大学のウェイポイント
long long int rits_North_way_point[RITS_WAY_POINT_NUM] = {        //北緯のウェイポイントデータ[1/10000 min]
    35453018,    // 0 (A地点)
    35450500,    // 1
    35448000,    // 2
    35445500,    // 3
    35443000,    // 4
    35440500,    // 5
    35438000,    // 6
    35436000,   // 7 (B地点)
    35436000,   // 8
    35436000,   // 9
    35436000,   // 10
    35436000,   // 11
    35436000,   // 12
    35436000,   // 13
    35436000,   // 14
    35436000,   // 15
    35436000,   // 16
    35436000,   // 17
    35436000,   // 18
    35436000,   // 19
    35436000,   // 20
    35436000,   // 21
    35433505,   // 22
    35436000,   // 23 (C地点)
    35438495,   // 24
    35436000,   // 25
    35436000,   // 26
    35436000,   // 27
    35436000,   // 28
    35436000,   // 29
    35436000,   // 30
    35436000,     // 31
    35436000,   // 32
    35436000,   // 33
    35436000,   // 34
    35436000,   // 35
    35436000,   // 36
    35436000,   // 37
    35436000,   // 38
    35436000,   // 39 (B地点)
    35438000,   // 40 
    35440500,   // 41
    35443000,   // 42
    35445500,           // 43
    35448000,           // 44
    35450500,           // 45
    35453018,   // 46 (A地点)
    35455997    // 47 (ゴール地点)


  };
long long int rits_East_way_point[RITS_WAY_POINT_NUM] = {        //東経のウェイポイントデータ[1/10000 min]
   
    136064675,    // 0 (A地点)
    136066800,    // 1
    136068900,    // 2
    136071100,    // 3
    136073300,    // 4
    136075500,    // 5
    136077750,    // 6
    136080000,    // 7 (B地点)
    136084200,    // 8
    136088400,    // 9
    136092600,    // 10
    136096800,    // 11
    136101000,    // 12
    136105200,    // 13
    136109400,    // 14
    136113600,    // 15
    136117800,    // 16
    136122000,    // 17
    136126200,    // 18
    136130400,    // 19
    136134600,    // 20
    136138800,    // 21
    136143000,    // 22
    136150000,    // 23 (C地点)
    136143000,    // 24
    136138800,    // 25
    136134600,    // 26
    136130400,    // 27
    136126200,    // 28
    136122000,    // 29
    136117800,    // 30
    136113600,    // 31
    136109400,    // 32
    136105200,    // 33
    136101000,    // 34
    136096800,    // 35
    136092600,    // 36
    136088400,    // 37
    136084200,    // 38
    136080000,    // 39 (B地点)
    136077750,    // 40
    136075500,    // 41
    136073300,    // 42
    136071100,    // 43
    136068900,    // 44
    136066800,    // 45
    136064675,    // 46 (A地点)
    136061992   // 47 (ゴール地点)

  };
  int rits_allowable_error_dis[RITS_WAY_POINT_NUM] = {        //許容誤差距離[m]    ウェイポイントまでの残り距離がこの値になるとウェイポイントを次に切り替える
    30,    // 0 (A地点)
    150,    // 1
    150,    // 2
    150,    // 3
    150,    // 4
    150,    // 5
    150,    // 6
    30,    // 7(B地点)
    150,    // 8
    150,    // 9
    150,    // 10
    150,    // 11
    150,    // 12
    150,    // 13
    150,    // 14
    150,    // 15
    150,    // 16
    150,    // 17
    150,    // 18
    150,    // 19
    150,    // 20
    150,    // 21
    150,    // 22
    30,    // 23(C地点)
    150,    // 24
    150,    // 25
    150,    // 26
    150,    // 27
    150,    // 28
    150,    // 29
    150,    // 30
    150,    // 31
    150,    // 32
    150,    // 33
    150,    // 34
    150,    // 35
    150,    // 36
    150,    // 37
    150,    // 38
    30,    // 39 (B地点)
    150,    // 40
    150,    // 41
    150,    // 42
    150,    // 43
    150,    // 44
    150,    // 45
    30,    // 46 (A地点)
    30    //  47 (ゴール地点)
  };

/*

  
  long long int rits_North_way_point[RITS_WAY_POINT_NUM] = {
    35453018,    // 0 (A地点) 35.436758
    35450500,           // 1
    35448000,           // 2
    35445500,           // 3
    35443000,           // 4
    35440500,           // 5
    35438000,           // 6
    35436000,   // 7 (B地点)
    35436000,   // 8
    35436000,   // 9
    35436000,   // 10
    35436000,   // 11
    35436000,   // 12
    35436000,   // 13
    35436000,   // 14
    35436000,   // 15
    35436000,   // 16
    35436000,   // 17
    35436000,   // 18
    35436000,   // 19
    35436000,   // 20
    35436000,   // 21
    35436000,   // 22
    35436000,   // 23
    35436000,   // 24 (C地点)
    35436000,   // 25
    35436000,   // 26
    35436000,   // 27
    35436000,   // 28
    35436000,   // 29
    35436000,   // 30
    35436000,   // 31
    35436000,   // 32
    35436000,     // 33
    35436000,   // 34
    35436000,   // 35
    35436000,   // 36
    35436000,   // 37
    35436000,   // 38
    35436000,   // 39
    35436000,   // 40
    35436000,   // 41 (B地点)
    35438000,   // 42 
    35440500,   // 43
    35443000,   // 44
    35445500,           // 45
    35448000,           // 46
    35450500,           // 47
    35453018,   // 48 (A地点)
    35455997    // 49 (ゴール地点)

     };
  
  long long int rits_East_way_point[RITS_WAY_POINT_NUM] = { 
    136064675,    // 0 (A地点)
    136066800,    // 1
    136068900,    // 2
    136071100,    // 3
    136073300,    // 4
    136075500,    // 5
    136077750,    // 6
    136080000,    // 7 (B地点)
    136084200,    // 8
    136088400,    // 9
    136092600,    // 10
    136096800,    // 11
    136101000,    // 12
    136105200,    // 13
    136109400,    // 14
    136113600,    // 15
    136117800,    // 16
    136122000,    // 17
    136126200,    // 18
    136130400,    // 19
    136134600,    // 20
    136138800,    // 21
    136143000,    // 22
    136147000,    // 23
    136150000,    // 24 (C地点)
    136147000,    // 25
    136143000,    // 26
    136138800,    // 27
    136134600,    // 28
    136130400,    // 29
    136126200,    // 30
    136122000,    // 31
    136117800,    // 32
    136113600,    // 33
    136109400,    // 34
    136105200,    // 35
    136101000,    // 36
    136096800,    // 37
    136092600,    // 38
    136088400,    // 39
    136084200,    // 40
    136080000,    // 41 (B地点)
    136077750,    // 42
    136075500,    // 43
    136073300,    // 44
    136071100,    // 45
    136068900,    // 46
    136066800,    // 47
    136064675,    // 48 (A地点)
    136061992   // 49 (ゴール地点)
  };
  int rits_allowable_error_dis[RITS_WAY_POINT_NUM] = {        //許容誤差距離[m]    ウェイポイントまでの残り距離がこの値になるとウェイポイントを次に切り替える
    30,    // 0 (A地点)
    150,    // 1
    150,    // 2
    150,    // 3
    150,    // 4
    150,    // 5
    150,    // 6
    30,    // 7(B地点)
    150,    // 8
    150,    // 9
    150,    // 10
    150,    // 11
    150,    // 12
    150,    // 13
    150,    // 14
    150,    // 15
    150,    // 16
    150,    // 17
    150,    // 18
    150,    // 19
    150,    // 20
    150,    // 21
    150,    // 22
    150,    // 23
    30,    // 24(C地点)
    150,    // 25
    150,    // 26
    150,    // 27
    150,    // 28
    150,    // 29
    150,    // 30
    150,    // 31
    150,    // 32
    150,    // 33
    150,    // 34
    150,    // 35
    150,    // 36
    150,    // 37
    150,    // 38
    150,    // 39
    150,    // 40
    30,    // 41 (B地点)
    150,    // 42
    150,    // 43
    150,    // 44
    150,    // 45
    150,    // 46
    150,    // 47
    30,    // 48 (A地点)
    30    //  49 (ゴール地点)
  };
*/  
  long long int North_way_point[MAX_WAY_POINT_NUM] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  };
  
  long long int East_way_point[MAX_WAY_POINT_NUM] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  };
  
  long long int allowable_error_dis[MAX_WAY_POINT_NUM] = {        //許容誤差距離[m]    ウェイポイントまでの残り距離がこの値になるとウェイポイントを次に切り替える
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
  };

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
long int GPS_temp =0;          //long long intから分割してシリアル送信するための変数

char North_char1[10];        //GPSからの北緯データ格納用変数
char North_char2[10];
char East_char1[10];          //GPSからの東経データ格納用変数
char East_char2[10]; 

long North_long;
long East_long;

long long int North;        //GPSから取得した北緯のlong long int型の変数
long long int East;          //GPSから取得した東経のlong long int型の変数

long long int way_point_North_mini;      //次のウェイポイントの北緯を分に変換した変数
long long int way_point_East_mini;        //次のウェイポイントの東経を分に変換した変数

long int d_North;        //目的地までの北緯方向の距離[min]
long int d_East;          //目的地までの東経方向の距離[min]

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

char North_deg[5] = {0,0,0,0};
char East_deg[6] = {0,0,0,0,0};

char next_way_point_num = 0;    

char boat_rad[3] = {0,0};    //bluetooth送信用データ（1.042[rad]→{10,42}，）

int signal_cnt = 0;

//---------------------
//セットアップ
//---------------------
void setup() {

  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  int i = 0;
  //int j = 0;
  Serial.begin(9600);
  Serial3.begin(115200);  
  Serial2.begin(115200);
  Wire.begin();
  
  servo.attach(SERVO_PIN_NUM);
  
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
   // no delay needed as we have already a delay(5) in HMC5843::init()
  compass.init(); // Dont set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  //compass.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
 
    compass.enableDefault();
//  compass.setMode(0);
  InitLPF_heading(SAMPLE_TIME, CUTOFF_FREQ_HEADING, ZETA_HEADING);//LPF初期設定
  InitLPF_pulse(SAMPLE_TIME, CUTOFF_FREQ_PULSE, ZETA_PULSE);//LPF初期設定
  
  way_point_cnt = START_WAY_POINT_NUM;      //最初のウェイポイントの番号を設定
  
  if(MAP_NUM == 2)
  {
    max_way_point_num = RITS_WAY_POINT_NUM;
    for(i = 0; i < max_way_point_num; i++)
    {
     East_way_point[i] = rits_East_way_point[i];
     North_way_point[i] = rits_North_way_point[i];
     allowable_error_dis[i] = rits_allowable_error_dis[i];
    }
  }
  
  for(i = 0; i < max_way_point_num; i++)    //北緯は8桁、東経は9桁に桁を合わせる
  {
   while(North_way_point[i] < 10000000) 
   {
    North_way_point[i] *= 10; 
   }
   while(North_way_point[i] >= 100000000)
   {
     North_way_point[i] /= 10;
   }
   
   while(East_way_point[i] < 100000000) 
   {
    East_way_point[i] *= 10; 
   }
   while(East_way_point[i] >= 1000000000)
   {
     East_way_point[i] /= 10;
   }
  }
}

//---------------------
//main
//---------------------

void loop() {
  int i;

 //   Serial.print("ccc");

  
  if (Serial3.available()) {
    c = Serial3.read(); 
    //Serial.print(c);
     
    if(flag_doll ==1 || doll_cnt!=0){
      GPS[doll_cnt] = c;
      doll_cnt ++;
    }    
   
    if(doll_cnt == 5){
      if(GPS[0] == 71 && GPS[1] == 80 && GPS[2] == 71 && GPS[3] == 71 && GPS[4] == 65 ){         //ASCIIコードでG=71,P=80,A=65
        flag_GPS_start = 1;
      }
      flag_doll = 0;
      doll_cnt = 0;
    }
    
    if(flag_GPS_start == 1){
      if(GPS_cnt >= 13 && GPS_cnt <= 22){
        North_char1[GPS_cnt -13 ] = c;
        
        
        if(c != 46){
          North_char2[north_cnt] = c;
          north_cnt++; 
        }
        
       if(GPS_cnt == 22){
         North_long = atol(North_char2);     

 //       Serial.print("N_L="); //////////
     //   Serial.print(North_long);////////
         
         North = (int)(North_long/1000000)*1000000;
         North = North + (North_long-North)*100/60;
  
         
        GPS_temp = North / 100000;

 
         north_cnt = 0; 
        }
      }
      
      if(GPS_cnt >= 25  && GPS_cnt <= 34){
        East_char1[GPS_cnt -25] = c;
        if(c != 46){
          East_char2[east_cnt] = c;
          east_cnt++; 
          
        }
         if(GPS_cnt == 34){
         East_long = atol(East_char2);
         East = (int)(East_long/1000000)*1000000;
         East = East + (East_long-East)*100/60;
          GPS_temp = East / 100000;
          GPS_temp = East % 100000;
          get_GPS_flag = 1;

          east_cnt = 0;
        }
       
      }
      
      GPS_cnt ++;
      if(GPS_cnt == 35){
        GPS_cnt = 0;
        flag_GPS_start = 0; 
      }
    }
         
    if(c==36){
      flag_doll = 1;
    }
  }
    if(get_GPS_flag == 1)
    {
        delay(20);
        get_GPS_flag = 0;
   
     d_North = North_way_point[way_point_cnt] - North;
     d_East = East_way_point[way_point_cnt] - East;
     
     d_North_dis =(double) d_North * 111/1000;
     d_East_dis = (double) d_East * 91/1000;

    Northes=(double)North;
    Eastes=(double)East;
     
        Serial.print("  N=");

     Serial.print(Northes);
     Serial.print("E=");

     Serial.print(Eastes);

     
   
     distance = sqrt((long double)d_North_dis * (long double)d_North_dis + (long double)d_East_dis * (long double)d_East_dis);
     Serial.print("d=");

     Serial.print(distance);
//     Serial.print("d  ");

     Serial.print("  ");

     Serial.print("cnt= ");


     Serial.print(way_point_cnt);
     
     if(distance < allowable_error_dis[way_point_cnt])
     {
       
      way_point_cnt++; 
      if(way_point_cnt == max_way_point_num)
      {
       way_point_cnt = 0; 
      }
     }
 //    alpha = atan2(d_East_dis,d_North_dis);
 //    alpha = atan2(d_East_dis,d_North_dis);
     alpha =  atan2(d_East_dis,d_North_dis);
  //   alpha2 = PI/2 - atan2(d_North_dis,d_East_dis);
    /*if(alpha < 0)
    {
    alpha += 2 * PI;
    }  */
    
    
     
    beta = Get_Compass()-PI;
   
    beta -= COMPASS_OFFSET* PI / 180;
    
    Serial.print("  al=");
    Serial.print(alpha *180/PI);

    //Serial.print("  al2=");
    //Serial.print(alpha2 *180/PI);
    
    Serial.print("  be=");  
    
   
    if(beta < -PI)
    {
    beta += 2 * PI;
    }  
    if(beta >  PI)
    {
     beta -= 2 * PI; 
    }

    
    
    
    Serial.print(beta *180/PI);
    
    d_direction = alpha - beta;
 //   d_direction = alpha + beta;
    d_direction -= GPS_OFFSET* PI / 180;
    
    if(d_direction < (-1) * PI)
    {
     d_direction += 2 * PI; 
    }
    if(d_direction > PI)
    {
     d_direction -= 2 * PI; 
    }    
    
    Serial.print("  dir=");
    
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
    
    Serial.println(d_direction*180/PI);
  
    
    if(d_direction < LOW_GAIN_DEG * PI / 180 && d_direction > LOW_GAIN_DEG * PI / 180 * (-1))
    {
      input_pulse = LOW_GAIN_KP * d_direction - LOW_GAIN_KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;
Serial.print("IP1=");
Serial.print(input_pulse);
Serial.print(" ");
      
    }
    else{
        input_pulse = KP * d_direction - KV * (d_direction - p_d_direction) / SAMPLE_TIME + MID_PULSE;  
      Serial.print("IP2=");
Serial.print(input_pulse);
Serial.print(" ");
    }
    
    if(PULSE_LPF != 0)
    {
      input_pulse = LPF_OnePassEx_pulse((double)input_pulse);      //入力パルスにローパスフィルタをかける
    }
    
    if(input_pulse < MIN_PULSE)      //制御入力パルスを上限、下限の範囲以内にする
    {
     input_pulse = MIN_PULSE; 
    }
    if(input_pulse > MAX_PULSE)
    {
     input_pulse = MAX_PULSE; 
    }
    
    p_d_direction = d_direction;

Serial.print("IP=");
Serial.print(input_pulse);
Serial.print(" ");

     
     servo.writeMicroseconds(input_pulse);
     
     
     if(signal_cnt >= SIGNAL_CNT_NUM)    //SIGNAL_CNT_NUM回に1度bluetoothにてデータを送信する
     {
       int_char_conv(North * 5 / 3,North_deg,8);
       int_char_conv(East * 5 / 3,East_deg,9);
       double_char_conv(beta,boat_rad,4,3);
       next_way_point_num = way_point_cnt + 1;
           
       Serial2.print(start_signal);
       
       for(i = 0; i < 4; i++)
       {
         Serial2.print(North_deg[i]);
       }
       for(i = 0; i < 5; i++)
       {
         Serial2.print(East_deg[i]);
       }
       Serial2.print(next_way_point_num);
       for(i = 0; i < 2; i++)
       {
        Serial2.print(boat_rad[i]) ;
       }
       signal_cnt = 0;
     }
     signal_cnt++;
    }
    GPS_loop_cnt = 0;
 // }
  GPS_loop_cnt++;
 }

//////////////////
//方位角取得関数//
//////////////////
double Get_Compass(void)
{

  compass.read();
float headingd = compass.heading();
  float heading = headingd*PI/180;

  Serial.print("  h=");
  Serial.print(heading *180/PI);
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
  if(HEADING_LPF != 0)
  {
    return heading_LPF;
  }
  else{
    return heading;
  }
}

/////////////////////////
// ローパスフィルタ    //
/////////////////////////

//ローパスフィルタの設定を行う
void InitLPF_heading(double sampTime, double cutoffFreq, double zeta)
{
  double wAF_heading = cutoffFreq * (2 * PI);     //アナログフィルタにおけるカットオフ角周波数 [rad/s]
  double w_heading   = atan2(wAF_heading*sampTime, 2) * 2 / sampTime; //デジタルフィルタにおけるカットオフ角周波数 [rad/s]
  double wT_heading  = w_heading * sampTime;        //ω･T

  g_lpf_wT_2_heading = wT_heading * wT_heading;         //(ω･T)^2 を計算する
  g_lpf_a0_heading   =  4 + (4 * zeta * wT_heading) + g_lpf_wT_2_heading;   //ローパスフィルタの係数を計算する
  g_lpf_a1_heading   = -8 + (2 * g_lpf_wT_2_heading);
  g_lpf_a2_heading   =  4 - (4 * zeta * wT_heading) + g_lpf_wT_2_heading;

  ResetLPF_heading();           //ローパスフィルタを通過した回数をリセットする
}

//ローパスフィルタを通過した回数をリセットする
void ResetLPF_heading(void)
{
  g_lpf_passCnt_heading = 0;          //ローパスフィルタを通過した回数を初期化する
}


//ローパスフィルタ通過後の信号を取得する
float LPF_OnePass_heading(double x, double xp, double xpp, double yp, double ypp)
{
  //ローパスフィルタ通過後の信号を計算する
  return (g_lpf_wT_2_heading*(x + 2*xp + xpp) - g_lpf_a1_heading*yp - g_lpf_a2_heading*ypp) / g_lpf_a0_heading;
}

//ローパスフィルタ通過後の信号を取得する（信号蓄積型）
float LPF_OnePassEx_heading(double x)
{
  static double xp;               //元の信号（１つ前の値）
  static double xpp;                //（２つ前の値）
  static double yp;               //フィルタ適用後の信号（１つ前の値）
  static double ypp;                //（２つ前の値）
  float        y;                 //（現在の値）

  if(g_lpf_passCnt_heading == 0){               //1 回目なら
    xpp = x;                //元の信号を蓄積する
    y = ypp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
    g_lpf_passCnt_heading++;              //ローパスフィルタを通過した回数をカウント
  }
  else if(g_lpf_passCnt_heading == 1){                    //2 回目なら
    xp = x;                 //元の信号を蓄積する
    y  = yp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
    g_lpf_passCnt_heading++;                                                        //ローパスフィルタを通過した回数をカウント
  }

        else{                                         //3 回目以降なら（不必要なので、通過回数のカウントは行わない）
    y = LPF_OnePass_heading(x, xp, xpp, yp, ypp);                     //ローパスフィルタ通過後の信号を取得する
    xpp = xp;               //次回以降に備えて、信号をずらして蓄積する
    xp  = x;
    ypp = yp;
    yp  = y;
  }
  return y;
}

void InitLPF_pulse(double sampTime, double cutoffFreq, double zeta)
{
  double wAF_pulse = cutoffFreq * (2 * PI);     //アナログフィルタにおけるカットオフ角周波数 [rad/s]
  double w_pulse   = atan2(wAF_pulse*sampTime, 2) * 2 / sampTime; //デジタルフィルタにおけるカットオフ角周波数 [rad/s]
  double wT_pulse  = w_pulse * sampTime;        //ω･T

  g_lpf_wT_2_pulse = wT_pulse * wT_pulse;         //(ω･T)^2 を計算する
  g_lpf_a0_pulse   =  4 + (4 * zeta * wT_pulse) + g_lpf_wT_2_pulse;   //ローパスフィルタの係数を計算する
  g_lpf_a1_pulse   = -8 + (2 * g_lpf_wT_2_pulse);
  g_lpf_a2_pulse   =  4 - (4 * zeta * wT_pulse) + g_lpf_wT_2_pulse;

  ResetLPF_pulse();           //ローパスフィルタを通過した回数をリセットする
}

//ローパスフィルタを通過した回数をリセットする
void ResetLPF_pulse(void)
{
  g_lpf_passCnt_pulse = 0;          //ローパスフィルタを通過した回数を初期化する
}


//ローパスフィルタ通過後の信号を取得する
float LPF_OnePass_pulse(double x, double xp, double xpp, double yp, double ypp)
{
  //ローパスフィルタ通過後の信号を計算する
  return (g_lpf_wT_2_pulse*(x + 2*xp + xpp) - g_lpf_a1_pulse*yp - g_lpf_a2_pulse*ypp) / g_lpf_a0_pulse;
}

int LPF_OnePassEx_pulse(double x)
{
  static double xp;               //元の信号（１つ前の値）
  static double xpp;                //（２つ前の値）
  static double yp;               //フィルタ適用後の信号（１つ前の値）
  static double ypp;                //（２つ前の値）
  float        y;                 //（現在の値）

  if(g_lpf_passCnt_pulse == 0){               //1 回目なら
    xpp = x;                //元の信号を蓄積する
    y = ypp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
    g_lpf_passCnt_pulse++;              //ローパスフィルタを通過した回数をカウント
  }
  else if(g_lpf_passCnt_pulse == 1){                    //2 回目なら
    xp = x;                 //元の信号を蓄積する
    y  = yp = 0;                //フィルタ適用後の信号を蓄積する（0: 二次遅れ系の特性？）
    g_lpf_passCnt_pulse++;                                                        //ローパスフィルタを通過した回数をカウント
  }

        else{                                         //3 回目以降なら（不必要なので、通過回数のカウントは行わない）
    y = LPF_OnePass_pulse(x, xp, xpp, yp, ypp);                     //ローパスフィルタ通過後の信号を取得する
    xpp = xp;               //次回以降に備えて、信号をずらして蓄積する
    xp  = x;
    ypp = yp;
    yp  = y;
  }
  return (int)y;
}

void int_char_conv(long int x, char data[], int data_num)
{
 int i = 0;
 int j = 0;
 long int pow_up = 0;
 long int pow_down = 0;

  for(i = 0; i < data_num / 2 + data_num % 2; i++)
  {
    if( data_num % 2 !=1 || i != data_num / 2)
    {
      for(j = 0 , pow_up = 1, pow_down = 1; j < data_num - 2 * i; j++)
      {
        pow_up *= 10;
        if(j < data_num - 2 * (i + 1))
        {
          pow_down *= 10;
        }
      }
      data[i] = x % pow_up / pow_down;
    }
    else
    {
      data[i] = x % 10;
    }
    if(i == 0 && x < 0)
    {
     data[i] += 100; 
    }
  }
}

void double_char_conv(double x,char data[],int data_num, int floating_point_num)
{
  int i = 0;
  int j = 0;
  int offset = 0;
  long int pow_up = 0;
  long int pow_down = 0;
  
  for(j = 0, offset = 1; j < floating_point_num; j++)
  {
    offset *= 10;
  }
  
  for(i = 0; i < data_num / 2 + data_num % 2; i++)
  {
      if( data_num % 2 !=1 || i != data_num)
    {
      for(j = 0 , pow_up = 1, pow_down = 1; j < data_num - 2 * i; j++)
      {  
        pow_up *= 10;
        if(j < data_num - 2 * (i + 1))
        {
          pow_down *= 10;
        }
      }
      data[i] = (long int)(x * (double)offset) % pow_up / pow_down;
    }
    else
    {
      data[i] = (int)(x * (double)offset) % 10;
    }
  }
}




