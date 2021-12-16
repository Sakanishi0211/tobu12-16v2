#include "tobu.hpp"
/* -----------------------------------------------------------------------------------------------------------------------*/
//マルチコア設定
semaphore_t sem;

//for Kalman filter
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 7 ,7> P = MatrixXf::Identity(7,7);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 6, 6> Q = MatrixXf::Identity(6, 6)*1;
Matrix<float, 6, 6> R = MatrixXf::Identity(6, 6)*1;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> Beta;
volatile float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz;
volatile float Dmx,Dmy,Dmz;
volatile float Wqa=0.0, Wpa=0.0,Wra=0.0; 
volatile float Phi,Theta,Psi;
volatile float Kalman_time=0.0;
volatile uint8_t Hzcount=0;
volatile float Psiav=0.0, Phiav=0.0, Thetaav=0.0;
//for Log
volatile uint32_t Logcount=0;
volatile uint32_t Printcount=0;
const uint16_t Logdatanum=45000;
volatile float Logdata[Logdatanum]={0.0};
char sbuf[500];
const uint8_t DATANUM=33;
volatile float Tlog = 0.0;
volatile float Led_timer = 0.0;
volatile uint8_t Led_flag = 1;

//for Control
volatile float Ref_phi, Ref_t, Ref_psi;
volatile float Ref_p,Ref_q,Ref_r;
volatile float Olderr1_p=0.0,Olderr2_p=0.0,Olderr3_p=0.0;
volatile float Olderr1_q=0.0,Olderr2_q=0.0,Olderr3_q=0.0;
volatile float Olderr1_r=0.0,Olderr2_r=0.0,Olderr3_r=0.0;
volatile float Olderr1_phi=0.0, Olderr2_phi=0.0, Olderr3_phi=0.0;
volatile float Olderr1_t=0.0,   Olderr2_t=0.0,   Olderr3_t=0.0;
volatile float Olderr1_psi=0.0;
volatile float Sk_p=0.0, Sk_q=0.0, Sk_r=0.0;
volatile float Sk_phi=0.0, Sk_t=0.0, Sk_psi=0.0;
volatile float Dk_p=0.0, Dk_q=0.0, Dk_r=0.0;
volatile float Dk_phi=0.0, Dk_t=0.0;
volatile float Up=0.0,Uq=0.0,Ur=0.0;

//Filter Time Constance

const float Tc_angl = 0.018;
const float Tc_rate = 0.012;

//Angle ref Offset
const float Phi_offset   = -0.08;
const float Theta_offset = -0.01;
const float Psi_offset   = 0.00262;

//PID Gain
const float Kp_phi = 3.4;//3.5;
const float Ti_phi = 7100.0;
const float Td_phi = 0.008;//0.0015

const float Kp_theta = 3.2;
const float Ti_theta = 7600.0;
const float Td_theta = 0.008;//0.0015

const float Kp_psi = 0.7;

const float Kp_p = 0.55;//0.6;//0.20;
const float Ti_p = 18000.0;
const float Td_p = 0.0075;

const float Kp_q = 0.35;//0.40;//0.29
const float Ti_q = 18000.0;
const float Td_q = 0.0075;

const float Kp_r = 0.75;//0.9;
const float Ti_r = 10000.0;
const float Td_r = 0.0;

//for Motor control
volatile float Duty_rr,Duty_rl,Duty_fl,Duty_fr; 
volatile float Com_rr,Com_rl,Com_fl,Com_fr; 

//for etc
const uint LED_PIN = 25;
volatile uint8_t Safetycount=1;
uint32_t s_time, e_time, d_time;
float Q0av=0.0;
float RockingWings();
uint8_t Kalman_flag=1;
uint8_t Rocking_flag=0;
float Rocking_time=0.0;
float Autotakeoff_time=0.0;

void init_kalman(float mx, float my, float mz);
float rocking_wings(void);
float auto_takeoff(void);

#if 0
//volatile float PHI,THETA,PSI;
//volatile float Spsi=0.0,Sphi=0.0,St=0.0;

volatile float data2MID,data4MID;
volatile float Time_for_debug = 0.0;
#endif
/*--------------------------------------------------------------------------------------------------------------------------------------*/
void init_kalman(float mx, float my, float mz)
{
  Xe << 1.00, 0.0, 0.0, 0.0, mx, my, mz;
  Xp =Xe;
/*
  Q <<  0.00872, 0.0   , 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 0.00198, 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 0.000236 , 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 0.0      , 0.5e-5, 0.0   , 0.0,
        0.0   , 0.0   , 0.0      , 0.0   , 0.5e-5, 0.0,
        0.0   , 0.0   , 0.0      , 0.0   , 0.0   , 0.1e0;
  R <<  1e-1   , 0.0    , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 1e-1   , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 5e-1   , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 1e-2    , 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 1e-2    , 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 0.0     , 1e-3;
*/
  Q <<  1.0e-5, 0.0   , 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 1.0e-5, 0.0     , 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 1.0e-5 , 0.0   , 0.0   , 0.0,
        0.0   , 0.0   , 0.0      , 0.5e-6, 0.0   , 0.0,
        0.0   , 0.0   , 0.0      , 0.0   , 0.2e-6, 0.0,
        0.0   , 0.0   , 0.0      , 0.0   , 0.0   , 0.1e-6;

  R <<  1e-1   , 0.0    , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 1e-1   , 0.0    , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 5e-1   , 0.0     , 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 1e-4    , 0.0     , 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 1e-4    , 0.0,
        0.0    , 0.0    , 0.0    , 0.0     , 0.0     , 1e-2;


  G << -1.0,-1.0,-1.0, 0.0, 0.0, 0.0, 
        1.0,-1.0, 1.0, 0.0, 0.0, 0.0, 
        1.0, 1.0,-1.0, 0.0, 0.0, 0.0, 
       -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Beta << 0.0, 0.0, 0.0;

  P <<  1e0,   0,   0,   0,     0,     0,     0,  
          0, 1e0,   0,   0,     0,     0,     0,
          0,   0, 1e0,   0,     0,     0,     0,
          0,   0,   0, 1e0,     0,     0,     0, 
          0,   0,   0,   0, 1.0e0,     0,     0,  
          0,   0,   0,   0,     0, 1.0e0,     0,  
          0,   0,   0,   0,     0,     0, 1.0e0;
}

float auto_takeoff(void)
{
  float alpha=0.25;
  //printf("%f\n",alpha*Autotakeoff_time);
  return alpha*Autotakeoff_time;
}

float rocking_wings(void)
{
  float angle=25;
  float f=5.0;

  if(Rocking_time<4.0)
  {
    gpio_put(7,1);
    return angle*M_PI/180*sin(f*2*M_PI*Rocking_time);
  }
    gpio_put(7,0);
  return Theta_offset;

}

void read_sensor(void)
{
  float mx1,my1,mz1;

  imu_mag_data_read();
  Ax=   -acceleration_mg[0]*0.001*GRAV;
  Ay=   -acceleration_mg[1]*0.001*GRAV;
  Az=    acceleration_mg[2]*0.001*GRAV;
  Wp=    angular_rate_mdps[0]*0.001*0.017453292;
  Wq=    angular_rate_mdps[1]*0.001*0.017453292;
  Wr=   -angular_rate_mdps[2]*0.001*0.017453292;
  Dmx=  -(magnetic_field_mgauss[0]);
  Dmy=   (magnetic_field_mgauss[1]);
  Dmz=  -(magnetic_field_mgauss[2]);

#if 1     //1 多目的ホール　0研究室
//回転行列
  const float rot[9]={0.98816924, 0.14986289, -0.03259845,
    -0.14988208, 0.98870212,  0.00186814,
    0.003251012,  0.00303989,  0.99946678};
  //中心座標
  const float center[3]={245.084, 81.99799,762.78812};
  //拡大係数
  const float zoom[3]={0.0014399, 0.0022545, 0.0039835};
#else
  //回転行列
  const float rot[9]={-0.78435472, -0.62015392, -0.01402787,
    0.61753358, -0.78277935,  0.07686857,
    -0.05865107,  0.05162955,  0.99694255};
  //中心座標
  const float center[3]={-109.32529343620176, 72.76584808916506, 759.2285249891385};
  //拡大係数
  const float zoom[3]={0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

#endif
  //回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Dmx +rot[1]*Dmy +rot[2]*Dmz -center[0]);
  my1 = zoom[1]*( rot[3]*Dmx +rot[4]*Dmy +rot[5]*Dmz -center[1]);
  mz1 = zoom[2]*( rot[6]*Dmx +rot[7]*Dmy +rot[8]*Dmz -center[2]);
  //逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 
  float mag_norm=sqrt(Mx*Mx +My*My +Mz*Mz);
  Mx/=mag_norm;
  My/=mag_norm;
  Mz/=mag_norm;
}

void kalman(void){

  float err_psi, err_phi, err_t;
  float dk_a,dk_e,dk_r;
  float dk_phi,dk_psi,dk_t;
  float dt=0.01;

  /*------------------------------------------------------------------------------------------------------------------------------------*/
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    //printf("%f\n",time);

    //s_time=time_us_32();
    
    dt=0.01;
    Omega_m << (float)Wp, (float)Wq, (float)Wr;
    Z << (float)Ax, (float)Ay, (float)Az, (float)Mx, (float)My, (float)Mz;//ここに入れる
    if(Kalman_flag==1)
    {
      //std::cout << "Omega" << Omega_m << std::endl;
      //std::cout << "Z" << Z << std::endl;
      //printf("%8.2f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n",
      //  Kalman_time, Xe(0,0),Xe(1,0),Xe(2,0),Xe(3,0),Xe(4,0),Xe(5,0),Xe(6,0));
      
      ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);

      Phi = CalcPhi(Xe);
      Theta = CalcTheta(Xe);
      Psi = CalcPsi(Xe);
    
      Kalman_time = Kalman_time + 0.01;
      Rocking_time = Rocking_time + 0.01;
      Autotakeoff_time = Autotakeoff_time + 0.01;
    }    //--Begin Extended Kalman Filter--

    //printf("%8.2f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n",
    //  Kalman_time, Xe(0,0),Xe(1,0),Xe(2,0),Xe(3,0),Xe(4,0),Xe(5,0),Xe(6,0));


    //スティック上
    if(Data6<-0.2)
    {
      //printf("upper log sw\n");
      if(Logcount < Logdatanum-DATANUM){
        // LED Blink
        if(Led_timer < 20){
          gpio_put(LED_PIN, Led_flag);
          Led_timer++;
        }
        else
        {
          Led_flag=1-Led_flag;
          Led_timer = 0;
        }
        // Log write
        Logdata[Logcount++]=Xe(0,0);
        Logdata[Logcount++]=Xe(1,0);
        Logdata[Logcount++]=Xe(2,0);
        Logdata[Logcount++]=Xe(3,0);
        Logdata[Logcount++]=Xe(4,0);

        Logdata[Logcount++]=Xe(5,0);
        Logdata[Logcount++]=Xe(6,0);
        Logdata[Logcount++]=Wp-Wpa;
        Logdata[Logcount++]=Wq-Wqa;
        Logdata[Logcount++]=Wr-Wra;
        
        Logdata[Logcount++]=Ax;
        Logdata[Logcount++]=Ay;
        Logdata[Logcount++]=Az;
        Logdata[Logcount++]=Mx;
        Logdata[Logcount++]=My;
        
        Logdata[Logcount++]=Mz;
        Logdata[Logcount++]=Ref_p;
        Logdata[Logcount++]=Ref_q;
        Logdata[Logcount++]=Ref_r;
        Logdata[Logcount++]=Phi-Phiav;
        
        Logdata[Logcount++]=Theta-Thetaav;
        Logdata[Logcount++]=Psi-Psiav;
        Logdata[Logcount++]=Ref_phi;
        Logdata[Logcount++]=Ref_t;
        Logdata[Logcount++]=Ref_psi;
        
        Logdata[Logcount++]=Up;
        Logdata[Logcount++]=Uq;
        Logdata[Logcount++]=Ur;
        Logdata[Logcount++]=Dk_p;
        Logdata[Logcount++]=Dk_q;
        
        Logdata[Logcount++]=Dk_r;
        Logdata[Logcount++]=Dk_phi;
        Logdata[Logcount++]=Dk_t;
      }
      else{
        gpio_put(LED_PIN,0);
      }
    }

    //スティック下
    else if(Data6>0.2){
      //printf("lower log sw\n");
      if(Data3 < 0.3){
        if(Printcount+DATANUM < Logdatanum ){
          for (uint8_t i=0;i<DATANUM;i++){
            if(i==0){
              printf("%8.2f ", Tlog);
              Tlog = Tlog + 0.01;
            }
            sprintf(sbuf,"%16.5f ",Logdata[Printcount+i]);
            printf("%s",sbuf);
          }

          printf("\n");
          Printcount=Printcount+DATANUM;
        }
        else
        {
          gpio_put(LED_PIN, 0);
        }
      }
    }
    else if(-0.1<Data6 && Data6<0.1)
    {
      //printf("mid log sw\n");
      if(Kalman_time>7.0)gpio_put(LED_PIN, 0);
      Printcount = 0;
      Logcount = 0;
      Tlog = 0.0;
    }
    
    //角度PID制御

    Ref_phi = (Data4)*0.52398775598299*1.0 + Phi_offset;
    Ref_t   = (Data2)*0.523598775598299*1.7 + Theta_offset;
    Ref_psi =  Data1*5.5 + Psi_offset;;
    
    if(Data8 < 0.0){
      gpio_put(7,0);
      Rocking_time=0.0;
    }
    else{
      gpio_put(7, 1);
      Ref_phi=rocking_wings();
    }


    if(Data3>=0.05){
      //phi
      err_phi = (Ref_phi - (Phi - Phiav) );
      Sk_phi = Sk_phi + err_phi;//修正しました
      if (Sk_phi>30000.0)         //修正しました
      {                         //修正しました
        Sk_phi = 30000.0;         //修正しました
      }                         //修正しました
      else if(Sk_phi<-30000.0)    //修正しました
      {                         //修正しました
        Sk_phi=-30000.0;          //修正しました
      }                         //修正しました
      dk_phi = (err_phi - Olderr3_phi)*100.0;
      Dk_phi = Dk_phi*Tc_angl/(Tc_angl + 0.01) + dk_phi*0.01/(Tc_angl + 0.01);
      Ref_p = Kp_phi*(err_phi + Sk_phi*0.01/Ti_phi + Td_phi * Dk_phi);
      if(Ref_p>36.0)Ref_p = 36.0;
      else if(Ref_p<-36.0)Ref_p =-36.0;
      Olderr3_phi = Olderr2_phi;
      Olderr2_phi = Olderr1_phi;
      Olderr1_phi = err_phi;
      
      //theta
      err_t = (Ref_t - (Theta - Thetaav) );
      Sk_t = Sk_t + err_t;//修正しました
      if (Sk_t>30000.0)     //修正しました
      {                   //修正しました
        Sk_t = 30000.0;     //修正しました
      }                   //修正しました
      else if(Sk_t<-30000.0)//修正しました
      {                   //修正しました
        Sk_t =-30000.0;     //修正しました
      }                   //修正しました
      dk_t = (err_t - Olderr3_t)*100.0;
      Dk_t = Dk_t * Tc_angl/(Tc_angl + 0.01) + dk_t * 0.01/(Tc_angl + 0.01);
      Ref_q = Kp_theta * (err_t + Sk_t*0.01/Ti_theta + Td_theta * Dk_t);
      if(Ref_q>36.0)Ref_q = 36.0;
      else if(Ref_q<-36.0)Ref_q =-36.0;
      Olderr3_t = Olderr2_t;
      Olderr2_t = Olderr1_t;
      Olderr1_t = err_t;
      
      //Psi
      Ref_r = Ref_psi;

    }
    else{
      Sk_phi=0.0;
      Sk_t=0.0;
      Ref_p = 0.0;
      Ref_q = 0.0;
      Ref_r = 0.0;
      Olderr1_phi=0.0;
      Olderr2_phi=0.0;
      Olderr3_phi=0.0;
      Olderr1_t=0.0;
      Olderr2_t=0.0;
      Olderr3_t=0.0;
      //現在の角度（水平の姿勢）を記憶
      Phiav=Phi;
      Thetaav=Theta;
      Psiav=Psi;
      Dk_phi = 0.0;//追加しました
      Dk_t = 0.0;  //追加しました
      //printf("%f  %f  %f  \n",Dmx,Dmy,Dmz);
    }


    //printf("kalman %04f\n", Data6);    

  
    //e_time=time_us_32();
    //d_time=e_time-s_time;
    //printf("%u\n", d_time);
  }

}

/*float RockingWings (void){
  float ref_t;
  float deg_A;
  float w;
  float rad_A=30.0;
  float f=  0.8;
  float olddata8;

  if(olddata8-Data8>0.1 or olddata8-Data8<-0.1 ){
    Kalman_time=0.0;
  }
  rad_A=deg_A*3.141592653589793/180;
  w=f*2*3.141592653589793;
  ref_t=rad_A*sin(w*Kalman_time);
  Kalman_time=Kalman_time+0.01;
  olddata8=Data8;
  return ref_t;
}*/
/*--------------------------------------------------------------------------------------------------------------------------------------*/
void MAINLOOP(void)
{
  float mx1,my1,mz1,mag_norm;
  float dk_p, dk_q, dk_r;
  float err_p, err_q, err_r;
  float p_rate, q_rate, r_rate;
  float throt;
  //e エレベータ a エルロン r ラダー t スロットル
  pwm_clear_irq(2);
  
  read_sensor();

  if (Data10<0.0)
  {
    throt = Data3;
    Autotakeoff_time=0.0;
    gpio_put(7,0);
  }
  else
  { 
    if(Autotakeoff_time<3.0)
    {
      gpio_put(7,1);
      throt = auto_takeoff();
      if (throt>0.5)throt=0.5;
    
  }

  }



  Hzcount=Hzcount+1;
  if(Hzcount==4){
    sem_release(&sem);
    Hzcount=0;
  }

  //角速度PID制御
  
  //最大角速度 e,a 6π,r 2π
  //最大角度30°

  //エレベータピッチレートq
  q_rate = Wq - Wqa;
  err_q = (Ref_q - q_rate );
  Sk_q = Sk_q + err_q;       //修正しました
  if (Sk_q > 30000.0){     //修正しました
    Sk_q = 30000.0;        //修正しました
  }                      //修正しました
  else if(Sk_q <-30000.0 ){//修正しました
    Sk_q =-30000.0;        //修正しました
  }                      //修正しました
  dk_q  = (err_q - Olderr3_q)*400;
  Dk_q  = Dk_q * Tc_rate/(Tc_rate + 0.0025) + dk_q*0.0025/(Tc_rate + 0.0025);
  Uq = Kp_q * (err_q + Sk_q*0.0025/Ti_q + Td_q * Dk_q);
  Olderr3_q = Olderr2_q;
  Olderr2_q = Olderr1_q;
  Olderr1_q = err_q;

  //エルロンロールレートp
  p_rate = Wp - Wpa;
  err_p = (Ref_p - p_rate );
  Sk_p = Sk_p + err_p;    //修正しました
  if (Sk_p > 30000.0)     //修正しました
  {                     //修正しました
    Sk_p = 30000.0;       //修正しました
  }                     //修正しました
  else if(Sk_p <-30000.0){//修正しました
    Sk_p =-30000.0;       //修正しました
  }                     //修正しました
  dk_p = (err_p - Olderr3_p)*400;
  Dk_p = Dk_p * Tc_rate/(Tc_rate + 0.0025) + dk_p * 0.0025/(Tc_rate + 0.0025);  
  Up = Kp_p * (err_p + Sk_p * 0.0025/Ti_p + Td_p * Dk_p);
  Olderr3_p = Olderr2_p;
  Olderr2_p = Olderr1_p;
  Olderr1_p = err_p;

  //ラダーr
  r_rate = Wr - Wra;
  err_r = (Ref_r - r_rate);
  Sk_r = Sk_r + err_r;   //修正しました
  if (Sk_r > 30000.0)    //修正しました
  {                    //修正しました
    Sk_r = 30000.0;      //修正しました
  }                    //修正しました
  else if(Sk_r <-30000.0)//修正しました
  {                    //修正しました
    Sk_r =-30000.0;      //修正しました
  }                    //修正しました
  dk_r =(err_r - Olderr3_r)*400;
  Dk_r =Dk_r * Tc_rate/(Tc_rate + 0.0025) +dk_r* 0.0025/(Tc_rate + 0.0025);  
  Ur=Kp_r * (err_r + Sk_r*0.0025/Ti_r + Td_r * Dk_r);
  Olderr3_r = Olderr2_r;
  Olderr2_r = Olderr1_r;
  Olderr1_r = err_r;

  if(Kalman_time>15.0){
    if(Data3<0.1){
      if(Data2>0.9 && Data4>0.9 && Safetycount==0){
        Safetycount=1;
        //      printf("A %d %f %f \n",Safetycount,Data2,Data4);
      }
      else if(Data2<-0.9 && Data4<-0.9 && Safetycount==1){
        Safetycount=0;
        //    printf("B %d %f %f \n",Safetycount,Data2,Data4);
      }
    }
    if(Safetycount==1){
      Com_fr=0.0;
      Com_fl=0.0;
      Com_rr=0.0;         
      Com_rl=0.0;
    }
    else if(Safetycount==0){
      Com_fr = throt + ( Uq -Up +Ur) * 0.25;
      Com_fl = throt + ( Uq +Up -Ur) * 0.25;
      Com_rr = throt + (-Uq -Up -Ur) * 0.25;         
      Com_rl = throt + (-Uq +Up +Ur) * 0.25;
      //printf("%f\n",Data3);
    }
  }


  if(Data3<0.05 || Safetycount==1 || Safty_flag!=0)
  {
    set_duty_rr(0.0);
    set_duty_fr(0.0);
    set_duty_rl(0.0);
    set_duty_fl(0.0);
    Sk_p=0.0;
    Sk_q=0.0;
    Sk_r=0.0;
    Olderr1_p=0.0;
    Olderr1_q=0.0;
    Olderr1_r=0.0;
    Olderr2_p=0.0;
    Olderr2_q=0.0;
    Olderr2_r=0.0;
    Olderr3_p=0.0;
    Olderr3_q=0.0;
    Olderr3_r=0.0;
    Dk_p=0.0;
    Dk_q=0.0;
    Dk_r=0.0;     //追加しました
    Up = 0.0;
    Uq = 0.0;
    Ur = 0.0;
    //oldDk_phi=0.0; //角度制御に持っていきました
    //oldDk_t=0.0;   //角度制御に持っていきました
  }
  else
  {
    set_duty_rr(Com_rr);
    set_duty_fr(Com_fr);
    set_duty_rl(Com_rl);
    set_duty_fl(Com_fl);
  }
  if(Data9>0.0){
    
    set_servo(DUTYMIN);
  }
  else if(Data9<0.0){
    set_servo(2250);
  }

}
/*-----------------------------------------------------------------------------------------------------------------------------------------------------------*/


int main(void)
{
  uint16_t f;
  uint8_t waittime=5;
  float old_kalman_time;
  //float Thetaplas=0.0,Psiplas=0.0,Phiplas=0.0;
  const uint LED_PIN = 25;        //LED_PIN=0  
  gpio_init(LED_PIN);             //gpioを使えるようにする
  gpio_init(7);             //gpioを使えるようにする
  gpio_set_dir(25, GPIO_OUT);
  gpio_set_dir(7, GPIO_OUT);
  stdio_init_all();
  imu_mag_init();
  serial_settei();
  gpio_put(LED_PIN, 1);
  sleep_ms(1000);
  gpio_put(LED_PIN, 0);
#if 0
  for (uint8_t i=0;i<waittime;i++)
  {
     printf("#Please wait %d[s] ! \n",waittime-i);
     sleep_ms(1000);
  }
  printf("#Start Kalman Filter\n");
#endif
  
  sem_init(&sem, 0, 1);
  multicore_launch_core1(kalman);
  pwm_settei();
  printf("#Start Kalman filter test !\n");

  Q0av = 0.0;
  while(Q0av<0.995 || isinf(Q0av))
  {
    MN = 0.0;
    ME = 0.0;
    MD = 0.0;
    Wpa = 0.0;
    Wqa = 0.0;
    Wra = 0.0;
    f = 0;
    Kalman_time = 0.0;
    old_kalman_time = Kalman_time;
    while(Kalman_time<2.0)
    {
      if(Kalman_time!=old_kalman_time)
      {
        MN=MN+Mx;
        ME=ME+My;
        MD=MD+Mz;
        Wqa=Wq+Wqa;
        Wpa=Wp+Wpa;
        Wra=Wr+Wra;
        f=f+1;
        old_kalman_time = Kalman_time;
      }
    }
    Wpa=Wpa/f;
    Wqa=Wqa/f;
    Wra=Wra/f;
    MN=MN/f;
    ME=ME/f;
    MD=MD/f;
    
    Kalman_flag = 0;//Kalman filter 一時停止
    init_kalman(Wpa, Wqa, Wra);//カルマンフィルタ初期化

    printf("#omega ave %f %f %f\n",Wpa, Wqa, Wra);
    printf("#omega now %f %f %f\n",Wp, Wq, Wr);
    printf("#mag ave %f %f %f\n",MN, ME, MD);
    printf("#mag now %f %f %f\n",Mx, My, Mz);
    printf("#Kalman init Xe %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n",
        Xe(0,0),Xe(1,0),Xe(2,0),Xe(3,0),Xe(4,0),Xe(5,0),Xe(6,0));
    printf("#Kalman init Xp %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n",
        Xp(0,0),Xp(1,0),Xp(2,0),Xp(3,0),Xp(4,0),Xp(5,0),Xp(6,0));
    printf("#Kalman init  P %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f %12.4f\n",
        P(0,0),P(1,1),P(2,2),P(3,3),P(4,4),P(5,5),P(6,6));

    Kalman_time=0.0;
    Kalman_flag = 1;//Kalman filter 再開


    f=0;
    old_kalman_time=Kalman_time;
    while(Kalman_time<6.0){
      if(Kalman_time!=old_kalman_time)
      {
        printf("#Kalman time %5.2f\r", Kalman_time);
        Q0av += Xe(0,0);
        f=f+1;
        old_kalman_time=Kalman_time;
      }

    }
    printf("\n");
    Q0av/=f;
    Q0av=abs(Q0av);
    printf("#Q0av=%f\n",Q0av);
    if(Q0av<0.995 || isinf(Q0av)){
      printf("#Kalman filter Failure !\n");
      for (int i=0;i<100;i++)
      {
        gpio_put(LED_PIN, 1);
        sleep_ms(10);
        gpio_put(LED_PIN, 0);
        sleep_ms(10);
      }
    }
    else{
      printf("#Kalman filter Sucess !\n");
      for (int i=0;i<10;i++)
      {
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
      }
    }
  }

  f=0;
  old_kalman_time=Kalman_time;
  while(Kalman_time<10.0){
    if(Kalman_time!=old_kalman_time)
    {
      printf("#Kalman time %5.2f\r", Kalman_time);
      Phiav += Phi;
      Thetaav += Theta;
      Psiav += Psi;
      f=f+1;
      old_kalman_time=Kalman_time;
    }
  }
  Phiav/=f;
  Thetaav/=f;
  Psiav/=f;
  printf("\n");
  printf("#angle ave %f %f %f\n",Phiav, Thetaav, Psiav);

  while(1)
  { 
  }

}

