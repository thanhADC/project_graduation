
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>      // Khai báo thư viện LCD sử dụng I2C
#include <AD5933.h>
#define PI 3.14159265
//#define REF_RESIST  (9960)
#define NUM_INCR (100)

LiquidCrystal_I2C lcd(0x27, 20, 4); // 0x27 địa chỉ LCD, 16 cột và 2 hàng
//Tạo ký tự đặc biệt &
 byte va[] = {
  B00000,
  B01100,
  B10010,
  B10100,
  B01000,
  B10101,
  B10010,
  B01101
}; 
long START_FREQ = 0;
long REF_RESIST = 0;
long STOP_FREQ = 0;
long FREQ_INCR = 0;
double gain[ NUM_INCR+1];
int phase[NUM_INCR+1];

const int up_buttonPin = 41;    
const int down_buttonPin = 43; 
int up_buttonState = 1;
int down_buttonState = 1;

const int inc_outPin =  45;     
const int ud_outPin =  47;       
const int cs_outPin = 49;
const int loopPeriod = 100;


int opened = 0;
int u = 0;
int on=0;
int analogPin = 0;
int raw = 0;
int Vin = 5;
float Vout = 0;
float R1 = 20000;
long R2 = 0;
float buffer = 0;
int yellowled = 27;
int vin = 40;
int Rcal = 23;
int vin1 = 42;
int vin2 = 44;
int gt_Rcal;
int i = 0;
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
int range1234 = 1; int PGA15 = 1;
int len = 36; int gtlen;  // Lên là trừ
int xuong = 37; int gtxuong; // Xuống là cộng
int menu = 38; int gtmenu; 
int back = 39; int gtback; 
int macdinh = 1;
int congtru_tong = 0; int congtru_chonthamso = 0; int congtru_hethong = 0;
int demtong = 0;
int demthamso = 0; int demhethong = 0; 
int demback = 0;
int congtru_range = 0; int demrange = 0;
int congtru_PGA = 0; int demPGA = 0;
byte rowPins[ROWS] = {22, 24, 26, 28}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {30,32,34}; //connect to the column pinouts of the keypad
//Keypad Mykeys = Keypad( makeKeymap(MatrixKey), rowPins, colPins, ROWS, COLS); 
Keypad Mykeys = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
//=================================================================================================================

void calibrated()
{
  if(PGA15 == 1)
  {
    if( range1234 == 1 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_1) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 2 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_2) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 3 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_3) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 4 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_4) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
  }

 if(PGA15 == 5)
  {
    if( range1234 == 1 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_1) &&
        AD5933::setPGAGain(PGA_GAIN_X5)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 2 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_2) &&
        AD5933::setPGAGain(PGA_GAIN_X5)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 3 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_3) &&
        AD5933::setPGAGain(PGA_GAIN_X5)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
      if( range1234 == 4 )
    {
      if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setSettlingCycles(NUM_SCYCLES_1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_4) &&
        AD5933::setPGAGain(PGA_GAIN_X5)))
        {
            Serial.println("FAILED in initialization!");
            lcd.print("loi cai dat he thong");
            while (true) ;
        }
    }
  }
  // Perform calibration sweep
  if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR+1))
     {
    Serial.println("Calibrated!");
    lcd.print("Calibrated");
     }
  else{
    Serial.println("Calibration failed..."); 
    lcd.print("Calibration failed");
  }
    digitalWrite(vin, HIGH);     
    //Serial.println("phase system : ");
    //Serial.print(phase[NUM_INCR+1]);
    delay(5000);

}  




void frequencySweepEasy() {
    // Tạo mảng để lưu trữ dữ liệu
    int real[NUM_INCR+1], imag[NUM_INCR+1];
    

    // Perform the frequency sweep
    if (AD5933::frequencySweep(real, imag, NUM_INCR+1)) {
      // Thực hiện quét tần số
      int cfreq = START_FREQ/1000;
      int phase_imp;
      int in = 0;
      int line = 1;
        lcd.setCursor(0, 0);
        lcd.print("NUM");        
        lcd.setCursor(5, 0);
        lcd.print("Phase"); 
        lcd.setCursor(11,0);        
        lcd.print("Impedance");
      for (int v = 0; v < NUM_INCR+1; v++, cfreq += FREQ_INCR/1000) {
        // In dữ liệu tần số thô
       
        // Serial.print(v);
        // Serial.print(": R=");
        // Serial.print(real[v]);
        // Serial.print("/I=");
        // Serial.print(imag[v]);
        if(real[v] > 0 && imag[v]>0)
        {
        phase_imp = atan((double)imag[v]/(double)real[v]) * (180/PI);
    	  }
    	  else if (real[v] < 0 && imag[v]>0)
    	  {
    		phase_imp = (atan((double)imag[v]/(double)real[v]) * (180/PI)) + 180;
		    }
		    else if (real[v] < 0 && imag[v]<0)
		    {
			  phase_imp =(atan((double)imag[v]/(double)real[v]) * (180/PI)) + 180;
		    }
		    else if (real[v] > 0 && imag[v]<0)
		  {
			  phase_imp = (atan((double)imag[v]/(double)real[v]) * (180/PI)) + 360;	
		  }
        // Tính trở kháng
       // phase_imp = atan((double)imag[i]/(double)real[i]) * (180/PI);
        double magnitude = sqrt(pow(real[v], 2) + pow(imag[v], 2));
        double impedance = 1/(magnitude*gain[v]);
        int phase_lech = phase_imp - phase[v];
        //double phase_deg[i] = phase[i]*(180/PI);
        //double Phase = phase_deg - phase_sys[i];
      

        // Serial.print(" Gainfactor = ");
        // Serial.print(gain[v]);       
        // Serial.print(" magniude =  ");
        // Serial.print(magnitude);       
        // Serial.print("  phase unknow =  ");
        // Serial.print(phase_imp); 
        // Serial.print("  phase system  =  ");
        // Serial.print(phase[v]);
        //Serial.print("  phase =  ");
        Serial.print(phase_lech); 
        Serial.print("    ");
       // Serial.print("  |Z|=");
        Serial.println(impedance);
        if(line > 3)
          line =1;
        lcd.setCursor(0, line);
        lcd.print(v);        
        lcd.setCursor(6, line);
        lcd.print(phase_lech); 
        lcd.setCursor(11,line);        
        lcd.print(impedance);
       
        line++;
        delay(500);
      }
      Serial.println("Frequency sweep complete!");
    } else {
      Serial.println("Frequency sweep failed...");
    }
    
}

void frequencySweepRaw() {
    // Tạo các biến để giữ dữ liệu trở kháng và theo dõi tần số
    int real, imag, v = 0, cfreq = START_FREQ/1000;
    int phase_imp;
    int line = 1;
    // Khởi tạo quét tần số
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // đặt ở chế độ chờ
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // 
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) //bắt đầu quét tần số
         {
             Serial.println("Could not initialize frequency sweep...");
         }

    // Thực hiện quét thực tế
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Nhận dữ liệu tần suất cho điểm tần suất này
        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Could not get raw frequency data...");
        }

        // In ra dữ liệu tần số
        // Serial.print(cfreq);
        // Serial.print(": R=");
        // Serial.print(real);
        // Serial.print("/I=");
        // Serial.print(imag);
        // Serial.print(": R=");
        // Serial.print(real);
        // Serial.print("/I=");
        // Serial.print(imag);
        if(real > 0 && imag>0)
        {
        phase_imp = (atan((double)imag/(double)real) * (180/PI));
    	  }
    	  else if (real < 0 && imag>0)
    	  {
    		phase_imp = (atan((double)imag/(double)real) * (180/PI)) + 180;
		    }
		    else if (real < 0 && imag<0)
		    {
			  phase_imp =(atan((double)imag/(double)real) * (180/PI)) + 180;
		    }
		    else if (real > 0 && imag<0)
		    {
			  phase_imp = (atan((double)imag/(double)real) * (180/PI)) + 360;	
		    }
        // Tính trở kháng
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        double impedance = 1/(magnitude*gain[v]);
        int phase_lech = phase_imp - phase[v];
         if(line > 3)
          line =1;
        
         
        lcd.setCursor(0, line);
        lcd.print(i);        
        lcd.setCursor(6, line);
        lcd.print(phase_lech); 
        lcd.setCursor(11,line);        
        lcd.print(impedance);
       
        line++;
        // Serial.print("  phase =  ");
        Serial.print(phase_lech); 
        // Serial.print("  |Z|=");
        Serial.println(impedance);
  
        // Tăng tần số
        i++;
        cfreq += FREQ_INCR/1000;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
        delay(500);
    }

    Serial.println("Frequency sweep complete!");

    // Đặt chế độ nguồn AD5933 thành chế độ chờ khi kết thúc
    if (!AD5933::setPowerMode(POWER_STANDBY))
        Serial.println("Could not set to standby...");
}

 
long getKeypadIntegerMulti()
{
  long value = 0;                                // the number accumulator
  long keyvalue;                                     // the key pressed at current moment
  int isnum;
  int u = 0;
  int m= 0;
  do
  {
    keyvalue = Mykeys.getKey();                          // input the key
    isnum = (keyvalue >= '0' && keyvalue <= '9'|| keyvalue == '*');       // is it a digit?
    
    if (isnum)
    { 
        m++;
        u++;
        if(keyvalue == '*')
        {
          value = 0;
          lcd.setCursor(7,1);
          lcd.print("             ");
          u = 0;
        }  
  
        else if(u == 0)
        {
            lcd.setCursor(7,1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 1)
        {
            lcd.setCursor(8, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 2)
        {
            lcd.setCursor(9, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 3)
        {
            lcd.setCursor(10, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 4)
        {
            lcd.setCursor(11, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
            
        else if(u == 5)
        {
            lcd.setCursor(12, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }       
        else if(u == 6)
        {
            lcd.setCursor(13, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
        else if(u == 7)
        {
            lcd.setCursor(14, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
        else if(u == 8)
        {
            lcd.setCursor(15, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 9)
        {
            lcd.setCursor(16, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 10)
        {
            lcd.setCursor(17, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 11)
        {
            lcd.setCursor(18, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
         else if(u == 12)
        {
            lcd.setCursor(19, 1);
            lcd.print(keyvalue - '0');
            value = value * 10 + keyvalue - '0';
        }
    }

    
  } while (isnum || !keyvalue);                          // until not a digit or while no key pressed
   
  
  return value;
 

}


//getKeypadInteger
void manhinh()  // HIỂN THỊ MÀN HÌNH CHÍNH
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("   MAN HINH CHINH ");
  lcd.setCursor(0, 2);
  lcd.print("    ->[ Next ] <- ");
}

void menu_tong() // HIỂN THỊ MENU TỔNG
{
  if (congtru_tong == 0)
  {
    lcd.clear();
    lcd.print(">chon tham so");
    lcd.setCursor(0, 1);
    lcd.print(" cai dat he thong");
    lcd.setCursor(0, 2);
    lcd.print(" kiem tra cai dat"); 
    lcd.setCursor(0, 3);
    lcd.print(" hieu chinh va quet"); 
  }
  else if (congtru_tong == 1)
  {
    lcd.clear();
    lcd.print(" chon tham so");
    lcd.setCursor(0, 1);
    lcd.print(">cai dat he thong"); 
    lcd.setCursor(0, 2);
    lcd.print(" kiem tra cai dat"); 
    lcd.setCursor(0, 3);
    lcd.print(" hieu chinh va quet");   
  }
  else if (congtru_tong == 2)
  {
    lcd.clear();
    lcd.print(" chon tham so");
    lcd.setCursor(0, 1);
    lcd.print(" cai dat he thong"); 
    lcd.setCursor(0, 2);
    lcd.print(">kiem tra cai dat"); 
    lcd.setCursor(0, 3);
    lcd.print(" hieu chinh va quet");   
  }
  else if (congtru_tong == 3)
  {
    lcd.clear();
    lcd.print(" chon tham so");
    lcd.setCursor(0, 1);
    lcd.print(" cai dat he thong"); 
    lcd.setCursor(0, 2);
    lcd.print(" kiem tra cai dat"); 
    lcd.setCursor(0, 3);
    lcd.print(">hieu chinh va quet");    
  }
 
}

void chonmenu_tong() // CHỌN MENU TỔNG
{
  switch (congtru_tong) 
  {
    case 0:
      //Không làm gì
      break;
    case 1:
      //Không làm gì
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Stop_freq =");
      lcd.setCursor(11,0);
      lcd.print(STOP_FREQ);
      lcd.setCursor(0,1);
      lcd.print("start_fre =");
      lcd.setCursor(11,1);
      lcd.print(START_FREQ);
      lcd.setCursor(0,2);
      lcd.print("R_Calibrate =");
      lcd.setCursor(13,2);
      lcd.print(REF_RESIST);
      lcd.setCursor(0,3);
      lcd.print("RANGE_");
      lcd.setCursor(6,3);
      lcd.print(range1234);
      lcd.setCursor(8,3);
      lcd.print("PGA_GAIN_X");
      lcd.setCursor(18,3);
      lcd.print(PGA15);
      
      break;
    case 3:
      lcd.clear();
        digitalWrite(yellowled,HIGH);
        digitalWrite(vin, LOW);  
        calibrated();
        delay(5000);
        lcd.clear();
        frequencySweepEasy();
        delay(5000);
      
        digitalWrite(yellowled,LOW);
        exit(0);
      break;
  }
}
void thamso()  // HIỂN THỊ tham số
{
  if (congtru_chonthamso == 0)
  {
    lcd.clear();
    lcd.print(">Start frequency");
    lcd.setCursor(0, 1);
    lcd.print(" Stop frequency"); 
    lcd.setCursor(0, 2);
    lcd.print(" Increment frequency"); 
    lcd.setCursor(0, 3);
    lcd.print(" R Calibrate"); 
  }
  else if (congtru_chonthamso == 1)
  {
   lcd.clear();
    lcd.print(" Start frequency");
    lcd.setCursor(0, 1);
    lcd.print(">Stop frequency"); 
    lcd.setCursor(0, 2);
    lcd.print(" Increment frequency"); 
    lcd.setCursor(0, 3);
    lcd.print(" R Calibrate");    
  }
  else if (congtru_chonthamso == 2)
  {
    lcd.clear();
    lcd.print(" Start frequency");
    lcd.setCursor(0, 1);
    lcd.print(" Stop frequency"); 
    lcd.setCursor(0, 2);
    lcd.print(">Increment frequency"); 
    lcd.setCursor(0, 3);
    lcd.print(" R Calibrate");   
  }
  else if (congtru_chonthamso == 3)
  {
    lcd.clear();
    lcd.print(" Start frequency");
    lcd.setCursor(0, 1);
    lcd.print(" Stop frequency"); 
    lcd.setCursor(0, 2);
    lcd.print(" Increment frequency"); 
    lcd.setCursor(0, 3);
    lcd.print(">R Calibrate");    
  }    
}

void chonthamso() // CHỌN tham số
{
  switch (congtru_chonthamso) 
  {
    case 0:
      lcd.clear();
      lcd.setCursor(0,0);
      Serial.print("star =");
      lcd.print("   Start Frequency");
      lcd.setCursor(0,1);
      lcd.print("value =");
      START_FREQ = getKeypadIntegerMulti();
      lcd.setCursor(0,2);
      lcd.print("start_fre =");
      lcd.setCursor(11,2);
      lcd.print(START_FREQ); 
      Serial.print(START_FREQ);      
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" Stop frequency");
      Serial.print("STOP_frequency =");
      lcd.setCursor(0,1);
      lcd.print("value =");
      STOP_FREQ = getKeypadIntegerMulti();
      lcd.setCursor(0,2);
      lcd.print("Stop_Fre =");
      lcd.setCursor(11,2);
      lcd.print(STOP_FREQ); 
      Serial.print(STOP_FREQ);
      break;
    case 2:
      lcd.clear();
      FREQ_INCR = (STOP_FREQ - START_FREQ) / NUM_INCR;
      lcd.setCursor(0,0);
      lcd.print(" Increment frequency");
      lcd.setCursor(0,1);
      lcd.print("he thong quet ");
      lcd.setCursor(14,1);
      lcd.print(NUM_INCR);
      lcd.setCursor(17,1);
      lcd.print("lan");
      lcd.setCursor(0,2);
      lcd.print("INCRE_FRE =");
      Serial.print(FREQ_INCR);      
      lcd.setCursor(11,2);
      lcd.print(FREQ_INCR);

      break;
    case 3:
      lcd.clear();
      digitalWrite(vin1, HIGH);  
      digitalWrite(vin2, HIGH);  
      lcd.setCursor(0,0);
      lcd.print("     R Calibrate  ");
      lcd.setCursor(0,1);
      lcd.print("value =");
      //REF_RESIST = getKeypadIntegerMulti();
      lcd.setCursor(0,2);
      lcd.print("R_Calib =");
      //lcd.print(REF_RESIST); 
      //Serial.print(REF_RESIST);
      break;      
  }
}

void hethong() //hien thi he thong
{
  if (congtru_hethong == 0)
  {
    lcd.clear();
    lcd.print(">Range");
    lcd.setCursor(0, 1);
    lcd.print(" PGA_Gain");  
  }
  else if (congtru_hethong == 1)
  {
    lcd.clear();
    lcd.print(" Range");
    lcd.setCursor(0, 1);
    lcd.print(">PGA_Gain");    
  }  
}
void chonhethong() // CHỌN MENU 2
{
  switch (congtru_hethong ) 
  {
    case 0:
      
      
      break;
    case 1:
         
      break;
  }
}

void range()
{
  if(congtru_range == 0)
  {
    lcd.clear();
    lcd.print(">Range 1");
    lcd.setCursor(0, 1);
    lcd.print(" Range 2"); 
    lcd.setCursor(0, 2);
    lcd.print(" Range 3"); 
    lcd.setCursor(0, 3);
    lcd.print(" Range 4"); 
  }
  else if(congtru_range == 1)
  {
    lcd.clear();
    lcd.print(" Range 1");
    lcd.setCursor(0, 1);
    lcd.print(">Range 2"); 
    lcd.setCursor(0, 2);
    lcd.print(" Range 3"); 
    lcd.setCursor(0, 3);
    lcd.print(" Range 4"); 
  }
  else if(congtru_range == 2)
  {
    lcd.clear();
    lcd.print(" Range 1");
    lcd.setCursor(0, 1);
    lcd.print(" Range 2"); 
    lcd.setCursor(0, 2);
    lcd.print(">Range 3"); 
    lcd.setCursor(0, 3);
    lcd.print(" Range 4"); 
  }
  else if(congtru_range == 3)
  {
    lcd.clear();
    lcd.print(" Range 1");
    lcd.setCursor(0, 1);
    lcd.print(" Range 2"); 
    lcd.setCursor(0, 2);
    lcd.print(" Range 3"); 
    lcd.setCursor(0, 3);
    lcd.print(">Range 4"); 
  }
}
void chonrange()
{
switch (congtru_range) 
  {
    case 0:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" Range 1");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong");
      range1234 = 1;
      
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" Range 2");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong"); 
      range1234 = 2;
      break;

    case 2:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" Range 3");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong");
      range1234 = 3;
      break;

    case 3:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" Range 4");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong");
      range1234 = 4;
      break;    
  }
}
void PGA_Gain()
{
    if (congtru_PGA == 0)
  {
    lcd.clear();
    lcd.print(">PGA_Gain X1");
    lcd.setCursor(0, 1);
    lcd.print(" PGA_Gain X5");  
  }
  else if (congtru_PGA == 1)
  {
     lcd.clear();
    lcd.print(" PGA_Gain X1");
    lcd.setCursor(0, 1);
    lcd.print(">PGA_Gain X5");   
  }  
}

void chonPGA_Gain()
{
 switch (congtru_PGA) 
  {
    case 0:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" PGA_Gain X1");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong");
      PGA15 = 1;      
      
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print(" PGA_Gain X5");
      lcd.setCursor(0,1);
      lcd.print(" da chon thanh cong");
      PGA15 = 5;
      break;
    
  }
} 
void setup()
{
  Serial.begin(9600);
  pinMode(len, INPUT_PULLUP);
  pinMode(xuong, INPUT_PULLUP);
  pinMode(menu, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  pinMode(Rcal, INPUT_PULLUP);
  pinMode(vin, OUTPUT);
  pinMode(vin1, OUTPUT);
  pinMode(vin2, OUTPUT);
  pinMode(yellowled,OUTPUT);
  pinMode(up_buttonPin, INPUT_PULLUP);  //Internal Pullup on Up button (No external resistor required)
  pinMode(down_buttonPin, INPUT_PULLUP);//Internal Pullup on Down button (No external resistor required)
  
  pinMode(inc_outPin, OUTPUT);
  pinMode(ud_outPin, OUTPUT);
  pinMode(cs_outPin, OUTPUT);
 
 digitalWrite(cs_outPin, HIGH);
  lcd.init(); // Khởi tạo màn hình Màn hình
  lcd.backlight(); // Bật đèn màn hình Màn hình
  lcd.createChar(0, va); // Tạo ký tự đặc biệt &
// Hiện chữ M&E Automation
  lcd.setCursor(6, 0); // Di chuển con trỏ đến cột 1 và hàng 0
  lcd.print("UET");
  lcd.setCursor(9,0 ); 
  lcd.write(byte(0)); // in ký tự đặc biệt đã tạo &.
  lcd.setCursor(10, 0);
  lcd.print("MEMS");
  lcd.setCursor(5, 1);
  lcd.print("EIS Device");
  lcd.setCursor(2, 2);
  lcd.print("1000< Fre <100000");  
  lcd.setCursor(2, 3);
  lcd.print("1000< Imp <100000");
  delay(4000);
  lcd.clear();
  manhinh();
  
}  
//=================================================================================================================

void loop()
{
  char EnterKey = Mykeys.getKey();
  gtlen = digitalRead(len);  
  gtxuong = digitalRead(xuong);  
  gtmenu = digitalRead(menu);
  gtback = digitalRead(back);
  gt_Rcal = digitalRead(Rcal);
  up_buttonState =  digitalRead(up_buttonPin);
  down_buttonState = digitalRead(down_buttonPin);
  raw = analogRead(analogPin);
  

if (gtlen != macdinh)  // NÚT LÊN
{
  if (gtlen == 0) // Khi nhấn nút lên
  {
    if (demtong == 1)   // LÊN Ở MENU TỔNG  
    {
      if (congtru_tong >= 3)
      { 
        congtru_tong= 0;  
      }
      else
      { 
        congtru_tong++;  
      }   
      menu_tong();
    }
    
    else if (demtong == 2 && congtru_tong == 0)   // Lên ở tham số
    {
      if (congtru_chonthamso >= 3)
      { 
        congtru_chonthamso = 0;  
      }
      else
      { 
        congtru_chonthamso++;  
      }   
      thamso();
    }
    
    else if (demtong == 2 && congtru_tong == 1)   // LÊN Ở hệ thống
    {
      if (congtru_hethong >= 1)
      { 
        congtru_hethong = 0;  
      }
      else
      { 
        congtru_hethong++;  
      }   
      hethong();
    }

    else if (demtong == 3 && congtru_hethong == 0 && demhethong == 1)   // lên ở range 
    {
        if (congtru_range >= 3)
      { 
        congtru_range = 0;  
      }
      else
      { 
        congtru_range++;  
      }   
      range();
    }  
    else if (demtong == 3 && congtru_hethong == 1 && demhethong == 1)   // lên ở PGA_GAIN
    {
        if (congtru_PGA >= 1)
      { 
        congtru_PGA = 0;  
      }
      else
      { 
        congtru_PGA++;  
      }   
      PGA_Gain();
    }  

    delay(200);
  }
  macdinh = gtlen;
}
  if (gtxuong != macdinh) // NÚT XUỐNG
{
  if (gtxuong == 0) //Khi nhấn nút xuống
  {
    if (demtong == 1)   // XUỐNG Ở MENU TỔNG
    {
      if (congtru_tong <= 0)
      { 
        congtru_tong = 3;  
      }
      else
      { 
        congtru_tong--;  
      }
      menu_tong();
    }
    
    else if (demtong == 2 && congtru_tong == 0)   // XUỐNG Ở tham số
    {
      if (congtru_chonthamso <= 0)
      { 
        congtru_chonthamso = 3;  
      }
      else
      { 
        congtru_chonthamso--;  
      }
      thamso();
    }
    
    else if (demtong == 2 && congtru_tong == 1)   // XUỐNG Ở hệ thống
    {
      if (congtru_hethong <= 0)
      { 
        congtru_hethong = 1;  
      }
      else
      { 
        congtru_hethong--;  
      }
      hethong();
    }

    else if (demtong == 3 && congtru_hethong == 0 && demhethong == 1)   // Xuống ở Range
    {
        if (congtru_range <= 0)
      { 
        congtru_range = 3;  
      }
      else
      { 
        congtru_range--;  
      }   
      range();
    }  
    else if (demtong == 3 && congtru_hethong == 1 && demhethong == 1)   //xuong ở PGA_GAIN
    {
        if (congtru_PGA <= 0)
      { 
        congtru_PGA = 1;  
      }
      else
      { 
        congtru_PGA--;  
      }   
      PGA_Gain();
    }  
    delay(200);
  }
  macdinh = gtxuong;
}

if (gtmenu != macdinh)    // NÚT MENU
{  
  if (gtmenu == 0) //Khi nhấn nút
  {  
    demtong ++;
    if (demtong == 1) //Ở menu tổng
    { 
      demback = 0;
      menu_tong(); 
    }

    else if (demtong == 2 && congtru_tong == 0) //ở tham số
    {
      demback = 0;
      thamso(); 
      demthamso++;
    }
      else if (demtong == 3 && demthamso == 1) // Chọn tham số
      {
        demback = 0;
        chonthamso();
      }

    else if (demtong == 2 && congtru_tong == 1) //Ở hệ thống
    {
      demback = 0;
      hethong(); 
      demhethong++;
    }
      else if (demtong == 3 && demhethong == 1 && congtru_hethong == 0) // range
      {
        demback = 0;
        range();
        demrange++;
      }
      
      else if(demtong == 4 && demrange == 1 && demhethong >= 1) // chon range
      {
        demback = 0;
        chonrange();
      }    
       else if(demtong == 3 &&  demhethong == 1  && congtru_hethong == 1 )   // PGA
      {
        demback = 0;
        PGA_Gain();
        demPGA++;
      }  
      else if(demtong == 4 && demPGA == 1 && demhethong >= 1) // chon PGA
      {
        demback = 0;
        chonPGA_Gain();
      } 

    else if (demtong == 2 && (congtru_tong == 2 or congtru_tong == 3)) // chọn menu tổng 3 or 4
    {
      demback = 0;
      chonmenu_tong();
    }
      else if (demtong == 3 && (congtru_tong == 2 or congtru_tong == 3)) // Đang chọn menu 3 or 4 nhưng nhấn nút menu lên 3
      {
        demback = 0;
        demtong = 2;
        chonmenu_tong();
      }    

    else if (demtong > 3)
    {
      demtong = 3;
      demback = 0;
    }

    delay(100);
  }
  macdinh = gtmenu;
}
if (gtback != macdinh)    // NÚT BACK
{  
  if (gtback == 0) // Khi nhấn nút
  {
    demback ++;
    if (demback == 1)
    {
      if (demtong == 1 && (congtru_tong == 0 or congtru_tong == 1 
                            or congtru_tong == 2 or congtru_tong == 3)) // TỪ MENU TỔNG TRỞ VỀ MÀN HÌNH
      {
        demtong = demtong - 1;
        demback = 0;
        manhinh();
      }
      else if (demtong == 2 && congtru_tong == 0) // TỪ tham số TRỞ VỀ MENU TỔNG
      {
        demtong = demtong - 1;
        demback = 0;
        demthamso = 0;
        menu_tong();
      }
      else if (demtong == 2 && congtru_tong == 1) // TỪ he thong VỀ MENU TỔNG
      {
        demtong = demtong - 1;
        congtru_tong = congtru_tong - 1; 
        congtru_hethong = 0;
        demhethong = 0;
        demback = 0;
        menu_tong(); 
      }
      else if (demtong == 2 && (congtru_tong == 2 or congtru_tong == 3)) // TỬ MENU 3 OR 4 THOÁT RA MENU TỔNG
      {
        demback = 0;
        demtong = demtong - 1;
        congtru_tong = 0;
        menu_tong();
      }
      else if (demtong == 3 && demthamso >= 1) // TỪ CHỌN- tham số  VỀ  tham số
      {
        demback = 0;
        demtong = demtong - 1;
        digitalWrite(vin1, LOW);  
        digitalWrite(vin2, LOW);  
        congtru_chonthamso = 0;
        thamso();
      } 
      else if (demtong == 3 && demhethong >= 1 ) // Range va PGA  VỀ hệ thống
      {
        demback = 0;
        demtong = demtong -1;
        congtru_hethong = 0;
        demrange = 0;
        demPGA= 0;
        hethong();
      }      
      else if(demtong == 4 && demrange >= 1 && demhethong >= 1 ) // từ chon range trở về hàm range
      {
        demback = 0;
        demtong = demtong -1;
        congtru_range = 0;
        range();  
              
      }
      else if(demtong == 4 && demPGA >= 1  && demhethong >= 1) // tu chon PGA ve PGA
      {
        demback = 0;
        demtong = demtong -1;
        congtru_PGA = 0;
        PGA_Gain();
      }
    }

    else
    {
      demback = 0;
    }
    delay(100);
  }
  macdinh = gtback;
}
 if(demtong == 3 && demthamso == 1)
 {
   if(congtru_chonthamso == 3)
   {
       if(!up_buttonState)
  {
    digitalWrite(cs_outPin, LOW);
    delay(5);
    digitalWrite(ud_outPin, HIGH);
    delay(5);
    digitalWrite(inc_outPin, HIGH);
    delay(5);
    digitalWrite(inc_outPin, LOW);
    delay(5);
    digitalWrite(inc_outPin, HIGH);
    digitalWrite(cs_outPin, HIGH);
    

  }
  else if (!down_buttonState)
  {
    digitalWrite(cs_outPin, LOW); 
    delay(5);
    digitalWrite(ud_outPin, LOW);
    delay(5);
    digitalWrite(inc_outPin, HIGH);
    delay(5);
    digitalWrite(inc_outPin, LOW);
    delay(5);
    digitalWrite(inc_outPin, HIGH);
    digitalWrite(cs_outPin, HIGH);
  }
  delay(100);
    if(raw)
    {
   buffer = raw * Vin;
    Vout = (buffer)/1024.0;
    buffer = (Vin/Vout) - 1;
    R2 = R1 * buffer;

    

    lcd.setCursor(0, 1);
    lcd.print("Value = ");
    lcd.setCursor(8, 1);
    lcd.print(R2);
    if(R2 < 100000)
    {
     lcd.setCursor(13,1);
     lcd.print("      ");
    }
    if(R2<10000)
    {
      lcd.setCursor(12,1);
      lcd.print("    ");
    }
    if(R2<1000)
    {
      lcd.setCursor(11,1);
      lcd.print("      ");
    }
    if(R2<100)
    {
      lcd.setCursor(10,1);
      lcd.print("      ");
    }
    if(R2<10)
    {
      lcd.setCursor(9,1);
      lcd.print("       ");
    }
     delay(300);
     if(gt_Rcal != macdinh)
      {
         if(gt_Rcal == 0)
         {
            REF_RESIST = R2;
            lcd.setCursor(9,2);
            lcd.print(REF_RESIST);
         }
      }
   }   
 }
 }

   
 }

