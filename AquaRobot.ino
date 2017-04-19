#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define BatteryPin 0

//通信用オブジェクト
MPU6050 accelgyro;

//送信Data
int16_t ax, ay, az;
int16_t gx, gy, gz;

//バッテリー残量
double val;

//温度
double temp;

//モーター使用ピン
int8_t mPin[]={3,5,6,9};


////////////////////////////////////////////////////////////////////////////////////////////////////

//データ送信関数
void sendData(){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    temp=(accelgyro.getTemperature()+12412.0)/340.0;
    //offset 340[LSB/℃]*35[℃]+521[LSB]=12412[LSB]

    //センサー受信エラー
    if(ax==0&&ay==0&&az==0){
        Serial.println("{\"Error\": \"Sensor error\"}");
        accelgyro.initialize();
        
        Serial.println("{\"Setting\": \"Restart\"}");
        if(accelgyro.testConnection()){
            Serial.println("{\"Setting\": \"MPU6050 connection successful\"}");
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            temp=(accelgyro.getTemperature()+12412.0)/340.0;
        }
        else{
            Serial.println("{\"Error\": \"MPU6050 connection failed\"}");
            return;
        }
    }
    
    //加速度は[G]単位=9.8[m/s^2]
    //オフセット ax:±50mG, ay:±50mG, az:±80mG
    Serial.print("{");
    Serial.print("\"ax\": ");   Serial.print(ax/16384.0);
    Serial.print(",\t\"ay\": ");Serial.print(ay/16384.0);
    Serial.print(",\t\"az\": ");Serial.print(az/16384.0);

    //ジャイロは[°/s]単位
    //オフセット ±20[°/s]
    Serial.print(",\t\"gx\": ");   Serial.print(gx/131.0);
    Serial.print(",\t\"gy\": ");Serial.print(gy/131.0);
    Serial.print(",\t\"gz\": ");Serial.print(gz/131.0);

    Serial.print(",\t\"temp\": ");
    Serial.print(temp);
    Serial.println("}");
}


//モータ制御関数
void MotorControl(String Data){
    Data.replace("Motor:","");
    int i=Data.toInt()-1;
    
    //受信エラー
    if(i<0){
        Serial.println("{\"Error\": \"Receive error\", \"State\": \"Motor\"}");
    }

    //受信成功時
    else if(i<sizeof(mPin)){
        //不要な情報を消去
        int sp=Data.indexOf(" ");
        Data=Data.substring(sp+1);
        Data.replace("Speed:","");
        sp=Data.toInt();
        
        analogWrite(mPin[i],sp);

        Serial.print("{\"MotorPin\": ");
        Serial.print(mPin[i]);
        Serial.print(", \"Speed\": ");
        Serial.print(sp);
        Serial.println("}");
    }

    //配列外参照
    else{
        Serial.println("{\"Error\": \"Out of index\"}");
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
    Wire.begin();
    Serial.begin(9600);

    //モーター出力設定
    for(int i=0 ; i < sizeof(mPin) ; i++){
        pinMode(mPin[i], OUTPUT);
    }

    //センサー接続待機中
    Serial.println("{\"Setting\": \"Initializing I2C devices...\"}");
    accelgyro.initialize();

    //センサー接続テスト
    Serial.println("{\"Setting\": \"Testing device connections...\"}");
    Serial.println(accelgyro.testConnection() ? "{\"Setting\": \"MPU6050 connection successful\"}" : "{\"Error\": \"MPU6050 connection failed\"}");
}


void loop() {    
    if(Serial.available()>0){
        char buffer[50]={};
        
        if(Serial.readBytesUntil("\n",buffer,50)>0){
            String Data = String(buffer);

            //モーター制御
            if (Data.startsWith("Motor:")){
                MotorControl(Data);
            }

            //センサーのデータを送信
            else if(Data.startsWith("Sensor")){
                sendData();
            }

            //バッテリー残量
            else if(Data.startsWith("Battery")){
                val=analogRead(BatteryPin)/1024.0*5.0;
                Serial.print("{\"Battery\": ");
                Serial.print(val);
                Serial.println("}");
            }

            //よくわからないデータを受信した時
            else{
                Serial.println("{\"Error\": \"Receive error\", \"State\": \"General\"}");
            }
        }
    }    
}
