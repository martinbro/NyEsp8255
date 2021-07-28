/****************************************************************************************************************************
	ESP8266-Minimal-Client: Minimal ESP8266 Websockets Client

	This sketch:
				0. Connects to a twin board.
				1. Connects to a WiFi network
				2. Connects to a Websockets server
					 a. Send Calculated val to a Webserver using Websockets
					 b. Get Waypoints info& PID values using Websockets
				3. Connects to BNO055
				4. Connects to GPS
				5. Connects to Servo
				6. Connects to Remote Control Device (not implemented)
					 Set Rudder Opreration in two MODES MANUAL and AUTO.
				7. Calculate course using datafusion
				8. Calculate position and speed using dead reckoning (not implemented)
				9. Calculate rudder position using PID (not implemented) only in AUTO MODE
				10. Stores Data in External storige (not implementet)


		NOTE:
		The sketch dosen't check or indicate about errors while connecting to
		WiFi or to the websockets server. For full example you might want
		to try the example named "ESP8266-Client".

	Hardware:
				An ESP8266 board.
				GPS
				BNO055
				Servo (rudder)

	Originally Created  : 15/02/2019
	Original Author     : By Gil Maimon
	Original Repository : https://github.com/gilmaimon/ArduinoWebsockets

*****************************************************************************************************************************/

#include <WebSockets2_Generic.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <strings_en.h>
#include <WiFiManager.h>

/* Konstanter ************************************************ */
static const uint8_t RORpwm = 14; //D5 som udlæg output
static const uint8_t RORpwm1 = 12; //D5 som udlæg output

// static const int SCL_gyro = 5, SDA_gyro = 4;// Gyro: D2 (SCL), D1(SDA)
static const int RXPin = 13, TXPin = 15;// GPS: D8 (RX), D7(TX)
static const uint16_t GPSBaud = 9600;

/* Globale variable ************************************************ */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
uint16_t WiFi_DELAY_MS = 100;
static uint32_t t0 = 0;//, t0_main_loop = 0 ;
static uint32_t t1 = 0;//, t1_main_loop = 0; //, tWiFi = 0;

// Der er performanceforbedring ved at anvende simple variable frem for structs
float mx,my,mz = 0.0;
float gx,gy,gz,dt = 0.0;
float rotx,roty,rotz = 0.0;
float ax,ay,az = 0.0;
float dN,dE = 0.0;// Estimeret pos
float dvx,dvy = 0.0;

float kurs,kursRaw,roll,rollRaw,pitch,pitchRaw = 0.0;
float K = 0.99;
float MISVISNING = 3.75;
float kursGyroStabiliseret;
uint8_t systemC, gyroC, accelC, magC = 0;
float bnoData[10];
float lg[]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0} ;
float br[]={-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0} ;
short antalWP = 0;
short activeWP = 0;

float brGps; 
float lgGps;
float tiGps;

float sp_kurs = 0.0;
float e = 0.0;
float P = 1;
float I = 0;
float D = 0;
float ror = 0.0;

/* Initialiserer moduler ******************************************* */
TinyGPSPlus gps;
SoftwareSerial ss(RXPin,TXPin);

using namespace websockets2_generic;
WebsocketsClient client;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);//(id,address)

void setup()
{
		//Seriel forbindelse
		Serial.begin(115200);
		while (!Serial) { ;}//venter til vi har kontakt til serielporten
		//kommunikation mellem GPS og ESP8266
		ss.begin(GPSBaud);
		
		//PWM-forbindelser
		//analogWriteResolution(10);//Setter 2^10 = 1024 resolution op 
		
		int ip = initConnectToWifi();

		client.onMessage(onMessageCallback);
		client.onEvent(onEventsCallback);
		// connect to Deno-server
		if(ip==0){ client.connect("192.168.137.1", 8000, "/ws");}//Min laptop Hotspot
//  if(ip==1){ client.connect("192.168.87.155", 8000, "/ws");}//Hjemme
		if(ip==1){ client.connect("192.168.8.220", 8000, "/ws");}//Marnav?
		if(ip==2){ client.connect("192.168.43.170", 8000, "/ws");}//HUAWEI?
		// Send a ping
		client.ping("gps");
		initBNO055(bno);

}
int i=0;
unsigned long tWiFi = millis();
void loop()
{   
	if(getBNO055val()){
		if((millis()-tWiFi) > WiFi_DELAY_MS){//begrænser netværkstrafik
			// Serial.print("send data, millis()-tWiFi: ");
			// Serial.print((millis()-tWiFi));
			// Serial.print(" WiFi_DELAY_MS: ");
			// Serial.println(WiFi_DELAY_MS);
			sendBNOdata();
			tWiFi = millis();
		}
	}
	
	while (ss.available() > 0){
		if (gps.encode(ss.read())){ 
			i++;
			if (gps.location.isValid()){
				if(i%9==0 ){//Primitiv netværks begrænsning
					//gemmer positionsoplysninger
					i=0;
					brGps = gps.location.lat();
					lgGps = gps.location.lng();
					tiGps = gps.time.hour()*3600+gps.time.minute()*60+gps.time.second()+gps.time.centisecond()/100;
					sendGPSdata(gps);       
				}
			}
		} 
	}
	
	client.poll();
		
}

/* ** FUNKTIONER ** FUNKTIONER ** FUNKTIONER ** FUNKTIONER ** */

//////////////////////Kompas///////////////////////////////

void initBNO055(Adafruit_BNO055 bno){
	/* Initialise the sensor */
	delay(1000);
	if (!bno.begin()){

		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(1000);
		ESP. restart();
	//  while (1);
	}
	Serial.print("Succesfuld forbindelse til BNO055");
	delay(1000);
	bno.setExtCrystalUse(true); /* Bruger microprocessorens  clock */ 
}

bool getBNO055val(){
		/* Get a new sensor vector*/
		if( (millis()-t0) < BNO055_SAMPLERATE_DELAY_MS)return false;

		imu::Vector<3> g = bno.getVector( Adafruit_BNO055::VECTOR_GYROSCOPE);
		imu::Vector<3> m = bno.getVector( Adafruit_BNO055::VECTOR_MAGNETOMETER);
		imu::Vector<3> a = bno.getVector( Adafruit_BNO055::VECTOR_ACCELEROMETER);

		/* Beregner tidsdifferencen dt mellem læsninger */
		t1 = millis(); 
		if(t0==0){t0 = t1;}//initialiserer
		dt =(t1-t0);
		dt =dt/1000.0; /* i sek. */
		t0 = t1;

		/* Henter data fra BNO055 */
		/* NB! akser fra 'euler'-koordinater til 'fly-koordinater' (dvs. x-akse frem & z-akse NEDAD!) dvs y- og z- akser skifter fortegn) 
		* Årsag: for at få pos ROT om z-akse i samme retn. som kompassets pos. retn)
		* NB! acc er neg da BNO055 måler reaktionskraft.
		*/
		rotx = g[0];
		roty = -g[1];
		rotz = -g[2];

		ax = -a[0];
		ay = -(-a[1]);
		az = -(-a[2]); 

		mx = m[0];
		my = -m[1];
		mz = -m[2];

		//Beregner gyroens kurs fra Rate Of Turn
		gx = gx + rotx*dt;
		gy = gy + roty*dt;
		gz = gz + rotz*dt;

		// Roll, Pitch og Yaw (kurs) beregnes - bare trigonometri
		float RollRaw = atan2(ay,az); //rollRaw i radianer
		rollRaw = RollRaw*180/PI;
		pitchRaw = atan2(-ax,(ay*sin(RollRaw)+az*cos(RollRaw)))*180/PI; 
		kursRaw = atan2(-my,mx)*180/PI - MISVISNING;

		//Comperatorfilter på roll, og pitch, 99% gyro, 1% acc
		float k=0.99;//procent gyro
		roll = (roll + rotx*dt)*k + rollRaw*(1-k); 
		pitch = (pitch + roty*dt)*k + pitchRaw*(1-k);

		//'Gyrostabiliserede' værdier
		//roll & pitch i radianer
		float Roll = roll*PI/180;
		float Pitch =pitch*PI/180;

		//tilt kompenseret kurs.(Jeg kan vise dig beregningen hvis du er interesseret Jørgen - fås direkte ud fra rotationsmatriserne for roll og pitch)  
		//(Findes også mange steder på nettet! Pas dog på wikipiedia - der har de byttet om på roll, pitch og yaw... Hmmm det ligner dem ellers ikke...) 
		// NB! her anvendes de gode! værdier for roll og pitch i radianer!
		float X = mx*cos(Pitch) + mz*sin(Pitch);
		float Y = mx*sin(Roll)*sin(Pitch) + my*cos(Roll) - mz * sin(Roll)*cos(Pitch);
		kursGyroStabiliseret = (atan2(-Y,X)*180/PI);
		float gyrokurs = kurs +rotz*dt;

		//Beregner hastighedsændringer fra accelerometret
		dvx = (ax*cos(Pitch) + az*sin(Pitch))*dt;
		dvy = (ax*sin(Roll)*sin(Pitch) + ay*cos(Roll) - az * sin(Roll)*cos(Pitch))*dt;

		// løser et fjollet diskontinuitetsproblem mellem gyro og mag. (Første del)
		// får mag til at være kontinuært over 180
		if(gyrokurs - kursGyroStabiliseret < -180){
				while (gyrokurs - kursGyroStabiliseret < -180){
						kursGyroStabiliseret = kursGyroStabiliseret -360.0;
				}
		}
		else if(gyrokurs - kursGyroStabiliseret > 180){
				while (gyrokurs - kursGyroStabiliseret > 180){
						kursGyroStabiliseret = kursGyroStabiliseret +360.0;
				}
		}
		kurs = (K*gyrokurs + (1-K)*kursGyroStabiliseret);

		
		//Auto kalibreringens status: (integer) 0=lavest niveau (forkast data), 3=højste niveau (fuldt kalibreret data)
		bno.getCalibration(&systemC, &gyroC, &accelC, &magC);

		
		//KUN HVIS GPS SIGNALER
		if(brGps>0){ 
				//Bestikregning
				dN += cos(-kurs*PI/180)*dt; // pos
				dE += -sin(-kurs*PI/180)*cos(brGps)*dt;
				// Serial.print("Bestik: "); Serial.print(dN); Serial.print(", "); Serial.println(dE);

				if(antalWP>0){
						float br_forandring = br[activeWP]- brGps;
						float afvigning = (lg[activeWP]  - lgGps)*cos(brGps*PI/180);//bruger bredde i stedet for middelbredde
						sp_kurs = atan2(afvigning,br_forandring)*180/PI;

						Serial.print(" , Set Point: ");  Serial.print(sp_kurs,6);
						Serial.print(" , PV kurs: ");  Serial.print(kurs,6); 
						float e = sp_kurs - kurs;
						while(e>180){e = e-360;};
						while(e<-180){e = e + 360;};
						Serial.print(" , e: ");  Serial.print(e,6);
						ror = P*e;
						if(ror < -45){ror=-45;};
						if(ror > 45){ror = 45;};//ror begrænsning
						
						int udlg = int (90 + 2*round(ror));//ganger med 2 da der er er noget i vejen med min servo
						Serial.print(" , udlaeg: ");  Serial.println(udlg);
						analogWrite(RORpwm,udlg);
						analogWrite(RORpwm1,udlg);
			
				}
		}

	//delay(BNO055_SAMPLERATE_DELAY_MS);
	return true;

}
void sendBNOdata(){
		String str = "bno,";
		str += kurs; str += ",";//0
		str += roll;str += ",";//1
		str += pitch; str += ",";//2
		str += String(dt,3); str += ",";//3
		str += gyroC*1000+accelC*100+magC*10+systemC; str += ",";//4
		str += String(kursRaw,1); str += ",";//5
		str += kursGyroStabiliseret; str += ",";//6
		str += sp_kurs; str += ",";//7
		str += ror; //8
		client.send( str);
}
////////////////////// GPS ////////////////////////////////
void sendGPSdata(TinyGPSPlus gps){
	if(gps.location.isValid()){


		//værdier sendes som semikolonsepereret streng - csv-stil
		String str = "gps,";//0
		str += String(gps.location.lat(),6);str += ",";//1
		str += String(gps.location.lng(),6);str += ",";//2
		str += gps.hdop.hdop(); str += ",";//3
		str += gps.satellites.value();str += ",";//4
		str += gps.course.isValid()? gps.course.deg():NAN;str += ",";//5
		str += gps.speed.isValid()? gps.speed.kmph():NAN ;//6
		
		client.send(str);	
	}
}

////////////////////// WiFi + websoket2 //////////////////////////
int initConnectToWifi(){
	// ESP8266 Connect to wifi
 
	delay(5000);
	for (byte j = 0; j < 3; j++) {
		
		if(j==0){
			WiFi.begin("SKIB1","u530}8T9");
			Serial.println("Prøver at forbinde til LAPTOP-RJIJOOL9 4301");
			}
		if(j==1){
			WiFi.begin("Marnav","");
			Serial.println("Prøver at forbinde til Marnav");
			}
		if(j==2){
			WiFi.begin("HUAWEI","1q2w3e4r");
			Serial.println("Prøver at forbinde til mobiltelefon");
		}

		for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) 
		{
			Serial.print(".");
			delay(1000);
		}
		if (WiFi.status() == WL_CONNECTED) {
			Serial.println("Succesfuld forbindelse til WiFi");
			return j;
			}
	}
	// Check if connected to wifi
	if (WiFi.status() != WL_CONNECTED) 
	{
		Serial.println("No Wifi!");
		return -1;
	}
	WiFi.setAutoReconnect(true);
	WiFi.persistent(true);
	//Serial.print("Connected to Wifi, Connecting to WebSockets Server: ");
	//Serial.println(websockets_server_host);
}

void onMessageCallback(WebsocketsMessage message) 
{
	String besked = message.data();
	
	int r=0; //besked nr
	// int nr = 0;
	bool nyPos=false;

	int delay_val;
	float lg_wp, br_wp;
	float k_value = 0;
	float misvisning = 3.75; //Default Marstal: https://www.magnetic-declination.com/

	char nr = besked.charAt(0);
	Serial.print("Got Message: ");
	Serial.print(message.data());
	Serial.print(" ,nr: ");
	Serial.println(nr);
	/**
	 * Ny ide:
	 *  besked er af formen:
	 * 1. "a54.8743326:10.469032,54.8743326:10.469032"
	 * 2. "b123.456"
	**/ 
      int startnr = 1;
      int midt = 0;
      int pos_nr = 0;
	switch (nr)
    {
        case 'a':  //waypoint

           	for(int i=1;i<besked.length();i++){
				if(besked[i] == ':'){ midt = i;}
				if(besked[i] == ','){
					br[pos_nr] = besked.substring(startnr,midt-startnr).toFloat();
					lg[pos_nr] = besked.substring(midt + 1,i-(midt+1)).toFloat();
					Serial.printf("way points: br:%8.5f lg:%8.5f",br[pos_nr],lg[pos_nr]);
					pos_nr++;
					startnr = i+1;
				}
           	}
			while (pos_nr<10){ //fjerner evt. gamle wp - hvis rettet
				br[pos_nr] = -1.0;
				lg[pos_nr] = -1.0;
			}
           break;
        case 'b'://rate

     		delay_val = besked.substring(1,besked.length()).toInt();
			Serial.print("delay_val ");Serial.println(delay_val);
	 		if(delay_val>17) {
				WiFi_DELAY_MS = delay_val;
				Serial.print("WiFi_DELAY_MS : ");Serial.println(WiFi_DELAY_MS);
			}
			break;
		case 'c'://K-val
			k_value = besked.substring(1,besked.length()).toFloat();
			if(k_value<0.0 ) break;
			if(k_value>1.0) break;
			K = k_value;
			Serial.print("K: ");Serial.println(K);
			break;  
		case 'd'://K-val
			misvisning = besked.substring(1,besked.length()).toFloat();
			if(misvisning<0.0 ) break;
			if(misvisning>180) break;
			MISVISNING = misvisning;
			Serial.print("Misvisning: ");Serial.println(MISVISNING);
			break;  
    }
	
	if(nyPos){ //Hvis der er en ny pos:
		if (lg_wp<0) {//Initialiserer array hvis der sendes et waypoint med neg længde
			int  i= 0;
			while(i <10){
				lg[i] = -1,0;
				br[i] = -1,0;
				i++;
			}
			antalWP = 0;
			activeWP = 0;
		}else{ //tilføjer wp
			lg[antalWP]= lg_wp;
			br[antalWP]= br_wp;
			antalWP++;
		}
		nyPos=false;
	}
 
	i= 0;
	while(i < antalWP ){
	 Serial.print(br[i],6);
	 Serial.print(", "); 
	 Serial.println(lg[i],6); 
	 i++; 
	}
	Serial.println();

}
void onEventsCallback(WebsocketsEvent event, String data) 
{
	if (event == WebsocketsEvent::ConnectionOpened) 
	{
		Serial.println("Connnection Opened");
		Serial.println(data);
		
	} 
	else if (event == WebsocketsEvent::ConnectionClosed) 
	{
		Serial.println("Connnection Closed");
	} 
	else if (event == WebsocketsEvent::GotPing) 
	{
		Serial.println("Got a Ping!");
	} 
	else if (event == WebsocketsEvent::GotPong) 
	{
		Serial.println("Got a Pong!");
	}
}
