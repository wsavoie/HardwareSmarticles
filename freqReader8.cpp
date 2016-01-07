#include <avr/io.h>
#include <avr035.h>
#include <Servo8Bit.cpp>
#define F_CPU 8000000UL
#include <util/delay.h>
#define PRESCALE_2    1
#define PRESCALE_4    2
#define PRESCALE_8    3
#define PRESCALE_16   4
#define PRESCALE_32   5
#define PRESCALE_64   6
#define PRESCALE_128  7
#define REFVCC        0x00
//#define REFP0         0x40 //attiny datasheet pg 134
#define myabs(n) ((n) < 0 ? -(n) : (n))
#define stressMoveThresh 2

//https://github.com/kehribar/proper-tiny-arduino/blob/master/pta/analog.c
// ATtiny45/85 Pin map
//                        +-\/-+
// Reset/Ain0 (D 5) PB5  1|o   |8  Vcc
//       Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 7
//       Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
//                  GND  4|    |5  PB0 (D 0) pwm0 <-- connect LED here
//                        +----+
uint8_t matchCount = 0;
uint16_t prevVal = 0;
uint8_t minn = 0; uint8_t maxx = 180; uint8_t midd = 90;
uint16_t curr = 0;

int oldVal = 0;
uint16_t newVal = 0;
uint16_t rangeType=0;
int diff = 0;
int meanCurr = 0;
uint16_t midPtCross = 0;
uint16_t micMean = 512;
uint16_t const del = 200;
// keep value a multiple of 2 and use 
//shift operator to divide by that amount 2^samps
uint16_t samps = 9; 
uint8_t thresh = 25;
long vccVal = 0;
uint8_t stress = 0;
uint8_t stressCount =0;

  
//attiny datasheet 17.13.1 p134
//when using analog pins you must use pin value not port
uint8_t LED   = PB3;//0;//0;
uint8_t SP2   = PB1;//1;//1;
uint8_t mic   = 1;//A1;//int mic = A1;P2 //electret //or 7
uint8_t currP = 3;//3;//int currP = A3;P3
uint8_t SP1   = PB4;//4; //4;
uint16_t oldTime;

  Servo8Bit S1;  //create a servo object.
  Servo8Bit S2;
//functions

void delay_ms(uint16_t Val);
void moveMotor2(uint8_t pos);
long readvcc();
void currentRead(uint16_t mm);
void getRange(uint16_t ftVal);
void initADC(uint8_t PRESCALE);

uint16_t ReadADC(uint8_t channel,uint8_t refs);
uint16_t mapp(double in, double in_min,double in_max, double out_min, double out_max);
//inline void digWrite(int channel, bool state);
int main(void)
{
  //DDRB 0=analog 1= analog
  DDRB = 0xff; //set all pins to input
  DDRB = (1 << SP1) | (1 << SP2) | (1 << LED);//Set pin 0,4,1
  CLEARBIT(DDRB,currP);
  CLEARBIT(DDRB,mic);
  CLEARBIT(PORTB, LED); //turn off
  //  pinMode(SP2, OUTPUT);
  //  pinMode(SP1, OUTPUT);
  //  pinMode(currP, INPUT);
  //  pinMode(mic, INPUT);
  //  pinMode(LED, OUTPUT);

  initADC(PRESCALE_16);

  S1.attach(SP1);
  S2.attach(SP2);
  // ADMUX |= _BV( REFS1 );  // Set Ref voltage

while(1)
{
  midPtCross = 0;
  meanCurr = 0;
  micMean = 0;
  //int meanVolt = 1024/5 * .220 = 45
  //curr = ReadADC(currP,REFVCC);
 
  //newVal=analogRead(mic);
  //meanCurr = meanCurr + curr;
  //vccVal=readvcc()/1000.0;
  //micMean=mapp(1.65/2.0,0.0,vccVal,0.0,1023.0); //337 3.3/2 //3.7/2, 4.0/2
   //micMean=mapp(3.3/2.0,0.0,vccVal,0.0,1023.0); //337 //3.7/2, 4.0/2
 
  //if (1<<samps==512)
  //{
	////SETBIT(PORTB,LED);
 // }
 //else
  //{
	//CLEARBIT(PORTB,LED);
	//}
////////////

for (int j = 0; j < 1<<6; j++)
	{
		micMean = micMean+ReadADC(mic,REFVCC);
	}
		micMean >>= 6; // take average of 1>>6 values to determine microphone mean
	 newVal = ReadADC(mic,REFVCC);
	 
	 ////////////////
	 //after new smarticle I don't need above loop can just use 512
  for (int i = 0; i < 1<<samps; i++)
 {
	oldVal  = newVal;
    curr    = ReadADC(currP,REFVCC);
	newVal  = ReadADC(mic,REFVCC);
    meanCurr = meanCurr + curr;
	diff = (newVal > micMean) - (oldVal > micMean);
    //diff = abs(diff);
	diff = myabs(diff);	
   //diff = (diff ^ (diff >> 15)) - (diff >> 15);
    //CLEARBIT(diff,15);
	midPtCross = midPtCross + diff;
 }
 
  //bitshift divide by sample, meancurr=meancurr/(2^samps)
  meanCurr >>= samps;
  currentRead(meanCurr);
  if(stressCount<stressMoveThresh || rangeType==6 || rangeType==7) //if previous moves were 6 or 7, continue without stress
  {
  	getRange(midPtCross);
  }
  _delay_ms(del);
}
return 0;
}
long readvcc()//http://digistump.com/wiki/digispark/quickref
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference

  ADMUX = _BV(MUX3) | _BV(MUX2);
  _delay_ms(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (CHECKBIT(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
   uint8_t high = ADCH; // unlocks both
   long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void currentRead(uint16_t mm) //fix and determine way to make current read amount more related to current rather than heuristic
{
  //if (millis() - 500 > oldTime)
 //{
    if (mm < 10)
    {
      //write led low  //digitalWrite(LED, LOW);
      //CLEARBIT(PORTB,LED);//PORTB &= ~(1 << LED);
	  
      //moveMotor2(stress);
	  stressCount = 0;
    }
    else
    {
      //write led high     digitalWrite(LED, HIGH);
      //SETBIT(PORTB,LED);//PORTB |= (1 << LED);
	  stressCount++;
	  if(stressCount>=stressMoveThresh && rangeType!=6 && rangeType!=7)
	  {
		(stress > 1 ? stress=0 : (stress++));
		moveMotor2(stress);
		
	  }	
	  _delay_ms(del);
    }
    //_delay_ms(del);//or here
  //}

}

 //if lower than 150 , else turn off
void light(uint16_t ftVal)
{ ftVal<150 ? SETBIT(PORTB,LED):CLEARBIT(PORTB,LED);}
 
void getRange(uint16_t ftVal)// at thresh = 50, thresh = 216-433-650
{
	rangeType=0; 
	
	if (ftVal < thresh)                            
		rangeType = 0;
	//else if ((ftVal >= thresh)&&(ftVal <2*thresh)) //216
	//	rangeType = 2;
	else if ((ftVal >= thresh)&&(ftVal <2*thresh)) //216
		rangeType = 0;
	else if ((ftVal>= 2*thresh)&&(ftVal< 3*thresh))//216-433
		rangeType = 2;
	else if ((ftVal>=3*thresh) && (ftVal<4*thresh))//433-650
		rangeType = 3;
	else if ((ftVal>=4*thresh) && (ftVal<5*thresh))//433-650
		rangeType = 4;
	else if ((ftVal>=5*thresh) && (ftVal<6*thresh))//433-650
		rangeType = 5;
	else if ((ftVal>=6*thresh) && (ftVal<7*thresh))//433-650
		rangeType = 6;
	else
		rangeType = 7;

  if (rangeType != prevVal)
  {
    prevVal = rangeType;
    matchCount=0;
    _delay_ms(del);
    return;
  }
  matchCount++;
  prevVal = rangeType;
  
  if(matchCount < 2 )
  {
    _delay_ms(del);
    return;
  }
  moveMotor2(rangeType);
}
void moveMotor2(uint8_t pos) //break method into chunks to allow "multithreading"
{
  switch (pos)
  {
    case 0:
      S1.writeMicroseconds(1500);
      S2.writeMicroseconds(1500);
      break;
	case 1:
      S1.writeMicroseconds(1500 - 300);
      S2.writeMicroseconds(1500 + 300);
      break;
    case 2:
      S1.writeMicroseconds(1500 + 300);
      S2.writeMicroseconds(1500 - 300);
      break;
    case 3:
      S1.writeMicroseconds(1500 + 900);
      S2.writeMicroseconds(1500 - 900);
      break;
	case 4:
      S1.writeMicroseconds(1500 - 900);
      S2.writeMicroseconds(1500 + 900);
      break;
	case 5:
      S1.writeMicroseconds(1500 + 900);
      S2.writeMicroseconds(1500 + 900);
      break;
    case 6:
      S1.writeMicroseconds(maxx * 10 + 600);
      S2.writeMicroseconds((minn) * 10 + 600);
      _delay_ms(del);

      S1.writeMicroseconds(maxx * 10 + 600);
      S2.writeMicroseconds((maxx) * 10 + 600);
      _delay_ms(del);

      S1.writeMicroseconds(minn * 10 + 600);
      S2.writeMicroseconds((maxx) * 10 + 600);
      _delay_ms(del);

      S1.writeMicroseconds(minn * 10 + 600);
      S2.writeMicroseconds((minn) * 10 + 600);
      _delay_ms(del);
      break;
    case 7:
      //S1.writeMicroseconds(maxx * 10 + 600);
      //S2.writeMicroseconds((midd) * 10 + 600);
      //_delay_ms(del);
      //S1.writeMicroseconds(midd * 10 + 600);
      //S2.writeMicroseconds((maxx) * 10 + 600);
      //_delay_ms(del);
      //S1.writeMicroseconds(minn * 10 + 600);
      //S2.writeMicroseconds((midd) * 10 + 600);
      //_delay_ms(del);
      //S1.writeMicroseconds(midd * 10 + 600);
      //S2.writeMicroseconds((minn) * 10 + 600);
      //_delay_ms(del);
      //S1.writeMicroseconds(maxx * 10 + 600);
      //S2.writeMicroseconds((midd) * 10 + 600);
	  
	  
	  S1.writeMicroseconds(maxx * 10 + 600);
      S2.writeMicroseconds((midd) * 10 + 600);
      _delay_ms(del*2);
      S1.writeMicroseconds(midd * 10 + 600);
      S2.writeMicroseconds((midd) * 10 + 600);
      _delay_ms(del*2);
      S1.writeMicroseconds(midd * 10 + 600);
      S2.writeMicroseconds((minn) * 10 + 600);
      _delay_ms(del*2);
      S1.writeMicroseconds(maxx * 10 + 600);
      S2.writeMicroseconds((minn) * 10 + 600);
      //S1.writeMicroseconds(maxx * 10 + 600);
      //S2.writeMicroseconds((midd) * 10 + 600);
	  
      //_delay_ms(del/2);
      break;
	case 8:
      S1.writeMicroseconds(1500 + 900);
      S2.writeMicroseconds(1500 - 900);
      break;
  }
}
void initADC(uint8_t PRESCALE)
{
  //attiny datasheet 17.13.2 p136
  //ADCSRA = (1 << ADEN) | (1 << ADPS2)| (1<<ADPS1); //Rrescaler div =64
  SETBIT(ADCSRA,ADEN);
  //#ADCSRA |= PRESCALE;
  SETBITMASK(ADCSRA,PRESCALE);
}

//http://extremeelectronics.co.in/avr-tutorials/using-the-analog-to-digital-converter/
//10bit conversion replaces analogread command
//https://github.com/kehribar/proper-tiny-arduino/blob/master/pta/analog.c
uint16_t ReadADC(uint8_t channel,uint8_t refs)
{
  //attiny datasheet 17.5 p126
  ADMUX=0;
  //SETBIT(ADMUX,refs);
  SETBITMASK(ADMUX,channel);  
  //Start Single conversion (vs continuous auto-trigger)
  SETBIT(ADCSRA,ADSC); //ADCSRA |= (1 << ADSC);
// 
  //Wait for conversion to complete 
  while(CHECKBIT(ADCSRA,ADSC));
  //Set ADIF to 1 to clear bin in IO standard way of clearing adif see 
  SETBIT(ADCSRA,ADIF);//ADCSRA |= (1 << ADIF); 17.4 p124 of datasheet
  _delay_us(300);
  return(ADC);
}
uint16_t mapp(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
