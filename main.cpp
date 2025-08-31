#include "mbed.h"

BufferedSerial uart0(P1_7, P1_6,9600);
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
DigitalOut cs1(P0_7);   //ADF4360 CS
DigitalOut gpio1(P1_5); //ADF4351_1 CS
DigitalOut gpio3(P1_9); //ADF4351_2 CS
DigitalIn sda(P0_5); //ADF4351 LD
DigitalOut gpio2(P1_8); //ATT_1 CS
DigitalOut scl(P0_4); //ATT_2 CS
DigitalOut iou(P0_2);   //io update
DigitalOut rst(P1_2);    //dds reset
DigitalOut cs2(P0_3);   //ch1 dds
DigitalOut cs3(P1_1);   //ch2 dds
DigitalOut cs4(P1_4);   //ch3 dds
DigitalOut cs5(P1_0);   //fs dac
AnalogIn ain0(P0_11);   //analog in

//CS control
void cs_hi(uint8_t num);
void cs_lo(uint8_t num);
void spi_send(uint32_t in);
//DDS control
uint32_t pll_data[3]={0x1900A,0x4FF1A4,0x300065};  //N, CON, R, f=400MHz, PFD=1MHz
void ddsset(uint8_t ch, uint32_t freq, uint16_t pha, uint16_t ampl);
const uint32_t mclk=400000000;//671088640;
const uint64_t res=4294967296;  //2^32
uint8_t buf;
const uint8_t CFR1=0x00;    //4B
const uint8_t ASF=0x02;     //2B
const uint8_t FTW0=0x04;    //4B
const uint8_t POW0=0x05;    //2B

//PLL control
const uint8_t fpfd=25;
const uint16_t nmod=25000;  //fpfd/spacing(0.001MHz)
uint32_t freq;              //kHz
int8_t ampl;               //means att state
void pllset(uint8_t ch,uint32_t freq,uint8_t att);
uint8_t k;
uint32_t data[6]={0,0,0x4E42,0x4B3,0,0x580005};
char local_buf[2];
uint8_t locked;
uint8_t att;

//analog out
uint16_t a1;

//parser
char char_read();
float char2flac();
void parser();
float pars[4];
uint32_t a,b,c,d;

int main(){
    iou=0;
    rst=1;
    cs1=1;
    cs2=1;
    cs3=1;
    cs4=1;
    cs5=1;
    gpio1=1;
    gpio2=1;
    gpio3=1;
    scl=1;
    spi.format(8,0);   //spi mode setting. 2byte(16bit) transfer, mode 2
    rst=0;
    thread_sleep_for(100);

    //pll set
    thread_sleep_for(3);
    cs1=0;
    spi_send(pll_data[2]);
    cs1=1;
    thread_sleep_for(3);
    cs1=0;
    spi_send(pll_data[1]);
    cs1=1;
    thread_sleep_for(20);
    cs1=0;
    spi_send(pll_data[0]);
    cs1=1;
    thread_sleep_for(100);

    //dds init
    cs2=0;
    cs3=0;
    cs4=0;
    buf=CFR1;
    spi.write(buf);
    buf=0<<1;           //OSK disable 0
    spi.write(buf);
    buf=0;
    spi.write(buf);
    buf=1<<5;           //accum. auto clear 
    spi.write(buf);
    buf=0;
    spi.write(buf);
    cs2=1;
    cs3=1;
    cs4=1;
    iou=1;
    iou=0;

    while (true){
        buf=char_read();
        if(buf=='S'){
            parser();
        }else if(buf=='?'){
            a1=ain0.read_u16()*3300/65535;  //mV unit
            locked=sda;
            printf("%d,%d,%04d,%04d\n\r",locked,0,a1,0);
        }
        a=(uint32_t)pars[0];
        b=(uint32_t)pars[1];
        c=(uint32_t)pars[2];
        d=(uint32_t)pars[3];

        if(a==1){
            ddsset(1,b,c,d);
            ddsset(3,b,0,d);
            iou=1;
            iou=0;
        }else if(a==2){
            ddsset(2,b,c,d);
            //ddsset(3,b,0,d);
            iou=1;
            iou=0;
        }else if(a==5){
            pllset(1,b,c);
        }else if(a==6){
            pllset(2,b,c);
        }
    }
}

//cs control func.
void cs_hi(uint8_t num){
    if(num==1) cs2=1;
    else if(num==2) cs3=1;
    else if(num==3) cs4=1;
}
void cs_lo(uint8_t num){
    if(num==1) cs2=0;
    else if(num==2) cs3=0;
    else if(num==3) cs4=0;
}
void spi_send(uint32_t in){
    uint8_t buf;
    buf=(in>>24)&0xff;
    spi.write(buf);
    buf=(in>>16)&0xff;
    spi.write(buf);
    buf=(in>>8)&0xff;
    spi.write(buf);
    buf=(in>>0)&0xff;
    spi.write(buf);
}
void ddsset(uint8_t ch, uint32_t freq, uint16_t pha, uint16_t ampl){
    uint8_t buf;
    uint32_t reg;
    float ampl_f;
    uint16_t dac_val;
    if(freq>180000000)freq=180000000;
    if(pha>360)pha=360;
    if(ampl>=2000)ampl=2000;
    
    reg=freq*res/mclk;
    cs_lo(ch);
    buf=FTW0;
    spi.write(buf);
    buf=(reg>>24)&0xff;
    spi.write(buf);
    buf=(reg>>16)&0xff;
    spi.write(buf);
    buf=(reg>>8)&0xff;
    spi.write(buf);
    buf=reg&0xff;
    spi.write(buf);
    cs_hi(ch);

    reg=pha*16384/360;
    cs_lo(ch);
    buf=POW0;
    spi.write(buf);
    buf=(reg>>8)&0x3f;
    spi.write(buf);
    buf=reg&0xff;
    spi.write(buf);
    cs_hi(ch);

    ampl_f=16.51507*(float)(2000-ampl);   //16.51507 is a mgic number for translate
    dac_val=(uint16_t)ampl_f;
    if(ch==1||ch==2){
        spi.format(8,1);
        cs5=0;
        if(ch==1)spi.write(0b100110);   //addr chd
        if(ch==2)spi.write(0b100100);   //addr chd
        spi.write((dac_val>>8)&0xff);
        spi.write(dac_val&0xff);
        cs5=1;
        spi.format(8,0);
    }
}
void pllset(uint8_t ch,uint32_t freq, uint8_t att){
    uint8_t i,j,ndiv,att_buf;
    uint16_t nint,nmodulus,nfrac_i,r,a,b;
    float freq_f,temp,nfrac;
    if(freq>4400000)freq=4400000;
    if(freq<35)freq=35;
    i=0;
    ndiv=2;
    freq_f=(float)freq;
    freq_f=freq_f/1000; //change MHz unit
    temp=4400/freq_f;   //for nint calc.
    while(1){
        temp=temp/2;
        i++;
        if(temp<2)break;
    }
    for(j=0;j<i-1;++j){
        ndiv=ndiv*2;
    }
    nint=freq*ndiv/fpfd/1000;
    temp=freq_f*ndiv/fpfd;  //for nfrac calc
    nfrac=nmod*(temp-nint);
    nfrac_i=(uint16_t)round(nfrac);

    //calc gcd by euclid algorithm
    a=nmod;
    b=nfrac_i;
    r=a%b;
    while(r!=0){
        a=b;
        b=r;
        r=a%b;
    }
    
    //calc nfrac and nmodulus
    nfrac_i=nfrac_i/b;
    nmodulus=nmod/b;
    if(nmodulus==1)nmodulus=2;
    
    //calc reg.
    data[0]=0;
    data[1]=0;
    data[4]=0;
    data[0]=(nint<<15)+(nfrac_i<<3)+0;
    data[1]=(1<<27)+(1<<15)+(nmodulus<<3)+1;
    data[4]=(1<<23)+(i<<20)+(200<<12)+(1<<5)+(3<<3)+4;
    
    //spi send
    for(i=0;i<5;++i){
        if(ch==1)gpio1=0;
        else gpio3=0;
        spi_send(data[5-i]);
        thread_sleep_for(3);
        if(ch==1)gpio1=1;
        else gpio3=1;
    }
    thread_sleep_for(20);
    if(ch==1)gpio1=0;
    else gpio3=0;
    spi_send(data[0]);
    if(ch==1)gpio1=1;
    else gpio3=1;

    //att
    att_buf=((att&0b1)<<5)+((att&0b10)<<3)+((att&0b100)<<1)+((att&0b1000)>>1)+((att&0b10000)>>3); //byte swapping
    if(ch==1)gpio2=0;
    else scl=0;
    spi.write(att_buf);
    if(ch==1)gpio2=1;
    else scl=1;
}

char char_read(){
    char local_buf[1];          //local buffer
    uart0.read(local_buf,1);    //1-char read
    return local_buf[0];        //return 1-char
}
float char2flac(){
    char temp[1],local_buf[20];          //local buffer
    uint8_t i;
    for(i=0;i<sizeof(local_buf);++i) local_buf[i]='\0'; //init local buf
    i=0;
    while(true){
        temp[0]=char_read();
        if(temp[0]==',') break; //',' is delimiter
        local_buf[i]=temp[0];
        ++i;
    }
    return atof(local_buf);
}
void parser(){
    uint8_t i=0;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
    ++i;
    pars[i]=char2flac();
}