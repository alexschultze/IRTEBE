#define PIN_INFO1 10
#define PIN_INFO2 11
#define PIN_ERROR 12

struct sMeasurement_t {
   uint32_t  cnt; // incrementing count
   uint32_t  ts; //micros, timestamp when measurement was packaged
   uint16_t status;
   uint16_t error;
   uint16_t trig;
   
   int16_t  adc[5];
   uint32_t  adc_ts[5];

   int16_t  acc[3];
   uint32_t  acc_ts;
  
   int32_t  strain[3];
   uint32_t  strain_ts;
   
   float  sensor[3];
   uint32_t  sensor_ts[3]; 
     
   int32_t  pos[4];
   int16_t v_bat;   
};
