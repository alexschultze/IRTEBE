struct sMeasurement {
   uint32_t  cnt; // incrementing count
   uint32_t  ts; //micros, timestamp when measurement was packaged
   uint16_t stat;
   uint16_t trig;
   
   int16_t  adc[5];
   uint32_t  adc_ts[5];
  
   int32_t  strain[3];
   uint32_t  strain_ts[5];
   
   int32_t  sensor[3];
   uint32_t  sensor_ts[3]; 
     
   int32_t  pos[4];
   int16_t v_bat;   
};
