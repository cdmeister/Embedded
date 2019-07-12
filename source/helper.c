#include "helper.h"

void RangeMap_init(RangeMap * RangeMapx,double input_start, double input_end,
                   double output_start,double output_end){

  RangeMapx->_input_start = input_start;
  RangeMapx->_input_end = input_end;
  RangeMapx->_output_start = output_start;
  RangeMapx->_output_end = output_end;

  RangeMapx->_slope = 1.0 * (output_end - output_start) / (input_end - input_start);


}

double RangeMap_map(RangeMap * RangeMapx, double input){

  return RangeMapx->_output_start + RangeMapx->_slope * (input - RangeMapx->_input_start);

}
