#ifndef __DM_LS_MODE_HPP
#define __DM_LS_MODE_HPP

#include "dm_motor.hpp"
#include "bsp_fdcan.hpp"
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef __cplusplus
}
#endif

namespace DM_Motor_n
{
    class DM_LS_Mode_c:public DM_Motor_Main_c,public BSP_CAN_Part_n::FDCANInstance_c
    {
        private:
        void TxDataProcessing(float postion,float velocity); 
        public:
        DM_LS_Mode_c(FDCAN_HandleTypeDef *fdcan_handle,
                      DM_ModePrame_s *param_, 
                      uint32_t tx_id, 
                      uint32_t rx_id,
                      uint32_t SAND_IDE
                      );
        void Transmit(float postion,float velocity);
        void StartStateSet(DM_StartState_e State);
    };
}




#endif /*__DM_LS_MODE_HPP*/



