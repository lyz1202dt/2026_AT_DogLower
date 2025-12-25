#include "ramp_simple.h"

/**
 * @brief 更新斜坡值，使其逐步接近目标值
 * @param ramp 斜坡结构体指针
 * @retval 1: 还未到达目标值, 0: 已到达目标值
 */
uint8_t SimpleRamp_Update(SimpleRamp_t *ramp)
{
    
    // 根据当前值和目标值的关系，决定增加还是减少
    if (ramp->current_value < ramp->target_value) {
        // 当前值小于目标值，增加
        ramp->current_value += ramp->step_size;
        
        // 检查是否超过目标值
        if (ramp->current_value >= ramp->target_value) {
            ramp->current_value = ramp->target_value;
            ramp->is_running = 0;
            return 0; // 已到达目标值
        }
    }
    else if (ramp->current_value > ramp->target_value) {
        // 当前值大于目标值，减少
        ramp->current_value -= ramp->step_size;
        
        // 检查是否小于目标值
        if (ramp->current_value <= ramp->target_value) {
            ramp->current_value = ramp->target_value;
            ramp->is_running = 0;
            return 0; // 已到达目标值
        }
    }
    else {
        // 当前值等于目标值
        ramp->is_running = 0;
        return 0;
    }
    
    return 1; // 未到达目标值
}

/**
 * @brief 停止斜坡变化
 * @param ramp 斜坡结构体指针
 */
void SimpleRamp_Stop(SimpleRamp_t *ramp)
{
    ramp->is_running = 0;
}

/**
 * @brief 获取当前值
 * @param ramp 斜坡结构体指针
 * @retval 当前值
 */
float SimpleRamp_GetValue(SimpleRamp_t *ramp)
{
    return ramp->current_value;
}