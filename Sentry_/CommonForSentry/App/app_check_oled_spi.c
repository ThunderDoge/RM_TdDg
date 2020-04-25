/**
 * @file      app_check_oled.c
 * @brief     离线检测·OLED液晶屏幕显示支持包
 * @details   
 * @author   ThunderDoge
 * @date      2020-4-19
 * @version   v0.1
 * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020 
 * Using encoding: utf-8
 */
#include "app_check_oled_spi.h"

app_check_oled_check_box_list GlobalBoxList;    /// 全局OLED显示设备列表



/**
 * @brief 初始化OLED显示
 * 这个函数会清空所有的OLED显示设备列表。所以请在这函数之后加入你要显示的设备
 */
void app_check_oled_Init()
{
    bsp_oled_Init();
    bsp_oled_Show_Logo();
    bsp_oled_Display_On();
    GlobalBoxList.BitOfBoxInUse=0U;
}
/**
 * @brief 
 * 
 * @param     device_id 
 * @param     device_name 
 * @param     position_at_list 填入哪个BOX，可选0~9. 另外填入 __OLED_SELECT_LAST_BOX 选择最靠前的可用的BOX
 * @return HAL_StatusTypeDef 返回状态 HAL_OK正常 HAL_ERROR表示已满无法添加
 */
HAL_StatusTypeDef app_check_oled_EnableDevice(DeviceIdEnum device_id, uint8_t* device_name,int position_at_list)
{
    if(position_at_list == __OLED_SELECT_LAST_BOX)
    {
        for(int i=0;i<__OLED_BOX_LIST_LENGTH;i++)
        {
            if(! ( (GlobalBoxList.BitOfBoxInUse >> i) & 0x1) )
            {
                GlobalBoxList.Box[i].DeviceName = device_name;
                GlobalBoxList.Box[i].isOfflineVar = &GlobalCheckList.IsOfflineList[device_id];
                GlobalBoxList.BitOfBoxInUse |= (1U << i);
                return HAL_OK;
            }
        }
    }
    else if(position_at_list <__OLED_BOX_LIST_LENGTH && position_at_list>=0)
    {
        if(! ( (GlobalBoxList.BitOfBoxInUse >> position_at_list) & 0x1) )
        {
            GlobalBoxList.Box[position_at_list].DeviceName = device_name;
            GlobalBoxList.Box[position_at_list].isOfflineVar = &GlobalCheckList.IsOfflineList[device_id];
            GlobalBoxList.BitOfBoxInUse |= (1U << position_at_list);
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}
/**
 * @brief 除能一个BOX里的设备
 * 
 * @param     position_at_list 
 */
void app_check_oled_DisableDevice(int position_at_list)
{
    if(position_at_list == __OLED_SELECT_ALL_BOX)   // 停用全部BOX
    {
        GlobalBoxList.BitOfBoxInUse &= 0U;  // 全部位置零
    }
    if(position_at_list == __OLED_SELECT_LAST_BOX)  // 停用最后一个BOX
    {
        for(int i= __OLED_BOX_LIST_LENGTH-1;i>=0;i--)
        {
            if( ( (GlobalBoxList.BitOfBoxInUse >> i) & 0x1) )
            {
                GlobalBoxList.BitOfBoxInUse &= (~( 0x1 << i )); // 将第i位置0，其他保留不变
            }
        }
    }
    if( position_at_list <=__OLED_BOX_LIST_LENGTH && position_at_list>=0 )  // 停用指定位置BOX
    {
        GlobalBoxList.BitOfBoxInUse &= (~( 0x1 << position_at_list )); // 将第i位置0，其他保留不变
    }
}
/**
 * @brief 把BOX画在现存上，并且输出
 * 
 */
void app_check_oled_Show_BoxList()
{
    bsp_oled_ClearGram();
    for(int i=0;i<__OLED_BOX_LIST_LENGTH;i++)
    {
        if( GET_BIT_N_OF_X(i,GlobalBoxList.BitOfBoxInUse) )
        {
            if(i<5)
            {
                bsp_oled_Show_String(1,i*12,GlobalBoxList.Box[i].DeviceName);
                bsp_oled_Show_Graphic(50,i*12,&check_box[ *GlobalBoxList.Box[i].isOfflineVar ] );
            }
            else if(i<10)
            {
                bsp_oled_Show_String(64,(i-5)*12,GlobalBoxList.Box[i].DeviceName);
                bsp_oled_Show_Graphic(114,(i-5)*12,& check_box[ *GlobalBoxList.Box[i].isOfflineVar ] );
            }
        }
    }
	bsp_oled_Refresh_Gram();
    return;
}
