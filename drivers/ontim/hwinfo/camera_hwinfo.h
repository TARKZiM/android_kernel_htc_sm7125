/*************************************************************************
    > File Name: camera_hwinfo.h
    > Author: NiCX
    > Mail: cuixia.niS@ontim.cn
    > Created Time: 2020年2月29日 星期刘
 ************************************************************************/
#ifndef _LINUX_CAMERA_HWINFO_H_
#define _LINUX_CAMERA_HWINFO_H_

//#define CAMERA_HWINFO_ID
#ifdef  HWINFO_C
char front_cam_name[64] = "Unknown";
char frontaux_cam_name[64] = "Unknown";
char back_cam_name[64] = "Unknown";
char backaux_cam_name[64] = "Unknown";
char backaux2_cam_name[64] = "Unknown";
char backaux3_cam_name[64] = "Unknown";
char front_cam_efuse_id[64] = {0};
char frontaux_cam_efuse_id[64] = {0};
char back_cam_efuse_id[64] = {0};
char backaux_cam_efuse_id[64] = {0};
char backaux2_cam_efuse_id[64] = {0};
char backaux3_cam_efuse_id[64] = {0};
#if 0
EXPORT_SYMBOL(front_cam_name);
EXPORT_SYMBOL(frontaux_cam_name);
EXPORT_SYMBOL(back_cam_name);
EXPORT_SYMBOL(backaux_cam_name);
EXPORT_SYMBOL(backaux2_cam_name);
EXPORT_SYMBOL(front_cam_efuse_id);
EXPORT_SYMBOL(frontaux_cam_efuse_id);
EXPORT_SYMBOL(back_cam_efuse_id);
EXPORT_SYMBOL(backaux_cam_efuse_id);
EXPORT_SYMBOL(backaux2_cam_efuse_id);
#endif
#else
extern char front_cam_name[];
extern char frontaux_cam_name[];
extern char back_cam_name[];
extern char backaux_cam_name[];
extern char backaux2_cam_name[];
extern char front_cam_efuse_id[];
extern char frontaux_cam_efuse_id[];
extern char back_cam_efuse_id[];
extern char backaux_cam_efuse_id[];
extern char backaux2_cam_efuse_id[];
#endif

#endif
